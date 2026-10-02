#!/usr/bin/env python3
"""Drive ctl-px4 SMC against a lightweight FS150 plant in two Zenoh hosts."""

import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Imu
from hover_thrust_estimator_msgs.msg import HoverThrustEstimate
from multirotor_reference_trajectory_msgs.msg import FlatReferencePoint, SampledReference
from std_msgs.msg import Float64MultiArray, String


STEP_NS = 100_000_000
TRAJECTORY_NS = 6_000_000_000
TRACKING_NS = 10_000_000_000
RUN_LIMIT_S = 60.0
CHANNELS = '''[[channel]]
name = "pose"
qos = "state"
[[channel]]
name = "velocity"
qos = "state"
[[channel]]
name = "imu"
qos = "state"
[[channel]]
name = "fcu_state"
qos = "state"
[[channel]]
name = "setpoint"
qos = "control"
[[channel]]
name = "fcu_request"
qos = "event"
[[channel]]
name = "command"
qos = "event"
[[channel]]
name = "alg_setpoint"
qos = "control"
[[channel]]
name = "status"
qos = "state"
[[channel]]
name = "attitude_command"
qos = "control"
[[channel]]
name = "attitude_target"
qos = "state"
[[channel]]
name = "hover_thrust"
qos = "state"
[[channel]]
name = "ref_sampled"
qos = "event"
[[channel]]
name = "fcu_result"
qos = "event"
[[channel]]
name = "fcu_extended_state"
qos = "state"
'''


def absolute_file(parser, option, value):
    try:
        path = value.expanduser().resolve(strict=True)
    except (OSError, RuntimeError) as error:
        parser.error(f"{option} does not resolve to an existing file: {error}")
    if not path.is_file():
        parser.error(f"{option} must name a file: {path}")
    return path


def zenoh_loopback_endpoint(value):
    match = re.fullmatch(r"tcp/127\.0\.0\.1:([1-9][0-9]{0,4})", value)
    if match is None or not 1 <= int(match.group(1)) <= 65535:
        raise argparse.ArgumentTypeError(
            "must be a loopback Zenoh endpoint in the form tcp/127.0.0.1:PORT"
        )
    return value


def make_plant_manifest(plant_path, ros_io_path, audit_dir, epoch_ns, plant_endpoint,
                        controller_endpoint, thr_mdl_fac=0.0):
    quoted = lambda value: json.dumps(str(value))
    hover = physical_hover_thrust(thr_mdl_fac)['hover_thrust_command']
    fcu_parameters = f', fcu_parameters = {{ MPC_THR_HOVER = {hover}, THR_MDL_FAC = {thr_mdl_fac} }}'
    return f'''[session]
id = "private-lightweight-controller-live"
node = "plant"
roster = ["plant", "uav1"]
period_ms = 1
epoch_ns = {epoch_ns}
run_for_ms = 60000

[transport]
kind = "zenoh"
listen = [{quoted(plant_endpoint)}]
connect = [{quoted(controller_endpoint)}]

[audit]
dir = {quoted(audit_dir)}

{CHANNELS}
[[plugin]]
name = "plant"
path = {quoted(plant_path)}
trigger = "on_round"
config = {{ model = "fs150", epoch_ns = {epoch_ns}, step_ms = 1, output_ms = 10, initial_pose = [0.0, 0.0, 0.0, 0.0]{fcu_parameters} }}
bind = {{ pose = {{ channel = "pose" }}, velocity = {{ channel = "velocity" }}, imu = {{ channel = "imu" }}, fcu_state = {{ channel = "fcu_state" }}, attitude_target = {{ channel = "attitude_target" }}, fcu_result = {{ channel = "fcu_result" }}, fcu_extended_state = {{ channel = "fcu_extended_state" }}, setpoint = {{ channel = "setpoint", from = ["uav1"] }}, attitude_command = {{ channel = "attitude_command", from = ["uav1"] }}, fcu_request = {{ channel = "fcu_request", from = ["uav1", "plant"] }} }}

[[plugin]]
name = "plant-ros-feedback"
path = {quoted(ros_io_path)}
trigger = "on_round"
config = {{ node_name = "xgc_ros_feedback_plant", frame_id = "world", sim_odometry_child_frame = "uav1/base_link", sim_imu_orientation_from_pose = true, sim_pose_topic = "/uav1/mavros/local_position/pose", sim_velocity_topic = "/uav1/mavros/local_position/velocity_local", sim_imu_topic = "/uav1/mavros/imu/data", sim_fcu_state_topic = "/uav1/mavros/state", sim_attitude_target_topic = "/uav1/mavros/setpoint_raw/target_attitude", sim_fcu_request_topic = "/uav1/mavros", sim_fcu_robot_index = 0, sim_extended_state_topic = "/uav1/mavros/extended_state" }}
bind = {{ sim_pose = {{ channel = "pose", from = ["plant"] }}, sim_velocity = {{ channel = "velocity", from = ["plant"] }}, sim_imu = {{ channel = "imu", from = ["plant"] }}, sim_fcu_state = {{ channel = "fcu_state", from = ["plant"] }}, sim_attitude_target = {{ channel = "attitude_target", from = ["plant"] }}, sim_fcu_request = {{ channel = "fcu_request" }}, sim_fcu_result = {{ channel = "fcu_result", from = ["plant"] }}, sim_extended_state = {{ channel = "fcu_extended_state", from = ["plant"] }} }}
'''


def make_controller_manifest(controller_path, ros_io_path, hte_path, audit_dir, epoch_ns,
                             plant_endpoint, controller_endpoint, backend, hte_initial):
    quoted = lambda value: json.dumps(str(value))
    estimator_config = '' if hte_initial is None else f', initial_hover_thrust = {hte_initial}'
    return f'''[session]
id = "private-lightweight-controller-live"
node = "uav1"
roster = ["plant", "uav1"]
period_ms = 1
epoch_ns = {epoch_ns}
run_for_ms = 60000

[transport]
kind = "zenoh"
listen = [{quoted(controller_endpoint)}]
connect = [{quoted(plant_endpoint)}]

[audit]
dir = {quoted(audit_dir)}

{CHANNELS}
[[plugin]]
name = "controller"
path = {quoted(controller_path)}
trigger = "on_round"
config = {{ time_source = "session", tracking_backend = "{backend}", takeoff_altitude = 1, planning_period = 0.1 }}
bind = {{ local_pose = {{ channel = "pose", from = ["plant"] }}, vrpn_pose = {{ channel = "pose", from = ["plant"] }}, local_velocity = {{ channel = "velocity", from = ["plant"] }}, imu = {{ channel = "imu", from = ["plant"] }}, fcu_state = {{ channel = "fcu_state", from = ["plant"] }}, command = {{ channel = "command", from = ["uav1"] }}, alg_setpoint = {{ channel = "alg_setpoint", from = ["uav1"] }}, ref_active_sampled = {{ channel = "ref_sampled", from = ["uav1"] }}, hover_thrust = {{ channel = "hover_thrust", from = ["uav1"] }}, setpoint = {{ channel = "setpoint" }}, attitude_command = {{ channel = "attitude_command" }}, fcu_request_full = {{ channel = "fcu_request" }}, status = {{ channel = "status" }} }}

[[plugin]]
name = "hover-thrust-estimator"
path = {quoted(hte_path)}
trigger = "both"
config = {{ time_source = "session"{estimator_config} }}
bind = {{ imu = {{ channel = "imu", from = ["plant"] }}, pose = {{ channel = "pose", from = ["plant"] }}, attitude_target_full = {{ channel = "attitude_target", from = ["plant"] }}, hover_thrust = {{ channel = "hover_thrust" }} }}

[[plugin]]
name = "ros-io"
path = {quoted(ros_io_path)}
trigger = "on_round"
config = {{ node_name = "xgc_ros_io_uav1", frame_id = "world", command_topic = "/command", alg_setpoint_topic = "/uav1/alg/setpoint_raw/local", ref_sampled_topic = "/uav1/alg/multirotor_reference_trajectory/active/sampled", sim_hover_thrust_topic = "/uav1/hover_thrust/estimate_state", sim_hover_thrust_trace_topic = "/uav1/hover_thrust/native_trace", status_topic = "/uav1/custom/statustext" }}
bind = {{ command = {{ channel = "command" }}, alg_setpoint = {{ channel = "alg_setpoint" }}, ref_sampled = {{ channel = "ref_sampled" }}, sim_hover_thrust = {{ channel = "hover_thrust", from = ["uav1"] }}, status = {{ channel = "status", from = ["uav1"] }} }}
'''


def position_target(elapsed_ns, stamp_ns):
    position, velocity, acceleration, _, _ = trajectory_values(elapsed_ns / 1e9)
    message = PositionTarget()
    message.header.stamp = rospy.Time(stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000)
    message.header.frame_id = "map"
    message.coordinate_frame = 1
    message.type_mask = 3072
    for field, values in ((message.position, position), (message.velocity, velocity),
                          (message.acceleration_or_force, acceleration)):
        field.x, field.y, field.z = values
    return message


def trajectory_values(elapsed):
    """Six-second quintic plus a 0.25 m lateral bump; endpoint remains (1,0,1)."""
    q = min(1.0, max(0.0, elapsed / 6.0))
    coefficients = ((0, 0, 0, 10, -15, 6), (0, 0, 0, 16, -48, 48, -16), (1,))
    derivatives = []
    for order in range(5):
        values = []
        for coeff in coefficients:
            value = sum(coeff[k] * math.factorial(k) / math.factorial(k - order)
                        * q ** (k - order) for k in range(order, len(coeff))) / 6.0**order
            values.append(0.0 if order and elapsed >= 6 else value)
        derivatives.append(values)
    return derivatives


def physical_hover_thrust(thr_mdl_fac):
    """Fixed FS150 kf/mass + PX4 armed idle/scale; THR_MDL_FAC is not a mass tuning knob."""
    omega = math.sqrt(.310 * 9.8066 / (4 * 5.33969944334e-6))
    motor = (omega - 100.0) / 1000.0
    command = (1-thr_mdl_fac)*motor + thr_mdl_fac*motor*motor
    return {'hover_omega_rad_s': omega, 'hover_motor_normalized': motor,
            'thr_mdl_fac': thr_mdl_fac, 'hover_thrust_command': command,
            'provenance': 'FS150 Iris-equivalent mass/kf unchanged; fixed PX4 omega_target=100+1000*m armed'}


def sampled_reference(start_ns):
    message = SampledReference()
    message.header.stamp = rospy.Time(start_ns // 10**9, start_ns % 10**9)
    message.header.frame_id = 'map'
    message.start_time = message.header.stamp
    message.trajectory_id = message.revision = 1
    message.sample_dt = 0.01
    for index in range(1201):
        point = FlatReferencePoint()
        point.t_from_start = index * message.sample_dt
        for field, values in zip((point.position, point.velocity, point.acceleration,
                                  point.jerk, point.snap), trajectory_values(point.t_from_start)):
            field.x, field.y, field.z = values
        message.points.append(point)
    return message


def qproduct(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def conjugate(q):
    return [q[0], -q[1], -q[2], -q[3]]


def rotate(q, v):
    return qproduct(qproduct(q, [0.0, *v]), conjugate(q))[1:]


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def verify_elf_interfaces(paths):
    """Reject mixed wrapper versions before starting either actual Host."""
    class Port(ctypes.Structure):
        _fields_ = [('name', ctypes.c_char_p), ('direction', ctypes.c_int),
                    ('schema', ctypes.c_char_p), ('qos', ctypes.c_int)]
    class Descriptor(ctypes.Structure):
        _fields_ = [('abi', ctypes.c_uint32), ('count', ctypes.c_uint32),
                    ('name', ctypes.c_char_p), ('version', ctypes.c_char_p),
                    ('ports', ctypes.POINTER(Port)), ('vtable', ctypes.c_void_p)]
    declarations = {}
    for role, path in paths.items():
        library = ctypes.CDLL(str(path), mode=ctypes.RTLD_LOCAL)
        entry = library.xgc_rt_plugin_v1
        entry.restype = ctypes.POINTER(Descriptor)
        descriptor = entry().contents
        if descriptor.abi != 1 or descriptor.count > 64:
            raise AssertionError(f'{role} unsupported ABI/port capacity')
        declarations[role] = {descriptor.ports[i].name.decode(): descriptor.ports[i].schema.decode()
                              for i in range(descriptor.count)}
    required = {'plant': {'pose': 'xgc.pose/1', 'velocity': 'xgc.twist/1', 'imu': 'xgc.imu/1',
                          'attitude_command': 'xgc.attitude_target/2', 'attitude_target': 'xgc.attitude_target/2',
                          'fcu_request': 'xgc.fcu_request/2', 'fcu_result': 'xgc.fcu_result/1',
                          'fcu_extended_state': 'xgc.fcu_extended_state/1'},
                'controller': {'attitude_command': 'xgc.attitude_target/2', 'fcu_request_full': 'xgc.fcu_request/2',
                               'ref_active_sampled': 'xgc.ref.sampled/1', 'hover_thrust': 'xgc.hover_thrust/1'},
                'hte': {'attitude_target_full': 'xgc.attitude_target/2', 'hover_thrust': 'xgc.hover_thrust/1'},
                'ros_io': {'sim_imu': 'xgc.imu/1', 'sim_attitude_target': 'xgc.attitude_target/2',
                           'sim_hover_thrust': 'xgc.hover_thrust/1', 'ref_sampled': 'xgc.ref.sampled/1',
                           'sim_fcu_request': 'xgc.fcu_request/2', 'sim_fcu_result': 'xgc.fcu_result/1',
                           'sim_extended_state': 'xgc.fcu_extended_state/1'}}
    for role, ports in required.items():
        for port, schema in ports.items():
            if declarations[role].get(port) != schema:
                raise AssertionError(f'{paths[role]}: {port} must be {schema}, actual {declarations[role].get(port)}')
    return declarations


def error_stats(values):
    if not values:
        return {'samples': 0, 'rms': None, 'p95': None, 'max': None}
    ordered = sorted(values)
    return {'samples': len(values), 'rms': math.sqrt(sum(v*v for v in values) / len(values)),
            'p95': ordered[math.ceil(.95*len(values))-1], 'max': ordered[-1]}


def analyse_flight(capture, custom_start_ns, expected_hover, hte_initial):
    """Independent kinematic checks on exact-stamp triples, never nearest ROS frames."""
    failures = []
    streams = [capture[key] for key in ('poses', 'velocities', 'imus')]
    for name, stream in zip(('pose', 'velocity', 'imu'), streams):
        stamps = [s['stamp_ns'] for s in stream]
        if not stamps or any(b <= a for a, b in zip(stamps, stamps[1:])):
            failures.append(name + ' source stamps are not strictly increasing')
    maps = [{s['stamp_ns']: s for s in stream} for stream in streams]
    stamps = sorted(set(maps[0]) & set(maps[1]) & set(maps[2]))
    coverage = []
    if stamps:
        for stream in streams:
            denominator = sum(stamps[0] <= s['stamp_ns'] <= stamps[-1] for s in stream)
            coverage.append(len(stamps) / max(1, denominator))
    if len(stamps) < 300 or not coverage or min(coverage) < .95:
        failures.append('insufficient exact-stamp pose/velocity/IMU coverage')
    flight_stamps = [t for t in stamps if custom_start_ns is not None
                     and custom_start_ns <= t <= custom_start_ns+TRACKING_NS]
    max_flight_gap_ns = max((b-a for a,b in zip(flight_stamps,flight_stamps[1:])),default=0)
    if (not flight_stamps or flight_stamps[0]-custom_start_ns > 30_001_000
            or custom_start_ns+TRACKING_NS-flight_stamps[-1] > 30_001_000
            or max_flight_gap_ns > 30_001_000):
        failures.append('trajectory lacks continuous exact-stamp coverage of the full ten-second interval')
    errors = {key: [] for key in ('quaternion_norm', 'imu_pose_quaternion', 'world_body_rate',
                                  'trajectory_position_m', 'trajectory_velocity_m_s',
                                  'q_derivative_gyro_rad_s', 'specific_force_derivative_m_s2')}
    tilt, roll, pitch, rate = [], [], [], []
    acceleration_checks = acceleration_aligned = 0
    finite_difference_gaps = 0
    for index, stamp in enumerate(stamps):
        pose, velocity, imu = (m[stamp] for m in maps)
        all_values = [*pose['position'], *pose['q_wxyz'], *velocity['linear'],
                      *velocity['angular'], *imu['q_wxyz'], *imu['gyro'], *imu['specific_force']]
        if not all(math.isfinite(value) for value in all_values):
            failures.append('nonfinite measured state/IMU')
            continue
        if pose['frame_id'] != 'world' or velocity['frame_id'] != 'world' or imu['frame_id'] != 'uav1/base_link':
            failures.append('state or IMU frame mismatch')
        q = pose['q_wxyz']
        qi = imu['q_wxyz']
        if sum(a*b for a, b in zip(q, qi)) < 0:
            qi = [-v for v in qi]
        errors['quaternion_norm'].append(abs(norm(q)-1))
        errors['imu_pose_quaternion'].append(max(abs(a-b) for a, b in zip(q, qi)))
        if imu['orientation_covariance_0'] == -1:
            failures.append('imu/data must contain measured same-stamp orientation')
        errors['world_body_rate'].append(norm([a-b for a, b in zip(rotate(q, imu['gyro']), velocity['angular'])]))
        elapsed = (stamp-custom_start_ns)/1e9 if custom_start_ns else -1
        if not 0 <= elapsed <= 10:
            continue
        desired = trajectory_values(elapsed)
        errors['trajectory_position_m'].append(norm([a-b for a, b in zip(pose['position'], desired[0])]))
        errors['trajectory_velocity_m_s'].append(norm([a-b for a, b in zip(velocity['linear'], desired[1])]))
        z = rotate(q, [0, 0, 1])
        tilt.append(math.acos(max(-1.0, min(1.0, z[2]))))
        roll.append(abs(math.atan2(z[1], z[2])))
        pitch.append(abs(math.atan2(z[0], z[2])))
        rate.append(norm(imu['gyro']))
        if index == 0 or index+1 == len(stamps):
            continue
        tm, tp = stamps[index-1], stamps[index+1]
        if not custom_start_ns <= tm < tp <= custom_start_ns+TRACKING_NS:
            continue
        if min(maps[0][t]['position'][2] for t in (tm, stamp, tp)) < .3:
            continue  # predefined airborne interval excludes contact impulses
        hm, hp = (stamp-tm)/1e9, (tp-stamp)/1e9
        if hm > .030001 or hp > .030001:
            finite_difference_gaps += 1
            continue
        weights = [-hp/(hm*(hm+hp)), (hp-hm)/(hm*hp), hm/(hp*(hm+hp))]
        qs = [maps[0][t]['q_wxyz'][:] for t in (tm, stamp, tp)]
        for quaternion in qs:
            if sum(a*b for a, b in zip(q, quaternion)) < 0:
                quaternion[:] = [-v for v in quaternion]
        qdot = [sum(w*x[j] for w, x in zip(weights, qs)) for j in range(4)]
        omega = [2*v for v in qproduct(conjugate(q), qdot)[1:]]
        errors['q_derivative_gyro_rad_s'].append(norm([a-b for a, b in zip(omega, imu['gyro'])]))
        acceleration = [sum(w*maps[1][t]['linear'][j] for w, t in zip(weights, (tm, stamp, tp))) for j in range(3)]
        force = rotate(q, imu['specific_force'])
        force[2] -= 9.8066
        errors['specific_force_derivative_m_s2'].append(norm([a-b for a, b in zip(force, acceleration)]))
        if math.hypot(*acceleration[:2]) > .08:
            acceleration_checks += 1
            acceleration_aligned += z[0]*acceleration[0]+z[1]*acceleration[1] > 0
    metrics = {key: error_stats(values) for key, values in errors.items()}
    gates = {'quaternion_norm': (1e-9, 1e-9), 'imu_pose_quaternion': (1e-9, 1e-9),
             'world_body_rate': (1e-8, 1e-8), 'q_derivative_gyro_rad_s': (.02, .04),
             'specific_force_derivative_m_s2': (.10, .20), 'trajectory_position_m': (.10, .20)}
    for key, (rms, p95) in gates.items():
        metric = metrics[key]
        if metric['rms'] is None or metric['rms'] >= rms or metric['p95'] >= p95:
            failures.append(key + ' exceeds declared RMS/P95 gate')
    if len(errors['q_derivative_gyro_rad_s']) < 300 or len(errors['trajectory_position_m']) < 500:
        failures.append('too few airborne trajectory/difference samples')
    if not roll or min(max(roll), max(pitch), max(rate)) <= .005:
        failures.append('trajectory must excite roll, pitch and measured gyro')
    if acceleration_checks < 100 or acceleration_aligned < .9*acceleration_checks:
        failures.append('attitude does not tilt with realized horizontal acceleration')
    trace = capture['hover_thrust_trace']
    initial_expected = .3 if hte_initial is None else hte_initial
    if not trace or any(abs(s['initial_hover_thrust']-initial_expected) > 1e-9 for s in trace):
        failures.append('real HTE did not retain its configured/original initial value')
    healthy = [s for s in trace if s['state_native'] == 12 and s['sample_used']
               and not s['flags'] & (1|2|4|8|16|32|64|128|256|4096|8192|65536)]
    tail = [s for s in healthy if s['stamp_s'] >= healthy[-1]['stamp_s']-2] if healthy else []
    if len(tail) < 10 or max(abs(s['hover_thrust']-expected_hover) for s in tail) >= .02:
        failures.append('real HTE did not converge to physical hover ratio within .02')
    ordinary = capture['hover_thrust']
    matched_hte = sum(any(abs(s['stamp_ns']/1e9-t['stamp_s']) <= 5e-7 and s['flags']==t['flags']
                          and abs(s['hover_thrust']-t['hover_thrust']) < 1e-12
                          and s['state'] == t['state_native']-10 for s in ordinary) for t in trace)
    if trace and matched_hte/len(trace) < .95:
        failures.append('HTE ordinary ROS message and complete native trace do not agree')
    return {'passed': not failures, 'failures': sorted(set(failures)), 'metrics': metrics,
            'exact_stamp_samples': len(stamps), 'exact_stamp_coverage': coverage,
            'trajectory_exact_stamp_span_s': ((flight_stamps[-1]-flight_stamps[0])/1e9 if flight_stamps else 0),
            'trajectory_max_exact_stamp_gap_s': max_flight_gap_ns/1e9,
            'finite_difference_excluded_gaps': finite_difference_gaps,
            'finite_difference_interval': 'Custom1 0..10s, all three heights >0.3m, adjacent gaps <=30ms',
            'derivative_method': 'unequal-step central difference; q sign aligned; omega_body=2*(conj(q)*qdot).vector; R(q)*f_body-g=dv_world/dt',
            'max_tilt_rad': max(tilt, default=0), 'max_roll_rad': max(roll, default=0),
            'max_pitch_rad': max(pitch, default=0), 'max_gyro_rad_s': max(rate, default=0),
            'acceleration_aligned': acceleration_aligned, 'acceleration_checks': acceleration_checks,
            'declared_rms_p95_gates': gates, 'hte_healthy_used_samples': len(healthy),
            'hte_tail_error': error_stats([abs(s['hover_thrust']-expected_hover) for s in tail]),
            'hte_initial_expected': initial_expected, 'hte_trace_ros_matches': matched_hte}


def main():
    parser = argparse.ArgumentParser(
        description="Exercise ctl-px4 SMC with a lightweight FS150 plant in two Zenoh-connected Hosts."
    )
    parser.add_argument("--host", required=True, type=Path, help="path to xgc-rt-host")
    parser.add_argument("--plant", required=True, type=Path, help="path to liblightweight_vehicle.so")
    parser.add_argument("--controller", required=True, type=Path, help="path to libctl_px4.so")
    parser.add_argument("--ros-io", required=True, type=Path, help="path to libros_io.so")
    parser.add_argument('--hte', required=True, type=Path, help='path to the real libest_hover_thrust.so')
    parser.add_argument('--backend', choices=('smc', 'dfbc'), default='smc')
    parser.add_argument('--thr-mdl-fac', type=float, default=0.0, help='fixed PX4 curve through the owning fcu_parameters block')
    parser.add_argument('--hte-initial', type=float, default=None, help='omit to retain upstream .3 default; probe uses .5')
    parser.add_argument("--plant-endpoint", required=True, type=zenoh_loopback_endpoint)
    parser.add_argument("--controller-endpoint", required=True, type=zenoh_loopback_endpoint)
    parser.add_argument(
        "--output-dir", required=True, type=Path, help="directory for manifests, logs, audits, and result JSON"
    )
    args = parser.parse_args()
    if args.plant_endpoint == args.controller_endpoint:
        parser.error("--plant-endpoint and --controller-endpoint must use different ports")

    host_path = absolute_file(parser, "--host", args.host)
    plant_path = absolute_file(parser, "--plant", args.plant)
    controller_path = absolute_file(parser, "--controller", args.controller)
    ros_io_path = absolute_file(parser, "--ros-io", args.ros_io)
    hte_path = absolute_file(parser, '--hte', args.hte)
    elf_interfaces = verify_elf_interfaces({'plant': plant_path, 'controller': controller_path,
                                           'hte': hte_path, 'ros_io': ros_io_path})
    if not 0 <= args.thr_mdl_fac <= 1 or (args.hte_initial is not None and not .05 < args.hte_initial < .95):
        parser.error('THR_MDL_FAC must lie in [0,1]; optional HTE initial inside (.05,.95)')
    hover_physics = physical_hover_thrust(args.thr_mdl_fac)
    expected_hover = hover_physics['hover_thrust_command']
    work = args.output_dir.expanduser().resolve()

    started_wall_ns = time.time_ns()
    epoch_ns = started_wall_ns + 3_000_000_000
    deadline = time.monotonic() + RUN_LIMIT_S
    host_dirs = {role: work / role for role in ("plant", "uav1")}
    for host_dir in host_dirs.values():
        (host_dir / "audit").mkdir(parents=True, exist_ok=True)
    manifest_paths = {
        "plant": host_dirs["plant"] / "manifest.toml",
        "uav1": host_dirs["uav1"] / "manifest.toml",
    }
    audit_paths = {role: host_dirs[role] / "audit" for role in host_dirs}
    log_paths = {role: host_dirs[role] / "host.log" for role in host_dirs}
    manifest_paths["plant"].write_text(
        make_plant_manifest(
            plant_path,
            ros_io_path,
            audit_paths["plant"],
            epoch_ns,
            args.plant_endpoint,
            args.controller_endpoint,
            args.thr_mdl_fac,
        ),
        encoding="utf-8",
    )
    manifest_paths["uav1"].write_text(
        make_controller_manifest(
            controller_path,
            ros_io_path,
            hte_path,
            audit_paths["uav1"],
            epoch_ns,
            args.plant_endpoint,
            args.controller_endpoint,
            args.backend,
            args.hte_initial,
        ),
        encoding="utf-8",
    )

    capture_lock = threading.Lock()
    capture = {
        "controller_state": None,
        "controller_states": [],
        "fcu_state": None,
        "fcu_state_changes": [],
        "poses": [],
        "velocities": [],
        "imus": [],
        "attitude_targets": [],
        "hover_thrust": [],
        "hover_thrust_trace": [],
        'extended_states': [],
        'service_responses': [],
        "actions": [],
    }

    def stamp_ns(message):
        return message.header.stamp.to_nsec()

    def on_controller_state(message):
        received_ns = time.time_ns()
        with capture_lock:
            if message.data != capture["controller_state"]:
                capture["controller_states"].append(
                    {"state": message.data, "received_wall_time_ns": received_ns}
                )
                capture["controller_state"] = message.data

    def on_fcu_state(message):
        received_ns = time.time_ns()
        current = {
            "connected": bool(message.connected),
            "armed": bool(message.armed),
            "mode": message.mode,
            "stamp_ns": stamp_ns(message),
            "received_wall_time_ns": received_ns,
        }
        with capture_lock:
            previous = capture["fcu_state"]
            if previous is None or any(
                current[key] != previous[key] for key in ("connected", "armed", "mode")
            ):
                capture["fcu_state_changes"].append(current)
            capture["fcu_state"] = current

    def on_pose(message):
        item = {
            "stamp_ns": stamp_ns(message),
            "received_wall_time_ns": time.time_ns(),
            "frame_id": message.header.frame_id,
            "position": [message.pose.position.x, message.pose.position.y, message.pose.position.z],
            "q_wxyz": [message.pose.orientation.w, message.pose.orientation.x,
                       message.pose.orientation.y, message.pose.orientation.z],
        }
        with capture_lock:
            capture["poses"].append(item)

    def on_velocity(message):
        item = {
            "stamp_ns": stamp_ns(message),
            "received_wall_time_ns": time.time_ns(),
            "frame_id": message.header.frame_id,
            "linear": [message.twist.linear.x, message.twist.linear.y, message.twist.linear.z],
            "angular": [message.twist.angular.x, message.twist.angular.y, message.twist.angular.z],
        }
        with capture_lock:
            capture["velocities"].append(item)

    def on_imu(message):
        item = {'stamp_ns': stamp_ns(message), 'received_wall_time_ns': time.time_ns(),
                'frame_id': message.header.frame_id,
                'q_wxyz': [message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z],
                'gyro': [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z],
                'specific_force': [message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z],
                'orientation_covariance_0': message.orientation_covariance[0]}
        with capture_lock:
            capture['imus'].append(item)

    def on_target(message):
        item = {'stamp_ns': stamp_ns(message), 'received_wall_time_ns': time.time_ns(),
                'frame_id': message.header.frame_id, 'type_mask': message.type_mask,
                'q_wxyz': [message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z],
                'body_rate': [message.body_rate.x, message.body_rate.y, message.body_rate.z], 'thrust': message.thrust}
        with capture_lock:
            capture['attitude_targets'].append(item)

    def on_hover_thrust(message):
        item = {'stamp_ns': stamp_ns(message), 'received_wall_time_ns': time.time_ns(),
                'state': message.state, 'flags': message.flags, 'hover_thrust': message.hover_thrust}
        with capture_lock:
            capture['hover_thrust'].append(item)

    def on_hover_thrust_trace(message):
        fields = ('stamp_s', 'hover_thrust', 'raw_hover_thrust', 'initial_hover_thrust',
                  'thrust_to_acceleration', 'last_estimate_stamp', 'state_native', 'flags', 'sample_used')
        if len(message.data) != 9 or not message.layout.dim or message.layout.dim[0].label != 'xgc.hover_thrust/1':
            return
        item = dict(zip(fields, message.data))
        for field in ('state_native', 'flags', 'sample_used'):
            item[field] = int(item[field])
        item['received_wall_time_ns'] = time.time_ns()
        with capture_lock:
            capture['hover_thrust_trace'].append(item)

    def on_extended_state(message):
        with capture_lock:
            capture['extended_states'].append({'stamp_ns': stamp_ns(message),
                'received_wall_time_ns': time.time_ns(), 'landed_state': message.landed_state,
                'vtol_state': message.vtol_state})

    hosts = {"plant": None, "uav1": None}
    host_pids = {"plant": None, "uav1": None}
    host_logs = {}
    host_shutdown_escalation = {"plant": None, "uav1": None}
    endpoint = None
    endpoint_error = None
    failure = None
    custom_start_ns = None
    validation = None
    loss_validation = None
    paused_controller = False
    stop_quiet = False

    def check_deadline(wait_description=None):
        for role, host in hosts.items():
            if host is not None and host.poll() is not None:
                raise RuntimeError(f"{role} xgc-rt-host exited with status {host.returncode}")
        if rospy.is_shutdown():
            raise RuntimeError("ROS node shut down before the flight sequence completed")
        if time.monotonic() >= deadline:
            if wait_description:
                raise TimeoutError(f"timeout waiting for {wait_description}")
            raise TimeoutError("controller live fixture exceeded its 60 second limit")

    def wait_for(predicate, description):
        while True:
            check_deadline(description)
            if predicate():
                return
            time.sleep(0.01)

    def current(name):
        with capture_lock:
            value = capture[name]
            if name in ("poses", "velocities", "imus", "hover_thrust", "hover_thrust_trace"):
                return value[-1] if value else None
            return value

    def record_action(action, stamp_ns_value=None):
        with capture_lock:
            capture["actions"].append(
                {"action": action, "wall_time_ns": time.time_ns(), "target_stamp_ns": stamp_ns_value}
            )

    def call_service(name, service, **fields):
        """Bound the test client as well as the real facade's result deadline."""
        rospy.wait_for_service(name, timeout=3)
        proxy = rospy.ServiceProxy(name, service)
        outcome = []
        finished = threading.Event()

        def invoke():
            try:
                outcome.append(proxy(**fields))
            except BaseException as error:
                outcome.append(error)
            finally:
                finished.set()

        worker = threading.Thread(target=invoke, daemon=True)
        worker.start()
        until = time.monotonic()+5
        try:
            while not finished.wait(.05):
                check_deadline(name + ' executed-request response')
                if time.monotonic() >= until:
                    raise TimeoutError(name + ' did not return within 5 seconds')
            if isinstance(outcome[0], BaseException):
                raise outcome[0]
            return outcome[0]
        finally:
            proxy.close()

    def result_payload():
        with capture_lock:
            controller_states = list(capture["controller_states"])
            fcu_state_changes = list(capture["fcu_state_changes"])
            poses = list(capture["poses"])
            velocities = list(capture["velocities"])
            imus = list(capture['imus'])
            targets = list(capture['attitude_targets'])
            estimates = list(capture['hover_thrust'])
            estimator_trace = list(capture['hover_thrust_trace'])
            extended_states = list(capture['extended_states'])
            service_responses = list(capture['service_responses'])
            actions = list(capture["actions"])
            final_fcu_state = capture["fcu_state"]
        host_processes = {
            role: {
                "pid": host_pids[role],
                "exit_code": hosts[role].returncode if hosts[role] is not None else None,
                "manifest": str(manifest_paths[role]),
                "audit_dir": str(audit_paths[role]),
                "log": str(log_paths[role]),
                "shutdown_escalation": host_shutdown_escalation[role],
            }
            for role in hosts
        }
        return {
            "scope": "two real Zenoh Hosts and ROS: FS150 6DoF + unchanged ctl-px4 SMC/PVA or DFBC/body-rate + real HTE; no planner/SITL/Gazebo validation",
            "backend": args.backend, 'fixed_px4_hover_physics': hover_physics,
            'startup_fcu_overrides': {'MPC_THR_HOVER': expected_hover, 'THR_MDL_FAC': args.thr_mdl_fac},
            'startup_override_provenance': 'explicit authorized fixture physical-equilibrium calibration; owning FS asset defaults retained',
            'fcu_request_schema': 'xgc.fcu_request/2', 'elf_interfaces': elf_interfaces,
            'host_source': 'installed-published image4809/f6 lineage, compatible ABI; no new host build',
            'hte_initial_configured': args.hte_initial, 'six_dof_validation': validation,
            'control_stream_loss': loss_validation, 'stop_outputs_quiet': stop_quiet,
            "passed": failure is None and endpoint_error is not None and endpoint_error < 0.03 and validation is not None and validation['passed'] and stop_quiet,
            "failure": failure,
            "epoch_ns": epoch_ns,
            "endpoints": {
                "plant": args.plant_endpoint,
                "uav1": args.controller_endpoint,
            },
            "input_paths": {
                "host": str(host_path),
                "plant": str(plant_path),
                "controller": str(controller_path),
                "ros_io": str(ros_io_path),
                'hover_thrust_estimator': str(hte_path),
            },
            'input_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in (host_path, plant_path, controller_path, ros_io_path, hte_path)},
            "host_processes": host_processes,
            "controller_state_sequence": controller_states,
            "fcu_state_changes": fcu_state_changes,
            "final_fcu_state": final_fcu_state,
            "endpoint": endpoint,
            "endpoint_error_m": endpoint_error,
            "pose_output_stamps_ns": [sample["stamp_ns"] for sample in poses],
            "velocity_output_stamps_ns": [sample["stamp_ns"] for sample in velocities],
            "pose_outputs": poses,
            "velocity_outputs": velocities,
            'imu_outputs': imus, 'attitude_target_outputs': targets,
            'hover_thrust_outputs': estimates, 'hover_thrust_native_trace': estimator_trace,
            'extended_state_outputs': extended_states, 'real_ros_service_responses': service_responses,
            "actions": actions,
        }

    def print_summary(result):
        print(
            json.dumps(
                {
                    "passed": result["passed"],
                    "failure": result["failure"],
                    "endpoint_error_m": result["endpoint_error_m"],
                    'six_dof_validation': result['six_dof_validation'],
                    'control_stream_loss': result['control_stream_loss'],
                    'stop_outputs_quiet': result['stop_outputs_quiet'],
                    "state_sequences": {
                        "controller": result["controller_state_sequence"],
                        "fcu": result["fcu_state_changes"],
                    },
                    "host_processes": result["host_processes"],
                },
                indent=2,
                sort_keys=True,
            )
        )

    try:
        for role in ("plant", "uav1"):
            host_logs[role] = log_paths[role].open("w", encoding="utf-8")
            hosts[role] = subprocess.Popen(
                [str(host_path), "--manifest", str(manifest_paths[role])],
                stdout=host_logs[role],
                stderr=subprocess.STDOUT,
            )
            host_pids[role] = hosts[role].pid

        rospy.init_node("private_lightweight_controller_live", anonymous=False)
        rospy.Subscriber("/uav1/custom/statustext", String, on_controller_state, queue_size=100)
        rospy.Subscriber("/uav1/mavros/state", State, on_fcu_state, queue_size=100)
        rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, on_pose, queue_size=100)
        rospy.Subscriber("/uav1/mavros/local_position/velocity_local", TwistStamped, on_velocity, queue_size=100)
        rospy.Subscriber('/uav1/mavros/imu/data', Imu, on_imu, queue_size=100)
        rospy.Subscriber('/uav1/mavros/setpoint_raw/target_attitude', AttitudeTarget, on_target, queue_size=100)
        rospy.Subscriber('/uav1/hover_thrust/estimate_state', HoverThrustEstimate, on_hover_thrust, queue_size=100)
        rospy.Subscriber('/uav1/hover_thrust/native_trace', Float64MultiArray, on_hover_thrust_trace, queue_size=100)
        rospy.Subscriber('/uav1/mavros/extended_state', ExtendedState, on_extended_state, queue_size=100)
        command = rospy.Publisher("/command", String, queue_size=10)
        reference = rospy.Publisher("/uav1/alg/setpoint_raw/local", PositionTarget, queue_size=10)
        flat_reference = rospy.Publisher('/uav1/alg/multirotor_reference_trajectory/active/sampled', SampledReference, queue_size=1)

        wait_for(
            lambda: command.get_num_connections() > 0
            and (reference if args.backend == 'smc' else flat_reference).get_num_connections() > 0
            and current("fcu_state") is not None
            and current("poses") is not None
            and current("velocities") is not None,
            # IMU and real-estimator observations are part of readiness.
            "ROS edge connections and initial plant feedback",
        )
        wait_for(lambda: current("controller_state") == "Ready", "controller Ready")
        wait_for(lambda: current('imus') is not None and current('hover_thrust_trace') is not None,
                 'same-stamp oriented IMU and initial real HTE output')
        record_action("takeoff")
        command.publish(String(data="takeoff"))

        wait_for(lambda: current("controller_state") == "Hover", "controller Hover")
        wait_for(lambda: current('hover_thrust_trace') is not None
                 and current('hover_thrust_trace')['state_native'] == 12
                 and abs(current('hover_thrust_trace')['hover_thrust']-expected_hover) < .02,
                 'real HTE Airborne convergence before closed-loop tracking')
        custom_start_ns = time.time_ns()
        if args.backend == 'dfbc':
            flat_reference.publish(sampled_reference(custom_start_ns))
        record_action("custom1", custom_start_ns)
        command.publish(String(data="custom1"))

        next_setpoint_ns = custom_start_ns
        while True:
            check_deadline()
            now_ns = time.time_ns()
            elapsed_ns = max(0, now_ns - custom_start_ns)
            if now_ns >= next_setpoint_ns:
                setpoint = position_target(elapsed_ns, now_ns)
                if args.backend == 'smc':
                    reference.publish(setpoint)
                if elapsed_ns >= TRACKING_NS:
                    target_stamp_ns = setpoint.header.stamp.to_nsec()
                    wait_for(
                        lambda: current("poses") is not None
                        and current("poses")["stamp_ns"] >= target_stamp_ns,
                        "pose output at the 10 second tracking endpoint",
                    )
                    latest_pose = current("poses")
                    if current("controller_state") != "Custom1":
                        raise AssertionError(
                            f"controller left Custom1 before endpoint measurement: {current('controller_state')}"
                        )
                    endpoint = {
                        "stamp_ns": latest_pose["stamp_ns"],
                        "received_wall_time_ns": latest_pose["received_wall_time_ns"],
                        "position": latest_pose["position"],
                    }
                    endpoint_error = (
                        (latest_pose["position"][0] - 1.0) ** 2
                        + latest_pose["position"][1] ** 2
                        + (latest_pose["position"][2] - 1.0) ** 2
                    ) ** 0.5
                    break
                next_setpoint_ns += STEP_NS
                if next_setpoint_ns <= now_ns:
                    next_setpoint_ns = now_ns + STEP_NS
            time.sleep(0.005)

        if endpoint_error >= 0.03:
            raise AssertionError(f"10 second endpoint error {endpoint_error:.6f} m is not below 0.03 m")

        # Pause only the controller/HTE Host. Plant and its ROS observer keep
        # running, so the loss result is model feedback, not a silent observer.
        before_loss = current('poses')
        record_action('control_stream_loss_begin', before_loss['stamp_ns'])
        hosts['uav1'].send_signal(signal.SIGSTOP)
        paused_controller = True
        wait_for(lambda: current('fcu_state')['mode'] == 'POSCTL'
                 and current('poses')['stamp_ns'] > before_loss['stamp_ns']+500_000_000,
                 'plant Offboard-loss fallback with fresh independently observed state')
        after_loss = current('poses')
        drift = norm([a-b for a, b in zip(after_loss['position'], before_loss['position'])])
        loss_validation = {'channel': 'setpoint' if args.backend == 'smc' else 'attitude_command',
                           'before': before_loss, 'after': after_loss,
                           'mode': current('fcu_state')['mode'], 'drift_m': drift,
                           'source_time_advanced_s': (after_loss['stamp_ns']-before_loss['stamp_ns'])/1e9}
        if drift >= .15 or after_loss['position'][2] < .3 or not current('fcu_state')['armed']:
            raise AssertionError('Offboard loss did not hold a bounded airborne state')
        hosts['uav1'].send_signal(signal.SIGCONT)
        paused_controller = False
        record_action('control_stream_resumed', after_loss['stamp_ns'])
        wait_for(lambda: current('controller_state') in ('Hover', 'Custom1'), 'controller feedback after stream recovery')
        record_action('land')
        command.publish(String(data='land'))

        wait_for(
            lambda: current("fcu_state") is not None
            and not current("fcu_state")["armed"]
            and current("poses") is not None
            and current("poses")["position"][2] < 0.03,
            "plant ground state and armed=false",
        )
        record_action("landed_from_plant_feedback")
        for name, service, fields in (
                ('/uav1/mavros/cmd/arming', CommandBool, {'value': False}),
                ('/uav1/mavros/cmd/command', CommandLong, {'command': 400, 'param1': 0.0})):
            before_call = time.time_ns()
            response = call_service(name, service, **fields)
            with capture_lock:
                capture['service_responses'].append({'service': name, 'request': fields,
                    'sent_wall_time_ns': before_call, 'received_wall_time_ns': time.time_ns(),
                    'success': bool(response.success), 'MAV_RESULT': int(response.result),
                    'scope': 'actual correlated service reply on already-disarmed plant; no armed-state change substituted'})
            if not response.success or response.result != 0:
                raise AssertionError(name + ' did not return executed-request ACCEPTED')
        response = call_service('/uav1/mavros/set_mode', SetMode, custom_mode='POSCTL')
        with capture_lock:
            capture['service_responses'].append({'service': '/uav1/mavros/set_mode',
                'mode_sent': bool(response.mode_sent), 'received_wall_time_ns': time.time_ns(),
                'scope': 'SetMode means host delivery; actual mode must be independently observed'})
        if not response.mode_sent:
            raise AssertionError('SetMode request was not delivered')
        wait_for(lambda: current('fcu_state')['mode'] == 'POSCTL', 'actual FCU mode after SetMode delivery')
        with capture_lock:
            validation = analyse_flight(capture, custom_start_ns, expected_hover, args.hte_initial)
            sequence = [s['state'] for s in capture['controller_states']]
            targets = [s for s in capture['attitude_targets']
                       if custom_start_ns <= s['stamp_ns'] <= custom_start_ns+TRACKING_NS]
            extended = list(capture['extended_states'])
        next_state = 0
        expected_states = ('Ready', 'Takeoff', 'Hover', 'Custom1', 'Landing')
        for state in sequence:
            family = 'Takeoff' if state.startswith('Takeoff') else state
            if next_state < len(expected_states) and family == expected_states[next_state]:
                next_state += 1
        if next_state != len(expected_states):
            validation['failures'].append('missing ordered Ready/Takeoff/Hover/Custom1/Landing')
        if not targets or (args.backend == 'dfbc' and sum(s['type_mask'] == 128 for s in targets) < 100):
            validation['failures'].append('real controller body-rate/target telemetry missing')
        if not extended or not any(s['landed_state'] == 2 for s in extended) or extended[-1]['landed_state'] != 1:
            validation['failures'].append('real extended state did not show airborne then on-ground')
        if extended and any(b['stamp_ns'] <= a['stamp_ns'] for a,b in zip(extended, extended[1:])):
            validation['failures'].append('extended-state source stamps are not strictly increasing')
        if extended and any(s['vtol_state'] != 0 for s in extended):
            validation['failures'].append('quadrotor extended state has a spurious VTOL state')
        validation['passed'] = not validation['failures']
        if not validation['passed']:
            raise AssertionError('; '.join(validation['failures']))
        check_deadline()
    except BaseException as error:
        failure = f"{type(error).__name__}: {error}"
        raise
    finally:
        if paused_controller and hosts['uav1'] is not None and hosts['uav1'].poll() is None:
            hosts['uav1'].send_signal(signal.SIGCONT)
        for role, host in hosts.items():
            if host is not None and host.poll() is not None and failure is None:
                failure = f"{role} xgc-rt-host exited before fixture shutdown with status {host.returncode}"
        for role in ("uav1", "plant"):
            host = hosts[role]
            if host is None or host.poll() is not None:
                continue
            try:
                host.send_signal(signal.SIGINT)
            except ProcessLookupError:
                if host.poll() is not None and failure is None:
                    failure = f"{role} xgc-rt-host exited before fixture shutdown with status {host.returncode}"
            if host.poll() is None:
                try:
                    host.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    host_shutdown_escalation[role] = "terminate"
                    host.terminate()
                    try:
                        host.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        host_shutdown_escalation[role] = "kill"
                        host.kill()
                        host.wait()
        for host_log in host_logs.values():
            host_log.close()
        if failure is None:
            if any(host.returncode != 0 for host in hosts.values()) or any(host_shutdown_escalation.values()):
                failure = 'Stop did not cleanly retire both real Hosts'
            else:
                time.sleep(.25)  # drain messages already delivered before Stop
                with capture_lock:
                    quiet_channels = ('poses', 'velocities', 'imus', 'attitude_targets', 'hover_thrust', 'hover_thrust_trace', 'extended_states')
                    counts = [len(capture[key]) for key in quiet_channels]
                time.sleep(.3)
                with capture_lock:
                    stop_quiet = counts == [len(capture[key]) for key in quiet_channels]
                if not stop_quiet:
                    failure = 'state/IMU/HTE outputs continued after both Hosts stopped'
        result = result_payload()
        (work / "controller-live-result.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print_summary(result)
    if not result['passed']:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
