"""The PX4 multirotor type server against a private roscore.

The vehicles are fake MAVROS instances (fake_robots.py): they publish the
MAVROS and VRPN topics the PX4 profiles read, answer the arming, mode and
command services, and record the setpoints the server streams.
"""

import json
import signal
import threading
import time
import unittest

from harness import (
    EXIT_MASTER_CHANGED, EXIT_OK, PX4_PROVIDER, MasterCase, Robot, Roscore, Server,
    Workspace, arm_payload, mode_payload, motion_intent_payload, new_run_id,
    process_exists, registrations, top_name, wait_until,
)

PROFILES = ("gazebo.vrpn", "physical.vrpn", "xsim.ros")
SUCCEEDED = "OPERATION_PHASE_SUCCEEDED"


def unique(name):
    return "{}_{}".format(name, new_run_id()[4:])


def stopped(setpoint):
    return setpoint["velocity"] == [0.0, 0.0, 0.0] and setpoint["yaw_rate"] == 0.0


class Px4ServerTest(MasterCase):
    provider = PX4_PROVIDER

    def robots(self, server, *profiles, prefix="uav"):
        return [Robot(self.provider, unique("{}{}".format(prefix, index)), profile,
                      run_id=server.run_id)
                for index, profile in enumerate(profiles)]

    def setpoints(self, fake, robot):
        return fake.received(top_name(robot.namespace, "mavros/setpoint_raw/local"))

    def last_setpoint(self, fake, robot):
        entries = self.setpoints(fake, robot)
        return entries[-1]["msg"] if entries else None

    def execute(self, server, robot, endpoint, payload):
        reply = server.client.execute(robot.identity, endpoint, "work-" + new_run_id(), payload)
        self.assertEqual(reply["status"]["code"], 0, reply)
        return reply["response"]["event"]

    def subscribed(self, server, robot):
        subscribers = self.roscore.system_state()["subscribers"]
        return all(self.node_of(server) in subscribers.get(topic, ())
                   for topic in robot.input_topics())

    # Members ----------------------------------------------------------------

    def test_describe_and_one_apply_for_robots_of_different_profiles(self):
        server = self.start_server(self.provider)
        description = self.ready(server)
        facts = json.loads(description["facts_json"])
        self.assertEqual(facts["provider"], self.provider)
        self.assertEqual(facts["ros_master_pid"], self.roscore.pid())
        self.assertEqual(description["provider_definition_id"], self.provider)

        robots = self.robots(server, *PROFILES)
        self.start_fake_robots(robots)
        self.apply_all(server, robots)
        for robot in robots:
            self.assertTrue(self.subscribed(server, robot), robot.profile_id)
        health = server.client.health()["response"]
        self.assertEqual((health["robots"], health["members"]), (3, 3))

        watch = self.watch(server)
        wait_until(lambda: all({"state.pose", "state.imu", "state.power", "state.flight"} <=
                               set(watch.members()[0].get(r.robot_id, ())) for r in robots),
                   30, 0.1, "status of every robot")

        # Per-member failures do not affect the others.
        extra, wrong_run = self.robots(server, "xsim.ros", "xsim.ros", prefix="more")
        wrong_run.run_id = "another-run"
        members = self.apply(server, [extra, wrong_run])
        self.assertNotIn("error", members[0], members[0])
        self.assertEqual(members[1]["error"]["class"], "ERROR_CLASS_REJECTED", members[1])

    # Commands ---------------------------------------------------------------

    def test_arm_and_mode_are_calls_to_the_vehicles_services(self):
        server = self.start_server(self.provider)
        self.ready(server)
        robot, other = self.robots(server, "xsim.ros", "xsim.ros")
        fake = self.start_fake_robots([robot, other])
        self.apply_all(server, [robot, other])

        arming = top_name(robot.namespace, "mavros/cmd/arming")
        event = self.execute(server, robot, "arm", arm_payload(True))
        self.assertEqual(event["phase"], SUCCEEDED, event)
        calls = fake.received(arming)
        self.assertEqual(len(calls), 1, calls)
        self.assertIn("value: True", calls[0]["msg"]["request"])

        event = self.execute(server, robot, "set-flight-mode", mode_payload("OFFBOARD"))
        self.assertEqual(event["phase"], SUCCEEDED, event)
        calls = fake.received(top_name(robot.namespace, "mavros/set_mode"))
        self.assertEqual(len(calls), 1, calls)
        self.assertIn("OFFBOARD", calls[0]["msg"]["request"])

        # A mode the profile does not allow never reaches the vehicle.
        event = self.execute(server, robot, "set-flight-mode", mode_payload("MISSION"))
        self.assertNotEqual(event["phase"], SUCCEEDED, event)
        self.assertEqual(len(fake.received(top_name(robot.namespace, "mavros/set_mode"))), 1)
        # The other vehicle was not touched.
        self.assertEqual(fake.received(top_name(other.namespace, "mavros/cmd/arming")), [])

    def drive(self, server):
        self.ready(server)
        moving, idle = self.robots(server, "gazebo.vrpn", "xsim.ros")
        fake = self.start_fake_robots([moving, idle])
        self.apply_all(server, [moving, idle])
        event = self.execute(server, moving, "set-motion-intent", motion_intent_payload(3, 1, 1, -1))
        self.assertEqual(event["phase"], SUCCEEDED, event)

        def streaming():
            setpoint = self.last_setpoint(fake, moving)
            return setpoint is not None and not stopped(setpoint)
        wait_until(streaming, 10, 0.05, "the setpoint stream")
        setpoint = self.last_setpoint(fake, moving)
        # gear 3 is the profile maximum: 1.5 m/s and 1.0 rad/s
        for actual, expected in zip(setpoint["velocity"], [1.5, 1.5, 0.0]):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertAlmostEqual(setpoint["yaw_rate"], -1.0, places=6)
        before = len(self.setpoints(fake, moving))
        wait_until(lambda: len(self.setpoints(fake, moving)) >= before + 3, 5, 0.05,
                   "the setpoints to be streamed")
        return moving, idle, fake

    def assert_stopped_cleanly(self, server, moving, idle, fake):
        self.assert_gone(server, EXIT_OK)
        self.assertTrue(stopped(self.last_setpoint(fake, moving)), self.last_setpoint(fake, moving))
        self.assertEqual(self.setpoints(fake, idle), [])
        self.assertFalse(server.socket.exists())

    def test_shutdown_rpc_ends_the_setpoint_stream_with_a_zero_setpoint(self):
        server = self.start_server(self.provider)
        moving, idle, fake = self.drive(server)
        self.assertEqual(server.client.shutdown()["status"]["code"], 0)
        self.assert_stopped_cleanly(server, moving, idle, fake)

    def test_sigterm_ends_the_setpoint_stream_with_a_zero_setpoint(self):
        server = self.start_server(self.provider)
        moving, idle, fake = self.drive(server)
        server.signal(signal.SIGTERM)
        self.assert_stopped_cleanly(server, moving, idle, fake)

    def test_removing_a_robot_ends_its_setpoint_stream(self):
        server = self.start_server(self.provider)
        moving, idle, fake = self.drive(server)
        reply = server.client.remove([moving.identity])
        self.assertTrue(all("error" not in m for m in reply["response"]["members"]), reply)
        wait_until(lambda: stopped(self.last_setpoint(fake, moving)), 10, 0.05, "the zero setpoint")
        count = len(self.setpoints(fake, moving))
        time.sleep(0.6)
        self.assertEqual(len(self.setpoints(fake, moving)), count)
        self.assertFalse(self.subscribed(server, moving))
        self.assertEqual(server.client.health()["response"]["members"], 1)
        server.signal(signal.SIGTERM)
        self.assert_gone(server, EXIT_OK)

    # Commands in flight when the Run ends -----------------------------------

    def shutdown_with_arming_in_flight(self, service_delay):
        server = self.start_server(self.provider)
        self.ready(server)
        robot, = self.robots(server, "xsim.ros")
        fake = self.start_fake_robots([robot], service_delay=service_delay)
        self.apply_all(server, [robot])
        result = {}

        def arm():
            result["reply"] = server.client.execute(
                robot.identity, "arm", "work-" + new_run_id(), arm_payload(True), deadline_s=60)
        executor = threading.Thread(target=arm)
        executor.start()
        # The call is on the vehicle's service when Shutdown arrives.
        wait_until(lambda: fake.received(top_name(robot.namespace, "mavros/cmd/arming")), 10, 0.02,
                   "the arming call to reach the vehicle")
        started = time.monotonic()
        self.assertEqual(server.client.shutdown()["status"]["code"], 0)
        executor.join(30)
        self.assertFalse(executor.is_alive())
        self.assert_gone(server, EXIT_OK)
        return result["reply"]["response"]["event"], time.monotonic() - started

    def test_a_command_that_finishes_within_the_drain_budget_completes(self):
        event, elapsed = self.shutdown_with_arming_in_flight(service_delay=1.0)
        self.assertEqual(event["phase"], SUCCEEDED, event)
        print("arming in flight finished {:.1f} s after Shutdown began".format(elapsed))

    def test_a_command_that_outlasts_the_drain_budget_is_cancelled(self):
        event, elapsed = self.shutdown_with_arming_in_flight(service_delay=15.0)
        self.assertNotEqual(event["phase"], SUCCEEDED, event)
        self.assertLess(elapsed, 12, event)
        print("arming in flight ended as {} {:.1f} s after Shutdown began".format(
            event["phase"], elapsed))

    # Runs one after the other -----------------------------------------------

    def test_consecutive_runs_with_different_robots_leave_nothing_behind(self):
        history = []
        for stop in ("shutdown", "sigterm"):
            server = self.start_server(self.provider)
            self.ready(server)
            robots = self.robots(server, *PROFILES[:2 + len(history)], prefix="run{}_".format(len(history)))
            fake = self.start_fake_robots(robots)
            self.apply_all(server, robots)
            for robot in robots:
                self.assertTrue(self.subscribed(server, robot))
            for earlier in history:
                for robot in earlier:
                    self.assertFalse(any(self.node_of(server) in nodes for topic, nodes in
                                         self.roscore.system_state()["subscribers"].items()
                                         if topic in robot.input_topics()))
            pid = server.pid
            if stop == "shutdown":
                self.assertEqual(server.client.shutdown()["status"]["code"], 0)
            else:
                server.signal(signal.SIGTERM)
            self.assert_gone(server, EXIT_OK)
            self.assertFalse(process_exists(pid))
            fake.cleanup()
            history.append(robots)

    def test_a_restarted_master_ends_the_server(self):
        workspace = Workspace()
        self.addCleanup(workspace.close)
        roscore = Roscore(workspace).start()
        server = Server(workspace, roscore, self.provider, new_run_id()).start()
        reply = server.client.describe(wait_ms=20000)
        self.assertTrue(reply["response"]["ready"], reply)
        robot = Robot(self.provider, unique("uav"), "xsim.ros", run_id=server.run_id)
        self.assertNotIn("error", server.client.apply([robot.registration()])["response"]["members"][0])
        roscore.restart()
        self.assertEqual(server.wait(60), EXIT_MASTER_CHANGED, server.output())


if __name__ == "__main__":
    unittest.main()
