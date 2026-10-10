#!/usr/bin/env python3
"""Exercise the installed native probe against one private ROS graph."""
import calendar
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    probe = sys.argv[1]
    with socket.socket() as port:
        port.bind(("127.0.0.1", 0))
        master_port = port.getsockname()[1]
    master = "http://127.0.0.1:" + str(master_port)
    os.environ.update(ROS_MASTER_URI=master, ROS_IP="127.0.0.1")
    os.environ.pop("ROS_HOSTNAME", None)
    core = subprocess.Popen(["roscore", "-p", str(master_port)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        for _ in range(100):
            check = subprocess.run([probe, "--mode", "master", "--master-uri", master,
                                    "--timeout-ms", "100"], stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
            if check.returncode == 0:
                break
            time.sleep(0.02)
        else:
            raise AssertionError("private ROS master did not start")
        rospy.init_node("pose_snapshot_fixture", anonymous=True, disable_signals=True)
        topics = ["/tracking/assigned/pose", "/tracking/unassigned/pose", "/custom/coordinate", "/tracking/invalid/pose", "/vrpn_client_node/origin_board/pose"]
        publishers = [rospy.Publisher(topic, PoseStamped, queue_size=1) for topic in topics]
        done = threading.Event()

        def publish():
            while not done.wait(0.01):
                for index, publisher in enumerate(publishers):
                    value = PoseStamped()
                    value.header.frame_id = "fixture_world"
                    # Simulation source time deliberately differs from wall receipt time.
                    value.header.stamp = rospy.Time(123, 456)
                    value.pose.position.x = index + 1.25 if index != 3 else math.nan
                    value.pose.orientation.w = 1.0
                    publisher.publish(value)

        writer = threading.Thread(target=publish)
        writer.start()
        request = {"sessionId": "frozen-session", "experimentCommitId": "frozen-commit", "sources": [
            {"instanceId": "frozen-tracking", "roots": ["/tracking"], "topics": []},
            {"instanceId": "frozen-slot", "roots": ["/robot"], "topics": ["/custom/coordinate"]},
        ]}
        try:
            started = time.time()
            result = subprocess.run([probe, "--mode", "pose-snapshot", "--master-uri", master,
                                     "--timeout-ms", "600"], input=json.dumps(request), text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3, check=True)
            snapshot = json.loads(result.stdout)
            assert snapshot["sessionId"] == request["sessionId"]
            assert snapshot["experimentCommitId"] == request["experimentCommitId"]
            tracking = snapshot["sources"][0]
            assert [sample["topic"] for sample in tracking["samples"]] == topics[:2]
            for sample in tracking["samples"]:
                assert sample["sourceStamp"] == "1970-01-01T00:02:03.000000456Z"
                observed = calendar.timegm(time.strptime(sample["observedAt"][:19], "%Y-%m-%dT%H:%M:%S"))
                assert started - 1 <= observed <= time.time() + 1
                assert sample["frameId"] == "fixture_world"
            assert snapshot["sources"][1]["samples"][0]["topic"] == "/custom/coordinate"
            if len(sys.argv) > 2:
                command = open(sys.argv[2]).read().replace(
                    "/opt/ros/noetic/lib/xgc_ros1_tools_adapter/xgc_ros1_tools_adapter_probe", probe)
                authored = {"context": {"sessionId": "frozen-session", "experimentCommitId": "frozen-commit",
                    "openingRunId": "opening", "openingAcceptedAtEpochNs": "123456789",
                    "containerizedDeployment": True, "visualizationTopics": [], "runMode": "simulation",
                    "simulation": {"rosMasterUri": master}, "coreRosIp": "127.0.0.1"},
                    "robots": [{"id": "frozen-slot", "namespace": "/robot", "simulationPoseTopic": "/custom/coordinate"}]}
                execution = subprocess.run(["bash", "-c", command], input=json.dumps(authored), text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=4)
                assert execution.returncode == 0, execution.stderr
                output = json.loads(execution.stdout)
                assert output["sessionId"] == "frozen-session"
                assert output["sources"][0]["samples"][0]["topic"] == "/vrpn_client_node/origin_board/pose"
                assert output["sources"][1]["samples"][0]["topic"] == "/custom/coordinate"
                del authored["context"]["openingAcceptedAtEpochNs"]
                rejected_author = subprocess.run(["bash", "-c", command], input=json.dumps(authored), text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2)
                assert rejected_author.returncode != 0 and rejected_author.stdout == ""
            # A quiet explicit source stays empty; cancellation exits and unregisters.
            quiet = {**request, "sources": [{"instanceId": "quiet", "roots": [], "topics": ["/quiet/pose"]}]}
            running = subprocess.Popen([probe, "--mode", "pose-snapshot", "--master-uri", master,
                                        "--timeout-ms", "10000"], stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            running.stdin.write(json.dumps(quiet))
            running.stdin.close()
            time.sleep(0.1)
            running.terminate()
            assert running.wait(timeout=2) == 130
            assert running.stdout.read() == ""
            # Declared limits fail before allocation/subscription of an oversized set.
            oversized = {**request, "sources": [{"instanceId": "too-many", "roots": [],
                         "topics": ["/topic_" + str(i) for i in range(257)]}]}
            rejected = subprocess.run([probe, "--mode", "pose-snapshot", "--master-uri", master,
                                       "--timeout-ms", "500"], input=json.dumps(oversized), text=True,
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2)
            assert rejected.returncode == 1 and rejected.stdout == ""
            print(json.dumps({"fresh_assigned_and_unassigned": True, "exact_override": True,
                              "source_time_preserved": True, "malformed_isolated": True,
                              "cancel_exit": 130, "bounded_topics": True, "frozen_author_roundtrip": len(sys.argv) > 2}))
        finally:
            done.set()
            writer.join(timeout=1)
            rospy.signal_shutdown("fixture finished")
    finally:
        os.killpg(core.pid, signal.SIGTERM)
        core.wait(timeout=5)


if __name__ == "__main__":
    main()
