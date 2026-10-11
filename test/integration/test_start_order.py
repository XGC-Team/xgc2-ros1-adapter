"""Start-order and reconnect checks against the existing private-master fixtures."""

import base64
import json
import os
import signal
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from harness import (
    EXIT_MASTER_CHANGED, EXIT_OK, MECANUM, PX4_PROVIDER, SCOUT,
    FakeRobots, Robot, Roscore, Server, ToolsServer, Workspace,
    new_run_id, wait_until,
)


class StartOrderTest(unittest.TestCase):
    def setUp(self):
        self.workspace = Workspace()
        self.addCleanup(self.workspace.close)
        self.roscore = Roscore(self.workspace)

    def start_adapters(self):
        servers = [Server(self.workspace, self.roscore, provider, new_run_id()).start()
                   for provider in (SCOUT, MECANUM, PX4_PROVIDER)]
        servers.append(ToolsServer(self.workspace, self.roscore, new_run_id()).start())
        return servers

    def describe(self, server, wait_ms=0):
        if isinstance(server, ToolsServer):
            status, _, body = server.http.describe(wait_ms=wait_ms)
            self.assertEqual(status, 200, body)
        else:
            reply = server.client.describe(wait_ms=wait_ms)
            self.assertEqual(reply["status"]["code"], 0, reply)
            body = reply["response"]
        return body

    def test_adapters_wait_for_a_late_master_and_hold_describe(self):
        servers = self.start_adapters()
        for server in servers:
            body = self.describe(server)
            self.assertFalse(body["ready"], body)
            facts = body.get("facts") or json.loads(body["facts_json"])
            self.assertEqual(facts["ros_master_uri"], self.roscore.uri)
            self.assertEqual(facts["ros_master_pid"], 0)
            self.assertEqual(facts["reason"], "waiting for ROS master binding")

        with ThreadPoolExecutor(max_workers=len(servers)) as executor:
            # Stay absent beyond the old five-second startup failure budget.
            started = time.monotonic()
            pending = [executor.submit(self.describe, server, 6200) for server in servers]
            for future, server in zip(pending, servers):
                self.assertFalse(future.result(timeout=12)["ready"])
                self.assertFalse(server.exited(), server.output())
            self.assertGreaterEqual(time.monotonic() - started, 6)

            pending = [executor.submit(self.describe, server, 20000) for server in servers]
            time.sleep(0.2)
            self.assertTrue(all(not future.done() for future in pending))
            self.roscore.start()
            for future, server in zip(pending, servers):
                body = future.result(timeout=25)
                self.assertTrue(body["ready"], body)
                facts = body.get("facts") or json.loads(body["facts_json"])
                self.assertEqual(facts["ros_master_pid"], self.roscore.pid())
                self.assertEqual(facts["ros_master_run_id"], self.roscore.run_id())
                server.signal(signal.SIGTERM)
                self.assertEqual(server.wait(10), EXIT_OK, server.output())

    @unittest.skipUnless(os.environ.get("XGC_TEST_LONG") == "1",
                         "long: set XGC_TEST_LONG=1 for the 60-second master outage")
    def test_long_master_outage_is_idle_and_binds_once(self):
        servers = self.start_adapters()

        def resources(server):
            proc = Path("/proc") / str(server.pid)
            # The command name in parentheses can contain spaces.
            fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
            cpu = (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
            return cpu, len(list((proc / "task").iterdir())), len(list((proc / "fd").iterdir()))

        with ThreadPoolExecutor(max_workers=len(servers)) as executor:
            held = [executor.submit(self.describe, server, 30000) for server in servers]
            time.sleep(0.5)
            before = [resources(server) for server in servers]
            started = time.monotonic()
            time.sleep(60)
            elapsed = time.monotonic() - started
            after = [resources(server) for server in servers]
            self.assertGreaterEqual(elapsed, 60)
            for future in held:
                self.assertFalse(future.result(timeout=5)["ready"])

        for server, initial, final in zip(servers, before, after):
            with self.subTest(adapter=server.name):
                body = self.describe(server)
                self.assertFalse(body["ready"], body)
                facts = body.get("facts") or json.loads(body["facts_json"])
                cpu = final[0] - initial[0]
                self.assertLess(cpu, 1.2, (cpu, server.output()))
                self.assertLessEqual(final[1], initial[1], (initial, final))
                self.assertLessEqual(final[2], initial[2], (initial, final))
                # Full jitter averages half its ceiling. A 10-second cap
                # therefore averages about 18 attempts, rather than having a
                # deterministic 15-attempt maximum. Allow random variation;
                # the fast unit test verifies every exponential ceiling.
                self.assertLessEqual(facts["master_probe_attempts"], 30, facts)
                self.assertEqual(facts["master_bindings"], 0, facts)
                outage_lines = [line for line in server.output().splitlines()
                                if "[ros-master]" in line]
                self.assertTrue(outage_lines, server.output())
                self.assertLessEqual(len(outage_lines), 7, outage_lines)
                self.assertTrue(all("suppressed_attempts=" in line
                                    for line in outage_lines[1:]), outage_lines)
                self.assertLessEqual(len(server.output().splitlines()), 9, server.output())
                print("long master outage {}: cpu={:.3f}s attempts={} logs={} threads={} fds={}"
                      .format(server.name, cpu, facts["master_probe_attempts"],
                              len(outage_lines), final[1], final[2]), flush=True)

        self.roscore.start()
        for server in servers:
            body = self.describe(server, 20000)
            self.assertTrue(body["ready"], body)
            facts = body.get("facts") or json.loads(body["facts_json"])
            self.assertEqual(facts["master_bindings"], 1, facts)
            if not isinstance(server, ToolsServer):
                robot = Robot(server.provider, "long_" + new_run_id().replace("-", "_"),
                              "xsim.ros", run_id=server.run_id)
                for _ in range(2):
                    reply = server.client.apply([robot.registration()])
                    self.assertEqual(reply["status"]["code"], 0, reply)
                    self.assertNotIn("error", reply["response"]["members"][0])
                facts = json.loads(self.describe(server)["facts_json"])
                self.assertEqual(facts["members"], 1, facts)
                self.assertEqual(facts["members_revision"], 1, facts)

        tools = servers[-1]
        instance = self.describe(tools)["instance_id"]
        topic = "/long_tools/chatter"
        for value in ("superseded", "latest"):
            status, _, body = tools.http.request("POST", "/api/v1/publish", instance=instance,
                body={"topic": topic, "messageType": "std_msgs/String",
                      "message": {"data": value}, "publishCount": 1,
                      "publishRateHz": 1, "latch": True, "queueSize": 1,
                      "waitForSubscribersMs": 0})
            self.assertEqual(status, 200, body)
        for _ in range(2):
            fake = FakeRobots(self.workspace, self.roscore,
                              record={topic: "std_msgs/String"}).start()
            wait_until(lambda: fake.received(topic), 10, message="one latest latched replay")
            time.sleep(0.2)
            messages = fake.received(topic)
            self.assertEqual(len(messages), 1, messages)
            self.assertEqual(messages[0]["msg"]["data"], "latest", messages)
            fake.cleanup()
        # No re-binding or repeated ROS registration after successful recovery.
        time.sleep(2)
        for server in servers:
            body = self.describe(server)
            facts = body.get("facts") or json.loads(body["facts_json"])
            self.assertEqual(facts["master_bindings"], 1, facts)
            server.signal(signal.SIGTERM)
            self.assertEqual(server.wait(10), EXIT_OK, server.output())

    def test_sigterm_during_the_master_wait_exits_cleanly(self):
        servers = self.start_adapters()
        with ThreadPoolExecutor(max_workers=len(servers)) as executor:
            pending = [executor.submit(self.describe, server, 20000) for server in servers]
            time.sleep(0.2)
            self.assertTrue(all(not future.done() for future in pending))
            for server in servers:
                server.signal(signal.SIGTERM)
            for future, server in zip(pending, servers):
                self.assertFalse(future.result(timeout=10)["ready"])
                self.assertEqual(server.wait(10), EXIT_OK, server.output())
                self.assertFalse(server.socket.exists())

    def test_a_new_master_run_id_with_the_same_pid_ends_bound_adapters(self):
        self.roscore.start()
        servers = self.start_adapters()
        for server in servers:
            self.assertTrue(self.describe(server, 20000)["ready"])
        pid = self.roscore.pid()
        self.roscore.call("setParam", "/run_id", new_run_id())
        self.assertEqual(self.roscore.pid(), pid)
        for server in servers:
            self.assertEqual(server.wait(15), EXIT_MASTER_CHANGED, server.output())

    def test_sigterm_during_a_silent_master_wait_exits_cleanly(self):
        self.roscore.start()
        self.roscore.stop_responding()
        servers = self.start_adapters()
        for server in servers:
            self.assertFalse(self.describe(server)["ready"])
            server.signal(signal.SIGTERM)
        for server in servers:
            self.assertEqual(server.wait(10), EXIT_OK, server.output())

    def test_late_robot_publishers_and_restarts_keep_the_same_adapter(self):
        self.roscore.start()
        for provider in (SCOUT, MECANUM, PX4_PROVIDER):
            with self.subTest(provider=provider):
                server = Server(self.workspace, self.roscore, provider, new_run_id()).start()
                self.assertTrue(self.describe(server, 20000)["ready"])
                robot = Robot(provider, "late_" + new_run_id().replace("-", "_"),
                              "xsim.ros", run_id=server.run_id)
                reply = server.client.apply([robot.registration()])
                self.assertEqual(reply["status"]["code"], 0, reply)
                self.assertNotIn("error", reply["response"]["members"][0])
                watch = server.client.watch(60)
                self.addCleanup(watch.close)

                def health():
                    message = watch.members()[0].get(robot.robot_id, {}).get("state.health")
                    return (base64.b64decode(message["message"]["payload"]["value"])
                            if message else b"")

                # The serialized health payload contains the missing-source reason.
                wait_until(lambda: b"has not been observed" in health(), 10,
                           message="waiting robot health")
                body = self.describe(server)
                self.assertTrue(body["ready"])
                facts = json.loads(body["facts_json"])
                self.assertEqual(facts["robots"][robot.robot_id]["state"], "waiting", facts)
                self.assertNotIn("state.imu", watch.members()[0].get(robot.robot_id, {}))
                fake = FakeRobots(self.workspace, self.roscore, [robot]).start()
                wait_until(lambda: "state.imu" in watch.members()[0].get(robot.robot_id, {}),
                           10, message="late telemetry")
                fake.cleanup()
                before = int(watch.members()[0][robot.robot_id]["state.imu"]["message"]["sequence"])
                fake = FakeRobots(self.workspace, self.roscore, [robot]).start()
                wait_until(lambda: int(watch.members()[0][robot.robot_id]["state.imu"]
                                       ["message"]["sequence"]) > before + 3,
                           10, message="telemetry after publisher restart")
                self.assertFalse(server.exited(), server.output())
                self.assertTrue(self.describe(server)["ready"])
                server.signal(signal.SIGTERM)
                self.assertEqual(server.wait(10), EXIT_OK, server.output())
                fake.cleanup()
                watch.close()

    def test_tools_keeps_publishers_for_late_and_restarted_subscribers(self):
        self.roscore.start()
        server = ToolsServer(self.workspace, self.roscore, new_run_id()).start()
        instance = self.describe(server, 20000)["instance_id"]
        topic = "/late_tools/chatter"
        status, _, body = server.http.request("POST", "/api/v1/publish", instance=instance,
                                             body={
            "topic": topic, "messageType": "std_msgs/String", "message": {"data": "held"},
            "publishCount": 1, "publishRateHz": 1, "latch": True, "queueSize": 1,
            "waitForSubscribersMs": 0,
        })
        self.assertEqual(status, 200, body)
        self.assertEqual(body.get("event"), "published", body)
        for _ in range(2):
            fake = FakeRobots(self.workspace, self.roscore,
                              record={topic: "std_msgs/String"}).start()
            wait_until(lambda: fake.received(topic), 10, message="the cached latched message")
            self.assertEqual(fake.received(topic)[-1]["msg"]["data"], "held")
            self.assertEqual(self.describe(server)["instance_id"], instance)
            fake.cleanup()
        server.signal(signal.SIGTERM)
        self.assertEqual(server.wait(10), EXIT_OK, server.output())

    def test_tools_publish_wait_wakes_for_a_late_subscriber(self):
        self.roscore.start()
        server = ToolsServer(self.workspace, self.roscore, new_run_id()).start()
        instance = self.describe(server, 20000)["instance_id"]
        topic = "/late_tools/waited"
        request = {"topic": topic, "messageType": "std_msgs/String",
                   "message": {"data": "connected"}, "publishCount": 1,
                   "publishRateHz": 1, "latch": False, "queueSize": 1,
                   "waitForSubscribersMs": 5000}
        with ThreadPoolExecutor(max_workers=1) as executor:
            pending = executor.submit(server.http.request, "POST", "/api/v1/publish",
                                      body=request, instance=instance)
            time.sleep(0.2)
            self.assertFalse(pending.done())
            fake = FakeRobots(self.workspace, self.roscore,
                              record={topic: "std_msgs/String"}).start()
            status, _, body = pending.result(timeout=10)
        self.assertEqual(status, 200, body)
        self.assertEqual(body.get("event"), "published", body)
        wait_until(lambda: fake.received(topic), 10, message="publish after connect event")
        self.assertEqual(fake.received(topic)[-1]["msg"]["data"], "connected")
        server.signal(signal.SIGTERM)
        self.assertEqual(server.wait(10), EXIT_OK, server.output())
        fake.cleanup()

    def test_tools_discovers_late_and_restarted_service_providers(self):
        self.roscore.start()
        server = ToolsServer(self.workspace, self.roscore, new_run_id()).start()
        instance = self.describe(server, 20000)["instance_id"]
        service = "/late_tools/set_bool"
        request = {
            "service": service, "serviceType": "std_srvs/SetBool", "request": {"data": True},
            "waitForServiceMs": 5000, "callTimeoutMs": 5000,
        }
        with ThreadPoolExecutor(max_workers=1) as executor:
            pending = executor.submit(server.http.request, "POST", "/api/v1/call",
                                      body=request, instance=instance)
            time.sleep(0.2)
            self.assertFalse(pending.done())
            fake = FakeRobots(self.workspace, self.roscore,
                              services={service: "std_srvs/SetBool"}).start()
            status, _, body = pending.result(timeout=10)
        self.assertEqual(status, 200, body)
        self.assertTrue(body["response"]["success"], body)
        fake.cleanup()
        fake = FakeRobots(self.workspace, self.roscore,
                          services={service: "std_srvs/SetBool"}).start()
        status, _, body = server.http.request("POST", "/api/v1/call",
                                             body=request, instance=instance)
        self.assertEqual(status, 200, body)
        self.assertTrue(body["response"]["success"], body)
        self.assertEqual(len(fake.received(service)), 1)
        self.assertEqual(self.describe(server)["instance_id"], instance)
        server.signal(signal.SIGTERM)
        self.assertEqual(server.wait(10), EXIT_OK, server.output())
        fake.cleanup()


if __name__ == "__main__":
    unittest.main()
