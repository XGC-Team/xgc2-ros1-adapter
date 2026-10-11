"""Start-order and reconnect checks against the existing private-master fixtures."""

import base64
import json
import signal
import time
import unittest
from concurrent.futures import ThreadPoolExecutor

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
                self.assertTrue(self.describe(server)["ready"])
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
