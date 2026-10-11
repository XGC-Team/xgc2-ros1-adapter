"""The ROS1 Tools process (http.v1 service ros1.tools) against a private roscore.

The process is started per Run, bound to one master, ready once it has seen the
master, and ends with the Run or when the master it was bound to is gone.
"""

import signal
import subprocess
import time
import unittest

from harness import (
    EXIT_FAILURE, EXIT_MASTER_CHANGED, EXIT_MASTER_UNREACHABLE, EXIT_OK, EXIT_ROS_SHUTDOWN,
    FakeRobots, MasterCase, Roscore, ToolsServer, Workspace, new_run_id, process_exists,
    registrations, wait_until,
)

SERVICE = "/tools_it/set_bool"


class ToolsServerTest(MasterCase):
    def start_tools(self, run_id=None):
        server = ToolsServer(self.workspace, self.roscore, run_id or new_run_id()).start()
        self.addCleanup(server.cleanup)
        return server

    def ready(self, server):
        status, headers, body = server.http.describe()
        self.assertEqual(status, 200, body)
        status, headers, body = server.http.describe(instance=body["instance_id"], wait_ms=20000)
        self.assertEqual(status, 200, body)
        self.assertTrue(body["ready"], body)
        return body

    def assert_gone(self, server, exit_code=EXIT_OK):
        self.assertEqual(server.wait(60), exit_code, server.output())
        self.assertFalse(process_exists(server.pid))
        self.assertNotIn(server.node, self.roscore.nodes())
        self.assertEqual(registrations(self.roscore, server.node),
                         {"publishers": [], "subscribers": [], "services": []})

    def test_describe_reports_the_master_the_process_is_bound_to(self):
        server = self.start_tools()
        # Discovery is a plain GET /v1/describe; it names the instance.
        status, headers, discovered = server.http.describe()
        self.assertEqual(status, 200, discovered)
        self.assertEqual(discovered["service"], "ros1.tools")
        self.assertEqual(discovered["api_version"], "1")
        self.assertTrue(discovered["instance_id"])
        # Holding for readiness needs the pinned instance.
        started = time.monotonic()
        status, _, body = server.http.describe(instance=discovered["instance_id"], wait_ms=20000)
        self.assertEqual(status, 200, body)
        self.assertTrue(body["ready"], body)
        self.assertLess(time.monotonic() - started, 15)
        facts = body["facts"]
        self.assertEqual(facts["ros_master_uri"], self.roscore.uri)
        self.assertEqual(facts["ros_master_pid"], self.roscore.pid())
        self.assertEqual(facts["ros_master_run_id"], self.roscore.run_id())
        self.assertEqual(facts["ros_node"], server.node)
        self.assertEqual(facts["run_id"], server.run_id)
        self.assertEqual(facts["target_id"], "local")
        self.assertIn(server.node, self.roscore.nodes())
        # Anything else about the query is refused, a wrong instance is a conflict.
        status, _, body = server.http.describe(instance=discovered["instance_id"], wait_ms=30001)
        self.assertEqual(status, 400, body)
        status, _, body = server.http.describe(instance="another-instance", wait_ms=0)
        self.assertEqual(status, 409, body)
        server.signal(signal.SIGTERM)
        self.assert_gone(server)

    def test_publish_reaches_a_subscriber_of_the_runs_master(self):
        server = self.start_tools()
        instance = self.ready(server)["instance_id"]
        fake = FakeRobots(self.workspace, self.roscore,
                          record={"/tools_it/chatter": "std_msgs/String"}).start()
        self.addCleanup(fake.cleanup)
        status, _, body = server.http.request("POST", "/api/v1/publish", instance=instance, body={
            "topic": "/tools_it/chatter", "messageType": "std_msgs/String",
            "message": {"data": "hello from the run"}, "publishCount": 3, "publishRateHz": 10,
            "latch": False, "queueSize": 10, "waitForSubscribersMs": 5000})
        self.assertEqual(status, 200, body)
        self.assertEqual(body.get("event"), "published", body)
        self.assertGreaterEqual(body["subscriberCount"], 1)
        wait_until(lambda: len(fake.received("/tools_it/chatter")) >= 3, 10, 0.1, "the messages")
        self.assertEqual({entry["msg"]["data"] for entry in fake.received("/tools_it/chatter")},
                         {"hello from the run"})
        self.assertIn("/tools_it/chatter", registrations(self.roscore, server.node)["publishers"])
        server.signal(signal.SIGTERM)
        self.assert_gone(server)

    def test_a_service_call_reaches_a_provider_of_the_runs_master(self):
        server = self.start_tools()
        instance = self.ready(server)["instance_id"]
        fake = FakeRobots(self.workspace, self.roscore, services={SERVICE: "std_srvs/SetBool"}).start()
        self.addCleanup(fake.cleanup)
        status, _, body = server.http.request("POST", "/api/v1/call", instance=instance, body={
            "service": SERVICE, "serviceType": "std_srvs/SetBool", "request": {"data": True},
            "waitForServiceMs": 10000, "callTimeoutMs": 10000})
        self.assertEqual(status, 200, body)
        self.assertEqual(body.get("event"), "response", body)
        self.assertTrue(body["response"]["success"], body)
        self.assertEqual(len(fake.received(SERVICE)), 1)
        # A request that is not the declared shape is an error, not a ROS call.
        status, _, body = server.http.request("POST", "/api/v1/call", instance=instance,
                                              body={"service": SERVICE})
        self.assertEqual(status, 200, body)
        self.assertIn("error", body)
        self.assertEqual(len(fake.received(SERVICE)), 1)
        server.signal(signal.SIGTERM)
        self.assert_gone(server)

    def test_consecutive_runs_leave_nothing_behind(self):
        pids = []
        for stop in (signal.SIGTERM, signal.SIGINT):
            server = self.start_tools()
            instance = self.ready(server)["instance_id"]
            status, _, body = server.http.request("POST", "/api/v1/publish", instance=instance, body={
                "topic": "/tools_it/latched", "messageType": "std_msgs/String",
                "message": {"data": "x"}, "publishCount": 1, "publishRateHz": 1,
                "latch": True, "queueSize": 1, "waitForSubscribersMs": 0})
            self.assertEqual(status, 200, body)
            self.assertTrue(registrations(self.roscore, server.node)["publishers"])
            pids.append(server.pid)
            server.signal(stop)
            self.assert_gone(server)
        self.assertEqual(len(set(pids)), 2)
        # A latched topic of a ended Run is not left in the master.
        self.assertNotIn("/tools_it/latched", self.roscore.system_state()["publishers"])

    def test_the_node_name_has_to_carry_the_run_id(self):
        server = ToolsServer(self.workspace, self.roscore, new_run_id(), name="xgc_name_without_the_run")
        out = subprocess.run(server.command(), env=self.roscore.environment(),
                             capture_output=True, timeout=30)
        self.assertEqual(out.returncode, 2, out.stderr)
        self.assertIn(b"--node-name must include the --run-id", out.stderr)
        self.assertFalse(server.socket.exists())

    def test_the_process_is_unusable_without_its_run_arguments(self):
        out = subprocess.run([str(ToolsServer(self.workspace, self.roscore, "x").executable),
                              "--socket-path", "/tmp/unused"], capture_output=True, timeout=30)
        self.assertEqual(out.returncode, 2)
        self.assertIn(b"usage", out.stderr)


class ToolsMasterLossTest(unittest.TestCase):
    def setUp(self):
        self.workspace = Workspace()
        self.addCleanup(self.workspace.close)
        self.roscore = Roscore(self.workspace).start()

    def started(self):
        server = ToolsServer(self.workspace, self.roscore, new_run_id()).start()
        status, _, body = server.http.describe()
        self.assertEqual(status, 200, body)
        status, _, body = server.http.describe(instance=body["instance_id"], wait_ms=20000)
        self.assertTrue(body["ready"], body)
        return server

    def test_a_restarted_master_ends_the_process(self):
        server = self.started()
        self.roscore.restart()
        self.assertEqual(server.wait(60), EXIT_MASTER_CHANGED, server.output())

    def test_a_master_that_stays_away_ends_the_process(self):
        server = self.started()
        started = time.monotonic()
        self.roscore.kill()
        self.assertEqual(server.wait(60), EXIT_MASTER_UNREACHABLE, server.output())
        self.assertGreater(time.monotonic() - started, 4)

    def test_a_master_that_is_never_there_ends_the_process(self):
        self.roscore.kill()
        server = ToolsServer(self.workspace, self.roscore, new_run_id()).start()
        self.assertEqual(server.wait(60), EXIT_MASTER_UNREACHABLE, server.output())

    def test_ros_shutting_the_node_down_ends_the_process(self):
        server = self.started()
        subprocess.check_call(["rosnode", "kill", server.node], env=self.roscore.environment(),
                              timeout=30, stdout=subprocess.DEVNULL)
        self.assertEqual(server.wait(60), EXIT_ROS_SHUTDOWN, server.output())


if __name__ == "__main__":
    unittest.main()
