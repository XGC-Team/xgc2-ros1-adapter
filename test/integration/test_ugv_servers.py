"""The UGV type server (Scout Mini and Mecanum) against a private roscore.

The vehicles are stood in for by fake_robots.py: it publishes the topics the
robot profiles read and records what the server publishes back. Everything
else is the installed executable, driven over its gRPC socket the way the
experiment workflow drives it.
"""

import json
import signal
import subprocess
import time
import unittest

from harness import (
    EXIT_MASTER_CHANGED, EXIT_MASTER_UNREACHABLE, EXIT_OK, EXIT_ROS_SHUTDOWN, EXIT_USAGE,
    MECANUM, SCOUT, MasterCase, Robot, Roscore, Server, Workspace,
    is_zero, motion_intent_payload, new_run_id, process_exists, registrations,
    top_name, wait_until,
)

PROFILES = ("gazebo.vrpn", "physical.vrpn", "xsim.ros")
SUCCEEDED = "OPERATION_PHASE_SUCCEEDED"


def unique(name):
    return "{}_{}".format(name, new_run_id()[4:])


class UgvServerTests:
    """What every UGV type server does; one concrete case per robot type."""

    provider = None
    # gear 3, forward, sideways, turning right: the limits of the chassis
    full_intent = None

    def robots(self, server, *profiles, prefix="bot"):
        return [Robot(self.provider, unique("{}{}".format(prefix, index)), profile,
                      run_id=server.run_id)
                for index, profile in enumerate(profiles)]

    def command_topic(self, robot):
        return top_name(robot.namespace, "cmd_vel")

    def last_command(self, fake, robot):
        entries = fake.received(self.command_topic(robot))
        return entries[-1]["msg"] if entries else None

    def wait_for_command(self, fake, robot, expected):
        def seen():
            message = self.last_command(fake, robot)
            return message is not None and all(
                abs(a - b) < 1e-6 for key in ("linear", "angular")
                for a, b in zip(message[key], expected[key]))
        wait_until(seen, 10, 0.05, "the command {} on {}".format(expected, self.command_topic(robot)))

    def execute(self, server, robot, endpoint, intent=None, work_id=None):
        reply = server.client.execute(
            robot.identity, endpoint, work_id or "work-" + new_run_id(),
            motion_intent_payload(*(intent or (1, 0, 0, 0))))
        self.assertEqual(reply["status"]["code"], 0, reply)
        return reply["response"]["event"]

    def assert_subscribed(self, server, robot, subscribed=True):
        node = self.node_of(server)
        subscribers = self.roscore.system_state()["subscribers"]
        for topic in robot.input_topics():
            self.assertEqual(node in subscribers.get(topic, ()), subscribed,
                             "{} on {}".format(node, topic))

    def test_the_node_name_has_to_carry_the_run_id(self):
        # A name two Runs could share would let the master displace one node.
        server = Server(self.workspace, self.roscore, self.provider, new_run_id(),
                        name="xgc_name_without_the_run")
        out = subprocess.run(server.command(), env=self.roscore.environment(),
                             capture_output=True, timeout=30)
        self.assertEqual(out.returncode, EXIT_USAGE, out.stderr)
        self.assertIn(b"--node-name must include the --run-id", out.stderr)
        self.assertFalse(server.socket.exists())

    # Describe --------------------------------------------------------------

    def test_describe_is_held_until_the_server_is_ready(self):
        server = self.start_server(self.provider)
        reply = server.client.describe(wait_ms=20000)
        self.assertEqual(reply["status"]["code"], 0, reply)
        description = reply["response"]
        self.assertTrue(description["ready"])
        facts = json.loads(description["facts_json"])
        self.assertEqual(facts["provider"], self.provider)
        self.assertEqual(facts["run_id"], server.run_id)
        self.assertEqual(facts["ros_master_uri"], self.roscore.uri)
        self.assertEqual(facts["ros_master_pid"], self.roscore.pid())
        self.assertEqual(facts["ros_master_run_id"], self.roscore.run_id())
        self.assertEqual(facts["ros_node"], self.node_of(server))
        self.assertEqual(facts["state"], "ready")
        self.assertEqual(facts["members"], 0)
        reference = description["service_ref"]
        self.assertEqual(reference["service"], "xgc2.robot-adapter")
        self.assertEqual(reference["profile"], "grpc.v1")
        self.assertEqual(reference["endpoint"]["address"], str(server.socket))
        self.assertTrue(reference["instance_id"])
        self.assertEqual(description["provider_definition_id"], self.provider)
        # The node the facts name is the node in the graph.
        self.assertIn(facts["ros_node"], self.roscore.nodes())
        # A wait beyond the limit is a usage error, not an unbounded hold.
        refused = server.client.describe(wait_ms=30001)
        self.assertEqual(refused["status"]["code"], 3, refused)

    # Members -----------------------------------------------------------------

    def test_one_apply_registers_robots_of_different_profiles(self):
        server = self.start_server(self.provider)
        self.ready(server)
        robots = self.robots(server, *PROFILES)
        self.start_fake_robots(robots)
        self.apply_all(server, robots)
        for robot in robots:
            self.assert_subscribed(server, robot)
        health = server.client.health()["response"]
        self.assertEqual((health["serving"], health["robots"], health["members"]), (True, 3, 3))
        self.assertEqual(json.loads(self.ready(server)["facts_json"])["members"], 3)

        watch = self.watch(server)

        def streaming():
            latest, removed = watch.members()
            return all({"state.imu", "state.power", "vrpn.position"} <= set(latest.get(r.robot_id, ()))
                       for r in robots)
        wait_until(streaming, 30, 0.1, "status of every robot")
        latest, _ = watch.members()
        for robot in robots:
            for message in latest[robot.robot_id].values():
                self.assertEqual(message["robot_id"], robot.robot_id)

    def test_member_failures_are_reported_per_member(self):
        server = self.start_server(self.provider)
        self.ready(server)
        good, foreign_run, unknown_profile, bad_epoch = self.robots(
            server, "xsim.ros", "xsim.ros", "xsim.ros", "xsim.ros")
        foreign_run.run_id = "another-run"
        unknown_profile.profile_id = "no-such-profile"
        bad_epoch.epoch = 0
        members = self.apply(server, [good, foreign_run, unknown_profile, bad_epoch])
        self.assertNotIn("error", members[0], members[0])
        for member in members[1:]:
            self.assertEqual(member["error"]["class"], "ERROR_CLASS_REJECTED", member)
            self.assertTrue(member["error"]["code"], member)
        self.assert_subscribed(server, good)
        self.assertEqual(server.client.health()["response"]["members"], 1)

        # The same registration again is a no-op; the same robot with another
        # configuration, or another connection epoch, is a conflict.
        self.assertNotIn("error", self.apply(server, [good])[0])
        changed = Robot(self.provider, good.robot_id, "xsim.ros", namespace=good.namespace,
                        run_id=server.run_id, positioning_frame_number="9")
        self.assertEqual(self.apply(server, [changed])[0]["error"]["code"],
                         "member-configuration-conflict")
        newer = Robot(self.provider, good.robot_id, "xsim.ros", namespace=good.namespace,
                      run_id=server.run_id, epoch=2)
        self.assertEqual(self.apply(server, [newer])[0]["error"]["code"], "member-epoch-conflict")
        self.assertEqual(server.client.health()["response"]["members"], 1)

    def test_remove_members_stops_the_robot_and_releases_its_ros_interface(self):
        server = self.start_server(self.provider)
        self.ready(server)
        keep, drop = self.robots(server, "gazebo.vrpn", "xsim.ros")
        fake = self.start_fake_robots([keep, drop])
        self.apply_all(server, [keep, drop])
        watch = self.watch(server)
        wait_until(lambda: {keep.robot_id, drop.robot_id} <= set(watch.members()[0]), 30, 0.1,
                   "status of both robots")

        event = self.execute(server, drop, "set-motion-intent", (3, 1, 1, -1))
        self.assertEqual(event["phase"], SUCCEEDED, event)
        self.wait_for_command(fake, drop, self.full_intent)

        reply = server.client.remove([drop.identity, keep.identity])
        self.assertEqual(reply["status"]["code"], 0, reply)
        self.assertTrue(all("error" not in m for m in reply["response"]["members"]), reply)
        # The robot that was commanded is stopped: a zero is the last it hears.
        wait_until(lambda: is_zero(self.last_command(fake, drop) or {"linear": [1]}), 10, 0.05,
                   "the final zero command")
        wait_until(lambda: {keep.robot_id, drop.robot_id} <= watch.members()[1], 10, 0.1,
                   "tombstones in the status stream")
        self.assert_subscribed(server, drop, False)
        self.assert_subscribed(server, keep, False)
        self.assertEqual(server.client.health()["response"]["members"], 0)

        # Removing again is not an error, and the server keeps serving.
        again = server.client.remove([drop.identity])
        self.assertEqual(again["status"]["code"], 0, again)
        self.apply_all(server, [keep])
        self.assert_subscribed(server, keep)

    # Commands ------------------------------------------------------------------

    def test_motion_intent_reaches_the_vehicle_and_release_stops_it(self):
        server = self.start_server(self.provider)
        self.ready(server)
        robot, other = self.robots(server, "xsim.ros", "physical.vrpn")
        fake = self.start_fake_robots([robot, other])
        self.apply_all(server, [robot, other])

        event = self.execute(server, robot, "set-motion-intent", (3, 1, 1, -1))
        self.assertEqual(event["phase"], SUCCEEDED, event)
        self.wait_for_command(fake, robot, self.full_intent)
        # The intent is republished while it holds.
        before = len(fake.received(self.command_topic(robot)))
        wait_until(lambda: len(fake.received(self.command_topic(robot))) >= before + 3, 5, 0.05,
                   "the intent to be republished")
        # The other robot is not driven.
        self.assertEqual(fake.received(self.command_topic(other)), [])

        event = self.execute(server, robot, "release-motion-intent", (1, 0, 0, 0))
        self.assertEqual(event["phase"], SUCCEEDED, event)
        self.wait_for_command(fake, robot, {"linear": [0, 0, 0], "angular": [0, 0, 0]})
        count = len(fake.received(self.command_topic(robot)))
        time.sleep(0.6)
        self.assertEqual(len(fake.received(self.command_topic(robot))), count)

        # Out-of-range intents and unknown operations are rejected, not forwarded.
        rejected = self.execute(server, robot, "set-motion-intent", (9, 0, 0, 0))
        self.assertNotEqual(rejected["phase"], SUCCEEDED, rejected)
        unknown = self.execute(server, robot, "no-such-operation")
        self.assertNotEqual(unknown["phase"], SUCCEEDED, unknown)
        unregistered = server.client.execute(
            dict(robot.identity, robot_id="nobody"), "set-motion-intent", "work-" + new_run_id(),
            motion_intent_payload(1, 0, 0, 0))
        self.assertNotEqual(unregistered["response"]["event"]["phase"], SUCCEEDED, unregistered)

    # Stopping -------------------------------------------------------------------

    def drive(self, server):
        """A server with two robots, one of them moving."""
        self.ready(server)
        moving, idle = self.robots(server, "gazebo.vrpn", "xsim.ros")
        fake = self.start_fake_robots([moving, idle])
        self.apply_all(server, [moving, idle])
        event = self.execute(server, moving, "set-motion-intent", (3, 1, 1, -1))
        self.assertEqual(event["phase"], SUCCEEDED, event)
        self.wait_for_command(fake, moving, self.full_intent)
        return moving, idle, fake

    def assert_stopped_cleanly(self, server, moving, idle, fake):
        self.assert_gone(server, EXIT_OK)
        self.assertTrue(is_zero(self.last_command(fake, moving)), self.last_command(fake, moving))
        self.assertEqual(fake.received(self.command_topic(idle)), [])
        self.assertFalse(server.socket.exists())

    def test_shutdown_rpc_publishes_the_final_zero_and_ends_the_process(self):
        server = self.start_server(self.provider)
        moving, idle, fake = self.drive(server)
        reply = server.client.shutdown()
        self.assertEqual(reply["status"]["code"], 0, reply)
        self.assertFalse(reply["response"]["serving"])
        self.assert_stopped_cleanly(server, moving, idle, fake)
        self.assertIn("exit 0", server.output())

    def test_sigterm_publishes_the_final_zero_and_ends_the_process(self):
        server = self.start_server(self.provider)
        moving, idle, fake = self.drive(server)
        server.signal(signal.SIGTERM)
        self.assert_stopped_cleanly(server, moving, idle, fake)

    def test_a_started_but_unused_server_stops_at_once(self):
        server = self.start_server(self.provider)
        self.ready(server)
        started = time.monotonic()
        server.signal(signal.SIGTERM)
        self.assert_gone(server, EXIT_OK)
        self.assertLess(time.monotonic() - started, 5)

    # Runs one after the other ----------------------------------------------------

    def test_consecutive_runs_with_different_robots_leave_nothing_behind(self):
        history = []
        for stop in ("shutdown", "sigterm", "shutdown"):
            run = new_run_id()
            server = self.start_server(self.provider, run)
            self.ready(server)
            count = 2 + len(history) % 2
            robots = self.robots(server, *PROFILES[:count], prefix="cycle{}_".format(len(history)))
            fake = self.start_fake_robots(robots)
            self.apply_all(server, robots)
            for robot in robots:
                self.assert_subscribed(server, robot)
            for earlier in history:
                for robot in earlier["robots"]:
                    self.assert_subscribed(server, robot, False)
                self.assertNotEqual(earlier["node"], self.node_of(server))
            registered = registrations(self.roscore, self.node_of(server))
            self.assertTrue(registered["subscribers"] and registered["publishers"], registered)
            pid = server.pid

            if stop == "shutdown":
                self.assertEqual(server.client.shutdown()["status"]["code"], 0)
            else:
                server.signal(signal.SIGTERM)
            self.assert_gone(server, EXIT_OK)
            self.assertFalse(process_exists(pid))
            fake.cleanup()
            history.append({"robots": robots, "node": self.node_of(server), "pid": pid})
        self.assertEqual(len({entry["pid"] for entry in history}), 3)

    def test_thread_count_does_not_depend_on_the_number_of_members(self):
        server = self.start_server(self.provider)
        self.ready(server)
        counts, members = {}, 0
        for target in (1, 20, 100):
            batch = [Robot(self.provider, unique("t{}".format(index)), "xsim.ros",
                           run_id=server.run_id)
                     for index in range(members, target)]
            self.assertTrue(all(
                "error" not in m for m in
                server.client.apply([r.registration() for r in batch],
                                    deadline_ms=120000)["response"]["members"]))
            members = target
            self.assertEqual(server.client.health()["response"]["members"], members)
            time.sleep(1.0)
            counts[members] = server.threads()
        print("threads by members: {}".format(counts))
        self.assertLessEqual(max(counts.values()) - min(counts.values()), 2, counts)


class ScoutServerTest(UgvServerTests, MasterCase):
    provider = SCOUT
    # 1.5 m/s and 0.5235 rad/s at gear 3; a Scout cannot move sideways
    full_intent = {"linear": [1.5, 0.0, 0.0], "angular": [0.0, 0.0, -0.5235]}


class MecanumServerTest(UgvServerTests, MasterCase):
    provider = MECANUM
    # 1.0 m/s and 90 deg/s at gear 3; a Mecanum chassis moves sideways too
    full_intent = {"linear": [1.0, 1.0, 0.0], "angular": [0.0, 0.0, -1.5707963267948966]}


class MixedProvidersTest(MasterCase):
    """A UGV executable serves one robot type per process."""

    def test_the_provider_is_fixed_by_the_command_line(self):
        server = self.start_server(SCOUT)
        self.ready(server)
        mecanum = Robot(MECANUM, unique("mix"), "xsim.ros", run_id=server.run_id)
        reply = server.client.apply([mecanum.registration()])
        member = reply["response"]["members"][0]
        self.assertEqual(member["error"]["class"], "ERROR_CLASS_REJECTED", reply)
        self.assertEqual(server.client.health()["response"]["members"], 0)
        server.signal(signal.SIGTERM)
        self.assert_gone(server, EXIT_OK)

    def test_two_robot_types_share_a_master_without_interfering(self):
        scout = self.start_server(SCOUT)
        mecanum = self.start_server(MECANUM)
        self.ready(scout), self.ready(mecanum)
        scout_robot = Robot(SCOUT, unique("sc"), "xsim.ros", run_id=scout.run_id)
        mecanum_robot = Robot(MECANUM, unique("mc"), "xsim.ros", run_id=mecanum.run_id)
        self.start_fake_robots([scout_robot, mecanum_robot])
        self.apply_all(scout, [scout_robot])
        self.apply_all(mecanum, [mecanum_robot])
        self.assertNotEqual(scout.name, mecanum.name)
        scout.signal(signal.SIGTERM)
        self.assert_gone(scout, EXIT_OK)
        # The other Run's server and its robot are untouched.
        self.assertIn(self.node_of(mecanum), self.roscore.nodes())
        self.assertEqual(mecanum.client.health()["response"]["members"], 1)
        mecanum.signal(signal.SIGTERM)
        self.assert_gone(mecanum, EXIT_OK)


class MasterLossTest(unittest.TestCase):
    """The server is bound to the master it started with."""

    def setUp(self):
        self.workspace = Workspace()
        self.addCleanup(self.workspace.close)
        self.roscore = Roscore(self.workspace).start()

    def start_server(self, provider=SCOUT, robots=1):
        server = Server(self.workspace, self.roscore, provider, new_run_id()).start()
        reply = server.client.describe(wait_ms=20000)
        self.assertTrue(reply["response"]["ready"], reply)
        members = [Robot(provider, unique("loss{}".format(i)), "xsim.ros", run_id=server.run_id)
                   for i in range(robots)]
        if members:
            reply = server.client.apply([m.registration() for m in members])
            self.assertTrue(all("error" not in m for m in reply["response"]["members"]), reply)
        return server

    def test_a_restarted_master_ends_the_server(self):
        server = self.start_server()
        first = self.roscore.pid()
        self.roscore.restart()
        self.assertNotEqual(self.roscore.pid(), first)
        self.assertEqual(server.wait(60), EXIT_MASTER_CHANGED, server.output())
        self.assertIn("master", server.output())
        self.assertFalse(process_exists(server.pid))

    def test_a_master_that_stays_away_ends_the_server(self):
        server = self.start_server()
        started = time.monotonic()
        self.roscore.kill()
        self.assertEqual(server.wait(60), EXIT_MASTER_UNREACHABLE, server.output())
        # unreachable for 5 s, then the drain; not an immediate exit
        self.assertGreater(time.monotonic() - started, 4)

    def test_a_brief_master_outage_is_survived_by_the_same_master(self):
        # The master process is stopped, not replaced: it answers again within
        # the budget with the same identity, so the server keeps serving.
        server = self.start_server()
        self.roscore.stop_responding()
        time.sleep(2.5)
        self.roscore.resume()
        time.sleep(2.0)
        self.assertFalse(server.exited(), server.output())
        self.assertEqual(server.client.health()["response"]["members"], 1)
        server.signal(signal.SIGTERM)
        self.assertEqual(server.wait(60), EXIT_OK, server.output())

    def test_a_silent_master_does_not_block_the_server(self):
        server = self.start_server(robots=0)
        silent_since = time.monotonic()
        self.roscore.stop_responding()
        # Management calls to the master are bounded, so the server keeps
        # answering its own clients while the master is silent ...
        started = time.monotonic()
        health = server.client.health()
        self.assertEqual(health["status"]["code"], 0, health)
        self.assertLess(time.monotonic() - started, 2.0)
        # ... a registration that needs the master fails within its budget ...
        robot = Robot(SCOUT, unique("silent"), "xsim.ros", run_id=server.run_id)
        started = time.monotonic()
        reply = server.client.apply([robot.registration()], deadline_ms=30000)
        elapsed = time.monotonic() - started
        self.assertEqual(reply["status"]["code"], 0, reply)
        self.assertEqual(reply["response"]["members"][0]["error"]["code"],
                         "member-registration-failed", reply)
        self.assertLess(elapsed, 12.0, reply)
        # ... and the server ends once the master has been silent too long.
        self.assertEqual(server.wait(90), EXIT_MASTER_UNREACHABLE, server.output())
        print("silent master: apply failed after {:.1f} s, server exited with {} after {:.1f} s".format(
            elapsed, EXIT_MASTER_UNREACHABLE, time.monotonic() - silent_since))

    def test_a_server_whose_master_is_never_there_exits(self):
        self.roscore.kill()
        server = Server(self.workspace, self.roscore, SCOUT, new_run_id()).start()
        started = time.monotonic()
        self.assertEqual(server.wait(90), EXIT_MASTER_UNREACHABLE, server.output())
        self.assertLess(time.monotonic() - started, 60)

    def test_a_node_of_the_same_name_displaces_the_first(self):
        # The master lets the newest node of a name win and tells the older one
        # to shut down: ROS ends the node, the server exits with that status. A
        # Run's node name carries its Run id, so only a process started again
        # for the same Run can meet this.
        first = self.start_server()
        second = Server(self.workspace, self.roscore, SCOUT, new_run_id(), name=first.name).start()
        reply = second.client.describe(wait_ms=20000)
        self.assertTrue(reply["response"]["ready"], reply)
        self.assertEqual(first.wait(60), EXIT_ROS_SHUTDOWN, first.output())
        self.assertFalse(second.exited(), second.output())
        self.assertEqual(second.client.health()["response"]["serving"], True)
        second.signal(signal.SIGTERM)
        self.assertEqual(second.wait(60), EXIT_OK, second.output())

    def test_ros_shutting_the_node_down_ends_the_server(self):
        server = self.start_server()
        subprocess.check_call(["rosnode", "kill", server.node],
                              env=self.roscore.environment(), timeout=30,
                              stdout=subprocess.DEVNULL)
        self.assertEqual(server.wait(60), EXIT_ROS_SHUTDOWN, server.output())


if __name__ == "__main__":
    unittest.main()
