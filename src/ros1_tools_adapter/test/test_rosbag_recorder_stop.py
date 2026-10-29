"""Real process/signal regressions for the native recorder's owned shutdown.

The recorder never preempts the process-tree owner's shutdown budget: it
forwards the signal to its rosbag child and waits for it. A requested stop is an
intent, so success requires a clean child exit and one verified issued bag.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from test_rosbag_record_facts import RECORDER_TRANSPORT_STUB, RUNTIME_SCRIPT

BAG_BYTES = b"owned-native-recording"


class NativeRecorderStopTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.process = None
        self.addCleanup(self.stop_group)

    def stop_group(self):
        if self.process is not None:
            # On a regression failure, let the recorder reap its child before
            # falling back to killing the isolated test process group.
            if self.process.poll() is None and hasattr(self, "child_pid"):
                try:
                    os.kill(self.child_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    self.process.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    pass
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.communicate(timeout=3)

    def start_recorder(self, mode="cooperative"):
        """Start the recorder against a real stub rosbag child.

        Modes: cooperative renames its closed volume and exits 0; empty-topic
        renames a data-free but issued volume; partial exits 0 leaving only
        ``.bag.active``; empty exits 0 with no bag at all; ignore never finishes
        finalizing, so only the owner can escalate.
        """
        root = self.directory / "ros"
        native = root / "lib/rosbag/record"
        native.parent.mkdir(parents=True)
        native.write_text('''#!/usr/bin/env python3
import os, signal, sys, time
from pathlib import Path
prefix = sys.argv[sys.argv.index('--output-name') + 1]
active = Path(prefix + '_0.bag.active')
mode = os.environ['STOP_MODE']
stopping = False
def stop(signum, frame):
    global stopping
    Path(os.environ['STOP_OBSERVED']).write_text(str(signum))
    if mode != 'ignore':
        stopping = True
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
if mode in ('cooperative', 'partial', 'ignore'):
    active.write_bytes(b'owned-native-recording')
elif mode == 'empty-topic':
    active.write_bytes(b'#ROSBAG V2.0\\n')
Path(os.environ['CHILD_READY']).write_text(str(os.getpid()))
while not stopping:
    time.sleep(.01)
if mode in ('partial', 'empty'):
    sys.exit(0)
active.rename(str(active)[:-7])
''')
        native.chmod(0o700)
        topics = ["/tf"]
        manifest = {"schemaVersion": 1, "recordNamingVersion": 1, "recordingId": "c" * 32,
                    "recordingProfile": "general", "topics": topics, "estimatedBytes": 0,
                    "experimentName": "Stop regression", "runMode": "simulation", "targetId": "local"}
        self.archive = self.directory / "Experiments" / "Data"
        command = [sys.executable, "-c", RECORDER_TRANSPORT_STUB + RUNTIME_SCRIPT,
                   str(root), str(self.archive / "record"), json.dumps(topics),
                   "64", "1", "1", "none", "true", "1", "0", "1", json.dumps(manifest)]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                        text=True, start_new_session=True, env=dict(os.environ,
                                        XGC_USER_FILES_DIR=str(self.directory), XGC_RECORDER_RUN_ID="stop-test",
                                        XGC_RECORDER_SOCKET=str(self.directory / "rpc.sock"),
                                        STOP_MODE=mode,
                                        STOP_OBSERVED=str(self.directory / "signal-observed"),
                                        CHILD_READY=str(self.directory / "child-ready")))
        deadline = time.monotonic() + 5
        while not (self.directory / "child-ready").exists() and self.process.poll() is None and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue((self.directory / "child-ready").exists(), "native recorder child did not start")
        self.child_pid = int((self.directory / "child-ready").read_text())

    def manifest(self):
        return json.loads((self.archive / "session-manifest.json").read_text())

    def cancel_and_read(self, signum, returncode=0):
        started = time.monotonic()
        self.process.send_signal(signum)
        try:
            stdout, stderr = self.process.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            raise AssertionError("native recorder did not bound cancellation of its real child within 8s") from None
        self.assertLess(time.monotonic() - started, 8)
        self.assertEqual(self.process.returncode, returncode, stdout + stderr)
        self.assertEqual(int((self.directory / "signal-observed").read_text()), signum)
        with self.assertRaises(ProcessLookupError):
            os.kill(self.child_pid, 0)
        manifest = self.manifest()
        self.assertEqual(manifest["terminationSignal"], signum)
        self.assertEqual(manifest["recordFacts"]["status"], "closed")
        return manifest

    def test_sigterm_publishes_the_issued_bag(self):
        self.start_recorder("cooperative")
        manifest = self.cancel_and_read(signal.SIGTERM)
        self.assertEqual(manifest["recorderExitCode"], 0)
        self.assertEqual(manifest["status"], "stopped")
        self.assertEqual(manifest["archiveFinalizationState"], "finalized")
        self.assertEqual(manifest["bagFiles"], [{"id": "bag." + "c" * 32 + ".0", "name": "record.bag",
                                               "state": "finalized", "sizeBytes": len(BAG_BYTES)}])
        self.assertEqual((self.archive / "record.bag").read_bytes(), BAG_BYTES)

    def test_sigint_publishes_the_issued_bag(self):
        self.start_recorder("cooperative")
        manifest = self.cancel_and_read(signal.SIGINT)
        self.assertEqual(manifest["recorderExitCode"], 0)
        self.assertEqual(manifest["status"], "stopped")
        self.assertEqual(manifest["archiveFinalizationState"], "finalized")
        self.assertEqual(manifest["bagFiles"][0]["name"], "record.bag")

    def test_data_free_issued_bag_is_a_successful_capture(self):
        # An authored physical Scene may supply no data; the issued volume is
        # still the artifact. Emptiness is not failure, absence is.
        self.start_recorder("empty-topic")
        manifest = self.cancel_and_read(signal.SIGTERM)
        self.assertEqual(manifest["status"], "stopped")
        self.assertEqual(manifest["archiveFinalizationState"], "finalized")
        self.assertEqual(manifest["bagFiles"], [{"id": "bag." + "c" * 32 + ".0", "name": "record.bag",
                                               "state": "finalized", "sizeBytes": len(b"#ROSBAG V2.0\n")}])
        self.assertEqual((self.archive / "record.bag").read_bytes(), b"#ROSBAG V2.0\n")

    def test_clean_child_without_an_issued_bag_fails(self):
        # The child exits 0 on the forwarded signal but never finalizes its
        # volume, so only a partial `.bag.active` exists. A requested stop is
        # not evidence of a capture.
        self.start_recorder("partial")
        manifest = self.cancel_and_read(signal.SIGTERM, returncode=1)
        self.assertEqual(manifest["recorderExitCode"], 0)
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["archiveFinalizationState"], "failed")
        self.assertIn("no closed bag volume was issued", manifest["archiveFinalizationError"])
        self.assertEqual(manifest["bagFiles"], [{"name": "record_0.bag.active", "state": "partial",
                                               "sizeBytes": len(BAG_BYTES)}])
        self.assertTrue((self.archive / "record_0.bag.active").exists())
        self.assertFalse((self.archive / "record.bag").exists())

    def test_clean_child_with_no_bag_fails(self):
        self.start_recorder("empty")
        manifest = self.cancel_and_read(signal.SIGTERM, returncode=1)
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["archiveFinalizationState"], "failed")
        self.assertEqual(manifest["bagFiles"], [])
        self.assertFalse((self.archive / "record.bag").exists())

    def test_archive_rename_failure_fails(self):
        self.start_recorder("cooperative")
        foreign = self.archive / "record.bag"
        foreign.write_bytes(b"foreign-bytes")
        manifest = self.cancel_and_read(signal.SIGTERM, returncode=1)
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["archiveFinalizationState"], "failed")
        self.assertIn("already exists", manifest["archiveFinalizationError"])
        self.assertEqual(foreign.read_bytes(), b"foreign-bytes")
        self.assertTrue((self.archive / "record_0.bag").exists())

    def test_owner_escalation_reaps_an_ignoring_child_without_success(self):
        # The adapter must not preempt the owner's shutdown budget: a child that
        # ignores the forwarded signal keeps the adapter waiting past the old
        # hidden 5 s deadline. Only the owner's SIGKILL ends the group.
        self.start_recorder("ignore")
        self.process.send_signal(signal.SIGTERM)
        observed = self.directory / "signal-observed"
        deadline = time.monotonic() + 5
        while not observed.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertEqual(int(observed.read_text()), signal.SIGTERM)
        time.sleep(6.0)
        self.assertIsNone(self.process.poll(), "the adapter preempted the owner's shutdown budget")
        os.kill(self.child_pid, 0)  # the ignoring child is still physically alive
        manifest = self.manifest()
        self.assertEqual(manifest["stopRequestSignal"], signal.SIGTERM)
        self.assertIn("stopRequestedAt", manifest)
        self.assertNotIn(manifest["status"], ("stopped", "completed"))
        self.assertNotEqual(manifest.get("archiveFinalizationState"), "finalized")

        os.killpg(self.process.pid, signal.SIGKILL)
        self.process.communicate(timeout=5)
        self.assertEqual(self.process.returncode, -signal.SIGKILL)

        def stopped(pid):
            try:
                state = Path("/proc/{}/stat".format(pid)).read_text().split(") ", 1)[1][0]
            except (FileNotFoundError, IndexError):
                return True
            return state == "Z"  # dead, awaiting the init reaper

        deadline = time.monotonic() + 5
        while not stopped(self.child_pid) and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(stopped(self.child_pid), "the escalated child is still running")
        self.assertTrue((self.archive / "record_0.bag.active").exists())
        self.assertFalse((self.archive / "record.bag").exists())
        self.assertNotEqual(self.manifest()["status"], "stopped")


if __name__ == "__main__":
    unittest.main()
