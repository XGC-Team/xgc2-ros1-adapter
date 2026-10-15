"""Real process/signal regressions for the native recorder's owned shutdown."""
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

    def start_recorder(self, cooperative):
        root = self.directory / "ros"
        native = root / "lib/rosbag/record"
        native.parent.mkdir(parents=True)
        native.write_text('''#!/usr/bin/env python3
import json, os, signal, sys, time
from pathlib import Path
bag=Path(sys.argv[sys.argv.index('--output-name')+1]+'_0.bag.active')
stopping=False
def stop(signum, frame):
    global stopping
    Path(os.environ['STOP_OBSERVED']).write_text(str(signum))
    if os.environ['STOP_COOPERATIVE']=='true':
        stopping=True
signal.signal(signal.SIGTERM,stop)
signal.signal(signal.SIGINT,stop)
bag.write_bytes(b'owned-native-recording')
Path(os.environ['CHILD_READY']).write_text(str(os.getpid()))
while not stopping:
    time.sleep(.01)
bag.rename(str(bag)[:-7])
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
                                        STOP_COOPERATIVE=str(cooperative).lower(),
                                        STOP_OBSERVED=str(self.directory / "signal-observed"),
                                        CHILD_READY=str(self.directory / "child-ready")))
        deadline = time.monotonic() + 5
        while not (self.directory / "child-ready").exists() and self.process.poll() is None and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue((self.directory / "child-ready").exists(), "native recorder child did not start")
        self.child_pid = int((self.directory / "child-ready").read_text())

    def cancel_and_read(self, signum):
        started = time.monotonic()
        self.process.send_signal(signum)
        try:
            stdout, stderr = self.process.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            raise AssertionError("native recorder did not bound cancellation of its real child within 8s") from None
        self.assertLess(time.monotonic() - started, 8)
        self.assertEqual(self.process.returncode, 0, stdout + stderr)
        self.assertEqual(int((self.directory / "signal-observed").read_text()), signum)
        with self.assertRaises(ProcessLookupError):
            os.kill(self.child_pid, 0)
        manifest = json.loads((self.archive / "session-manifest.json").read_text())
        self.assertEqual(manifest["terminationSignal"], signum)
        self.assertEqual(manifest["status"], "stopped")
        self.assertEqual(manifest["recordFacts"]["status"], "closed")
        return manifest

    def test_sigterm_reaps_a_native_child_that_keeps_recording(self):
        self.start_recorder(cooperative=False)
        manifest = self.cancel_and_read(signal.SIGTERM)
        self.assertEqual(manifest["recorderExitCode"], -signal.SIGKILL)
        self.assertEqual(manifest["bagFiles"], [{"name": "record_0.bag.active", "state": "partial",
                                               "sizeBytes": len(b"owned-native-recording")}])
        self.assertTrue((self.archive / "record_0.bag.active").exists())
        self.assertFalse((self.archive / "record.bag").exists())

    def test_sigint_reaps_a_native_child_that_keeps_recording(self):
        self.start_recorder(cooperative=False)
        manifest = self.cancel_and_read(signal.SIGINT)
        self.assertEqual(manifest["recorderExitCode"], -signal.SIGKILL)
        self.assertEqual(manifest["bagFiles"][0]["state"], "partial")

    def test_cooperative_cancel_keeps_the_native_closed_bag(self):
        self.start_recorder(cooperative=True)
        manifest = self.cancel_and_read(signal.SIGTERM)
        self.assertEqual(manifest["recorderExitCode"], 0)
        self.assertEqual(manifest["bagFiles"], [{"id": "bag." + "c" * 32 + ".0", "name": "record.bag",
                                               "state": "finalized", "sizeBytes": len(b"owned-native-recording")}])
        self.assertEqual((self.archive / "record.bag").read_bytes(), b"owned-native-recording")


if __name__ == "__main__":
    unittest.main()
