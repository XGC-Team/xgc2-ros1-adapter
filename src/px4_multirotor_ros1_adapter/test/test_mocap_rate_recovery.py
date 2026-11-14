import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / 'python'))
from xgc_mocap_calibration.rate_recovery import activate, atomic_json, read_lease, restore


class Link:
    calls = []
    component = 1
    interval = 10000
    reject = False

    def __init__(self, system):
        self.calls.append(('connect', system))

    def heartbeat(self):
        self.calls.append(('heartbeat',))

    def wait(self, *args, **kwargs):
        return bytes(9)

    def set_interval(self, value):
        self.calls.append(('set', value))
        if self.reject:
            raise RuntimeError('readback differs')
        self.interval = value

    def close(self):
        self.calls.append(('close',))


class RateRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.TemporaryDirectory()
        self.addCleanup(self.root.cleanup)
        self.path = Path(self.root.name) / 'lease.json'
        self.lease = dict(schemaVersion=1, phase='prepared', assetId='asset-7',
            assetCommitId='commit', agentId='agent-7', systemId=7, runId='run', nodeId='collect',
            resourceId='rate-run', agentBootId='boot-old', directory=self.root.name,
            lockPath=str(Path(self.root.name) / 'asset.lock'))
        atomic_json(self.path, self.lease)
        Link.calls, Link.component, Link.reject = [], 1, False

    def test_prepared_before_change_requires_no_device(self):
        result = restore(self.path, 'agent-7', Link)
        self.assertTrue(result['rateRestored'])
        self.assertEqual(Link.calls, [])
        self.assertEqual(read_lease(self.path)['phase'], 'restored')
        restore(self.path, 'agent-7', Link)
        self.assertEqual(Link.calls, [])

    def test_original_is_durable_before_change_and_failure_retains_it(self):
        activate(self.path, self.lease, 1, 100000)
        self.assertEqual(read_lease(self.path)['originalIntervalUs'], 100000)
        Link.reject = True
        with self.assertRaisesRegex(RuntimeError, 'readback differs'):
            restore(self.path, 'agent-7', Link)
        self.assertEqual(read_lease(self.path)['phase'], 'active')
        Link.reject = False
        result = restore(self.path, 'agent-7', Link)
        self.assertTrue(result['rateRestored'])
        self.assertEqual(read_lease(self.path)['phase'], 'restored')

    def test_later_restore_updates_receipts_without_hiding_worker_failure(self):
        cleanup = Path(self.root.name) / 'cleanup.json'
        state = Path(self.root.name) / 'state.json'
        atomic_json(cleanup, dict(rateRestored=False, failures=['worker interrupted']))
        atomic_json(state, dict(calibrationRunId='run', phase='collecting', rateRestored=False,
                               failures=['worker interrupted']))
        activate(self.path, self.lease, 1, 100000)
        restore(self.path, 'agent-7', Link)
        for path in (cleanup, state):
            record = json.loads(path.read_text())
            self.assertTrue(record['rateRestored'])
            self.assertEqual(record['failures'], ['worker interrupted'])
        self.assertEqual(json.loads(state.read_text())['phase'], 'stopped')

    def test_activation_fsyncs_file_and_directory_before_returning(self):
        events = []
        real_fsync, real_replace = os.fsync, os.replace
        def fsync(fd):
            events.append('fsync')
            return real_fsync(fd)
        def replace(source, destination):
            events.append('replace')
            return real_replace(source, destination)
        with patch('os.fsync', side_effect=fsync), patch('os.replace', side_effect=replace):
            activate(self.path, self.lease, 1, 100000)
        self.assertEqual(events, ['fsync', 'replace', 'fsync'])
        self.assertEqual(read_lease(self.path)['originalIntervalUs'], 100000)

    def test_wrong_agent_or_autopilot_component_never_changes_rate(self):
        activate(self.path, self.lease, 1, 100000)
        with self.assertRaisesRegex(ValueError, 'another Agent'):
            restore(self.path, 'agent-other', Link)
        Link.component = 100
        with self.assertRaisesRegex(RuntimeError, 'component changed'):
            restore(self.path, 'agent-7', Link)
        self.assertFalse(any(c[0] == 'set' for c in Link.calls))
        self.assertEqual(read_lease(self.path)['phase'], 'active')

    def test_capture_process_sigkill_leaves_independently_recoverable_lease(self):
        # A separate OS process takes the real advisory lock, persists the
        # original interval, then is killed without running any cleanup.
        code = '''import fcntl,json,pathlib,sys,time
from xgc_mocap_calibration.rate_recovery import activate
p=pathlib.Path(sys.argv[1]);v=json.loads(p.read_text())
lock=open(v['lockPath'],'a');fcntl.flock(lock,fcntl.LOCK_EX)
activate(p,v,1,100000)
print('active',flush=True)
time.sleep(60)
'''
        env = dict(os.environ, PYTHONPATH=str(Path(__file__).parents[1] / 'python'))
        process = subprocess.Popen([sys.executable, '-c', code, str(self.path)], env=env,
                                   stdout=subprocess.PIPE, text=True)
        self.addCleanup(lambda: process.kill() if process.poll() is None else None)
        self.assertEqual(process.stdout.readline().strip(), 'active')
        process.send_signal(signal.SIGKILL)
        self.assertEqual(process.wait(timeout=3), -signal.SIGKILL)
        process.stdout.close()
        result = restore(self.path, 'agent-7', Link)
        self.assertTrue(result['rateRestored'])
        self.assertIn(('set', 100000), Link.calls)
        self.assertEqual(json.loads((Path(self.root.name)/'rate-recovery.json').read_text())['componentId'], 1)


if __name__ == '__main__':
    unittest.main()
