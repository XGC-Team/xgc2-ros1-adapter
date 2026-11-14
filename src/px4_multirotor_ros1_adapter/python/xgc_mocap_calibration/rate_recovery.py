"""Durable, identity-bound recovery of the one temporary calibration interval.

This module has no NumPy/ROS/Zenoh dependency. The Agent can invoke it after the
capture process has disappeared, including after an Agent or kernel restart.
The journal is retired by the Agent only after the Run store acknowledges it.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import time

from .mavlink import Link


def atomic_json(path, value, durable=True):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, allow_nan=False, separators=(',', ':'))
        stream.flush()
        if durable:
            os.fsync(stream.fileno())
    os.replace(temporary, path)
    if durable:
        descriptor = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def read_lease(path, expected=None):
    lease = json.loads(Path(path).read_text())
    if lease.get('schemaVersion') != 1 or lease.get('phase') not in ('prepared', 'active', 'restored'):
        raise ValueError('unsupported calibration rate recovery journal')
    for key in ('assetId', 'agentId', 'runId', 'nodeId', 'resourceId', 'agentBootId', 'directory', 'lockPath'):
        if not isinstance(lease.get(key), str) or not lease[key]:
            raise ValueError('rate recovery requires ' + key)
    if type(lease.get('systemId')) is not int or not 1 <= lease['systemId'] <= 255:
        raise ValueError('invalid rate recovery system ID')
    if 'originalIntervalUs' in lease:
        if type(lease.get('componentId')) is not int or not 1 <= lease['componentId'] <= 255:
            raise ValueError('invalid rate recovery autopilot component')
        if type(lease.get('originalIntervalUs')) is not int or lease['originalIntervalUs'] == 0:
            raise ValueError('invalid original rate recovery interval')
    elif lease['phase'] == 'active':
        raise ValueError('active rate recovery has no original interval')
    if expected is not None:
        for key in ('assetId', 'agentId', 'runId', 'systemId', 'assetCommitId', 'directory'):
            if lease.get(key) != expected.get(key):
                raise ValueError('rate recovery identity mismatch: ' + key)
    return lease


def activate(path, expected, component, original):
    lease = read_lease(path, expected)
    if lease['phase'] != 'prepared':
        raise RuntimeError('previous calibration interval has not been retired')
    lease.update(phase='active', componentId=component, originalIntervalUs=original)
    # This rename AND its directory entry are durable before any SET command.
    atomic_json(path, lease)


def mark_restored(path):
    lease = read_lease(path)
    lease.update(phase='restored', restoredAtUnixSeconds=time.time())
    atomic_json(path, lease)


def restore(path, agent_id, link_factory=Link):
    path = Path(path)
    lease = read_lease(path)
    if lease['agentId'] != agent_id:
        raise ValueError('rate recovery belongs to another Agent')
    with Path(lease['lockPath']).open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # The worker may have restored the interval before releasing this lock.
        lease = read_lease(path)
        if lease['phase'] == 'active':
            link = link_factory(lease['systemId'])
            try:
                link.heartbeat()
                link.wait(0, seconds=6)
                if link.component != lease['componentId']:
                    raise RuntimeError('rate recovery autopilot component changed')
                link.set_interval(lease['originalIntervalUs'])  # includes readback
            finally:
                link.close()
        if lease['phase'] != 'restored':
            # Prepared means no rate-changing command could have been sent.
            # There is deliberately no device I/O for this crash boundary.
            lease.update(phase='restored', restoredAtUnixSeconds=time.time())
            atomic_json(path, lease)
        receipt = dict(rateRestored=True, recovered=True, runId=lease['runId'],
                       assetId=lease['assetId'], agentId=lease['agentId'],
                       systemId=lease['systemId'], componentId=lease.get('componentId'),
                       originalIntervalUs=lease.get('originalIntervalUs'), failures=[])
        atomic_json(Path(lease['directory']) / 'rate-recovery.json', receipt)
        # Keep prior worker failures as evidence, while correcting the later
        # authoritative restore outcome for archive readers and the panel.
        for name in ('cleanup.json', 'state.json'):
            record_path = Path(lease['directory']) / name
            try:
                record = json.loads(record_path.read_text())
                if not isinstance(record, dict):
                    continue
                if name == 'state.json' and record.get('calibrationRunId') != lease['runId']:
                    continue
                record.update(rateRestored=True, rateRecovery=receipt)
                if name == 'state.json':
                    record['phase'] = 'stopped'
                atomic_json(record_path, record)
            except (OSError, ValueError):
                # The independent receipt above is sufficient even if the
                # killed worker never produced its optional final files.
                pass
        return receipt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--lease', required=True, type=Path)
    parser.add_argument('--agent-id', required=True)
    args = parser.parse_args()
    try:
        print(json.dumps(restore(args.lease, args.agent_id)), flush=True)
    except Exception as error:
        print(json.dumps(dict(rateRestored=False, error=str(error))), flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
