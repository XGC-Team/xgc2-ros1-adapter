import io
import contextlib
import json
import tempfile
import time
import pathlib
import sys
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(pathlib.Path(__file__).parents[1]/'python'))
from xgc_mocap_calibration.solver import solve, InsufficientData, Stability
from xgc_mocap_calibration.mavlink import Link, checksum
from xgc_mocap_calibration.worker import Capture


def synthetic():
    step=.01;t=np.arange(0,38,step)
    r=Rotation.from_euler('xyz',np.column_stack([.5*np.sin(t*1.4),.4*np.sin(t*.9+.5),.6*np.sin(t*1.1+.7)]))
    rb=r.as_matrix();w=(r[:-1].inv()*r[1:]).as_rotvec()/step
    # Angular interval samples are centered, as on a physical gyroscope.
    g=np.vstack([np.mean(np.stack([np.vstack([w[0],w]),np.vstack([w,w[-1]])]),axis=0)])
    pb=np.column_stack([.3*np.sin(.7*t),.2*np.sin(.8*t),.2*np.cos(.6*t)])
    rkb=Rotation.from_euler('xyz',[.2,-.1,.4]);lever=np.array([-.05,.02,-.04])
    pm=pb+np.einsum('nij,j->ni',rb,lever)
    acc=np.column_stack([-.3*.7**2*np.sin(.7*t),-.2*.8**2*np.sin(.8*t),-.2*.6**2*np.cos(.6*t)])
    f=np.einsum('nji,nj->ni',rb,acc-np.array([0,0,-9.80665]))
    return t+.047,pm,(r*rkb.inv()).as_quat(),t,g,f,rkb,lever


class CalibrationTest(unittest.TestCase):
    def test_missing_original_interval_never_changes_device_rate(self):
        class UnknownRateLink:
            changed=[]
            component=1
            def __init__(self,*args):pass
            def heartbeat(self):pass
            def wait(self,*args,**kwargs):return bytes(9)
            def interval(self):return 0
            def set_interval(self,value):self.changed.append(value)
            def close(self):pass
        with tempfile.TemporaryDirectory() as tmp:
            lease=dict(schemaVersion=1,phase='prepared',directory=tmp,lockPath=str(pathlib.Path(tmp)/'asset.lock'),
                       assetId='asset',assetCommitId='commit',agentId='agent',systemId=7,runId='run',nodeId='collect',resourceId='rate-run',agentBootId='boot')
            lease_path=pathlib.Path(tmp)/'lease.json';lease_path.write_text(json.dumps(lease))
            capture=Capture(dict(lease,rateLeasePath=str(lease_path),assetName='UAV',tracker='uav7'))
            with patch.dict(sys.modules,{'zenoh':object()}),patch('xgc_mocap_calibration.worker.Link',UnknownRateLink),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(capture.execute(),1)
            self.assertEqual(UnknownRateLink.changed,[])
            self.assertTrue(json.loads((pathlib.Path(tmp)/'cleanup.json').read_text())['rateRestored'])

    def test_recovers_known_transform_without_confusing_delay(self):
        t,p,q,it,g,f,rkb,lever=synthetic()
        result=solve(t,p,q,it,g,f)
        self.assertLess(abs(result['identifiedDelayMs']-47),.2)
        self.assertLess(np.degrees((Rotation.from_quat(result['rotation']).inv()*rkb).magnitude()),.1)
        self.assertLess(np.linalg.norm(result['translation']+rkb.apply(lever)),.003)
        self.assertTrue(result['qualified'],result['reasons'])
        self.assertNotIn('localization_delay_ms',result)

    def test_stationary_and_one_axis_cannot_claim_observability(self):
        t,p,q,it,g,f,_,_=synthetic()
        with self.assertRaises(InsufficientData):
            solve(t,p,np.tile([0,0,0,1],(len(t),1)),it,np.zeros_like(g),f)
        one=Rotation.from_euler('z',.5*np.sin(it)).as_quat()
        z=np.column_stack([np.zeros(len(it)),np.zeros(len(it)),.5*np.cos(it)])
        result=solve(t,p,one,it,z,f)
        self.assertFalse(result['qualified'])
        self.assertIn('rotate around more than one axis',result['reasons'])

    def test_pose_teleport_is_excluded(self):
        t,p,q,it,g,f,_,_=synthetic();p=p.copy();p[1700:1705]+=10
        result=solve(t,p,q,it,g,f)
        self.assertGreaterEqual(result['excludedIntervals'],2)
        self.assertLess(abs(result['identifiedDelayMs']-47),.3)

    def test_stability_requires_repeated_qualified_results(self):
        state=Stability();t,p,q,it,g,f,_,_=synthetic();result=solve(t,p,q,it,g,f)
        for i in range(3):self.assertFalse(state.update(dict(result))['converged'])
        self.assertTrue(state.update(dict(result))['converged'])
        invalid=dict(result,qualified=False);self.assertFalse(state.update(invalid)['converged'])

    def test_scaled_gyro_cannot_claim_convergence(self):
        t,p,q,it,g,f,_,_=synthetic()
        result=solve(t,p,q,it,g*2,f)
        self.assertFalse(result['qualified'])
        self.assertIn('gyro and mocap angular motion disagree',result['reasons'])

    def test_large_delay_and_imu_gap_preserve_transform(self):
        t,p,q,it,g,f,rkb,lever=synthetic();t=t-.047+.180
        keep=(it<15)|(it>15.32)
        result=solve(t,p,q,it[keep],g[keep],f[keep])
        self.assertTrue(result['qualified'],result['reasons'])
        self.assertLess(abs(result['identifiedDelayMs']-180),.3)
        self.assertLess(np.linalg.norm(result['translation']+rkb.apply(lever)),.004)
        self.assertGreater(result['excludedIntervals'],0)

    def test_frozen_pose_or_lost_clock_revokes_old_candidate_until_new_solve(self):
        for failure in ('pose','clock'):
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as tmp:
                lease=dict(schemaVersion=1,phase='prepared',directory=tmp,lockPath=str(pathlib.Path(tmp)/'asset.lock'),
                           assetId='asset',assetCommitId='commit',agentId='agent',systemId=7,runId='run',nodeId='collect',resourceId='rate-run',agentBootId='boot')
                lease_path=pathlib.Path(tmp)/'lease.json';lease_path.write_text(json.dumps(lease))
                capture=Capture(dict(lease,rateLeasePath=str(lease_path),assetName='UAV',tracker='uav7'))
                now=time.monotonic()
                capture.result={'converged':True};capture.health_valid=True
                capture.last_pose=capture.last_imu=capture.last_sync=capture.last_distinct_pose=now
                if failure=='pose':capture.last_distinct_pose=now-1
                else:capture.last_sync=now-4
                with contextlib.redirect_stdout(io.StringIO()):capture.publish()
                self.assertEqual(capture.epoch,1);self.assertIsNone(capture.result)
                capture.last_sync=capture.last_distinct_pose=time.monotonic()
                with contextlib.redirect_stdout(io.StringIO()):capture.publish()
                state=json.loads((pathlib.Path(tmp)/'state.json').read_text())
                self.assertFalse(state['result'].get('converged',False))
                capture.lease_lock.close()

    def test_official_mavlink2_short_ack_and_nonflight_component(self):
        # Generated by the pinned common MAVLink C encoder; generation source
        # lives beside this test. The success ACK truncates its zero result.
        frames=[bytes.fromhex(line) for line in pathlib.Path(__file__).with_name('mavlink_calibration_frames.hex').read_text().splitlines()]
        class Socket:
            def recv(self,*args):raise AssertionError('fixture exhausted')
        link=object.__new__(Link);link.system_id=7;link.component=None
        link.socket=Socket();link.buffer=bytearray(b''.join(frames));link.sequence=0
        self.assertEqual(link.receive()[0],0)
        ack=link.receive();self.assertEqual(ack[0],77);self.assertEqual(len(ack[1]),2)
        self.assertEqual(link.component,1)
        link.interval_command=lambda *args:None
        link.wait=lambda *args:ack[1]
        link.interval=lambda:10000
        link.set_interval(10000)

    def test_protocol_writer_never_exposes_flight_commands(self):
        link=object.__new__(Link);link.sequence=0;link.system_id=7;link.component=1
        class Socket:
            def __init__(self):self.data=[]
            def sendall(self,data):self.data.append(data)
        link.socket=Socket()
        link.heartbeat();link.timesync(123);link.interval_command(511,10000)
        for data in link.socket.data:
            self.assertEqual(int.from_bytes(data[-2:],'little'),checksum(data[1:-2]+bytes([{0:50,111:34,76:152}[data[5]]])))
        with self.assertRaises(ValueError):link.interval_command(400,1)
        with self.assertRaises(ValueError):link.send(84,b'')

if __name__=='__main__':unittest.main()
