"""Run-owned UAV capture/solve worker. stdout is typed progress, raw data is NDJSON.

Only reads flight state and temporarily changes HIGHRES_IMU rate. A durable rate
lease is written before the change; recovery precedes any new capture. Shutdown
restores and reads back the prior interval before reporting cleanup complete.
"""
import argparse
from collections import deque
import fcntl
import json
import os
from pathlib import Path
import queue
import signal
import struct
import threading
import time

import numpy as np
from scipy.optimize import least_squares
from .mavlink import Link
from .rate_recovery import atomic_json, read_lease, activate, mark_restored
from .solver import solve, Stability, InsufficientData


def proto_fields(buf):
    def vint(index):
        number=0;shift=0
        while index<len(buf) and shift<70:
            byte=buf[index];index+=1;number|=(byte&127)<<shift
            if byte<128:return number,index
            shift+=7
        raise ValueError('invalid protobuf integer')
    out={};i=0
    while i<len(buf):
        tag,i=vint(i);field,wire=tag>>3,tag&7
        if wire==0:value,i=vint(i)
        elif wire in (1,5):size=8 if wire==1 else 4;value=buf[i:i+size];i+=size
        elif wire==2:size,i=vint(i);value=buf[i:i+size];i+=size
        else:raise ValueError('unsupported protobuf wire')
        out[field]=value
    return out


def decode_pose(raw):
    fields=proto_fields(raw);stamp=proto_fields(fields[1])
    def vector(data,count):
        fs=proto_fields(data)
        return [struct.unpack('<d',fs[i])[0] if i in fs else 0. for i in range(1,count+1)]
    return struct.unpack('<q',stamp[1])[0],vector(fields.get(2,b''),3),vector(fields.get(3,b''),4)


def clock_line(x,y):
    x,y=np.asarray(x),np.asarray(y);center=np.median(x)
    mat=np.column_stack([x-center,np.ones(len(x))]);initial=np.linalg.lstsq(mat,y,rcond=None)[0]
    result=least_squares(lambda z:mat@z-y,initial,loss='soft_l1',f_scale=.001).x
    error=np.abs(mat@result-y)
    return result[0],result[1]-result[0]*center,float(np.percentile(error,95))


class Capture:
    def __init__(self,config):
        self.config=config;self.directory=Path(config['directory']);self.directory.mkdir(parents=True,exist_ok=True)
        self.stop=threading.Event();self.lock=threading.Lock();self.poses=deque();self.imus=deque();self.sync=deque(maxlen=180)
        self.counts={'pose':0,'imu':0};self.failures=[];self.epoch=0;self.revision=0;self.start=time.monotonic()
        self.last_pose=self.last_imu=self.last_sync=self.last_distinct_pose=0.;self.last_pose_values=None;self.health_valid=False;self.state='starting';self.result=None;self.stability=Stability();self.pending={}
        self.queue=queue.Queue(4096);self.writer=threading.Thread(target=self.write_raw);self.solver=threading.Thread(target=self.solve_loop)
        self.rate_lease=Path(config['rateLeasePath'])
        lease=read_lease(self.rate_lease,config)
        self.lease_lock=Path(lease['lockPath']).open('a')
        fcntl.flock(self.lease_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.link=None;self.original=None;self.restored=False;self.session=None;self.sub=None

    def write_raw(self):
        try:
            with (self.directory/'raw.jsonl').open('x') as out:
                while True:
                    value=self.queue.get()
                    if value is None:break
                    out.write(json.dumps(value,separators=(',',':'),allow_nan=False)+'\n')
                out.flush();os.fsync(out.fileno())
        except Exception as error:
            self.failures.append('raw recording: '+str(error));self.stop.set()

    def record(self,row):
        try:self.queue.put_nowait(row)
        except queue.Full:self.failures.append('raw recording queue full');self.stop.set()

    def publish(self,**extra):
        now=time.monotonic()
        with self.lock:
            pose_age=now-self.last_pose if self.last_pose else None
            imu_age=now-self.last_imu if self.last_imu else None
            clock_age=now-self.last_sync if self.last_sync else None
            held=bool(self.last_distinct_pose and now-self.last_distinct_pose>=.1)
            fresh=not held and pose_age is not None and pose_age<.3 and imu_age is not None and imu_age<.3 and clock_age is not None and clock_age<3
            if not fresh and self.health_valid:
                self.epoch+=1;self.result=None;self.stability=Stability()
            self.health_valid=fresh
            result=dict(self.result or {})
            if not fresh:result['converged']=False
            state=dict(schemaVersion=1,assetId=self.config['assetId'],assetName=self.config['assetName'],rigidBodyName=self.config['tracker'],
                calibrationRunId=self.config['runId'],revision=self.revision,epoch=self.epoch,
                elapsedSeconds=now-self.start,phase=self.state,counts=dict(self.counts),
                mocapLost=pose_age is None or pose_age>.3,mocapHeld=held,imuLost=imu_age is None or imu_age>.3,
                poseAgeSeconds=pose_age,imuAgeSeconds=imu_age,clockAgeSeconds=clock_age,clockLost=clock_age is None or clock_age>=3,result=result,
                failures=list(self.failures),rateRestored=self.restored,**extra)
        state['emittedMonotonicNs']=time.monotonic_ns()
        state['emittedAtUnixSeconds']=time.time()
        atomic_json(self.directory/'state.json',state,durable=False)
        print(json.dumps(state,allow_nan=False,separators=(',',':')),flush=True)

    def pose(self,sample):
        try:
            stamp,p,q=decode_pose(sample.payload.to_bytes());mono=time.monotonic()
            if not np.isfinite(p+q).all() or np.linalg.norm(q)<1e-6:raise ValueError('nonfinite or zero pose')
            row=dict(kind='pose',source_ns=stamp,mono_ns=int(mono*1e9),rx_ns=time.time_ns(),p=p,q=q)
            with self.lock:
                if self.poses and stamp==self.poses[-1]['source_ns']:
                    return
                if self.poses and stamp<self.poses[-1]['source_ns']:
                    self.epoch+=1;self.poses.clear();self.result=None;self.stability=Stability()
                if self.last_pose and mono-self.last_pose>.3:
                    self.epoch+=1;self.result=None;self.stability=Stability()
                values=tuple(p+q)
                if values!=self.last_pose_values:
                    self.last_distinct_pose=mono;self.last_pose_values=values
                self.poses.append(row);self.last_pose=mono;self.counts['pose']+=1
                while self.poses and mono-self.poses[0]['mono_ns']/1e9>92:self.poses.popleft()
            self.record(row)
        except Exception as error:self.record(dict(kind='pose_rejected',reason=str(error),mono_ns=time.monotonic_ns()))

    def solve_loop(self):
        next_solve=0
        while not self.stop.wait(.5):
            self.publish()
            now=time.monotonic()
            if now<next_solve:continue
            next_solve=now+5
            with self.lock:
                poses=list(self.poses);imus=list(self.imus);sync=list(self.sync);epoch=self.epoch
                clock_fresh=bool(self.last_sync and now-self.last_sync<3)
            if not clock_fresh:
                with self.lock:self.result=dict(converged=False,reasons=["waiting for fresh clock alignment"]);self.stability=Stability()
                continue
            if len(poses)<120 or len(imus)<120 or len(sync)<6:continue
            try:
                # All receive timestamps use this onboard monotonic clock. No
                # host wall-clock correction can appear as measured dynamics.
                origin=poses[0]['source_ns'];mono_origin=poses[0]['mono_ns']/1e9
                slope,intercept,source_error=clock_line([(p['source_ns']-origin)/1e9 for p in poses],
                    [p['mono_ns']/1e9-mono_origin for p in poses])
                rtt=np.array([r['t3']-r['t1'] for r in sync]);mask=rtt<=np.percentile(rtt,25)
                selected=[r for r,m in zip(sync,mask) if m]
                cs,ci,fc_error=clock_line([r['fc']/1e9 for r in selected],[(r['t1']+r['t3'])/2e9-mono_origin for r in selected])
                if source_error>.015 or fc_error>.005:raise InsufficientData('clock alignment is unstable')
                pt=np.array([(p['source_ns']-origin)/1e9*slope+intercept for p in poses])
                it=np.array([r['sample_us']/1e6*cs+ci for r in imus])
                result=solve(pt,[r['p'] for r in poses],[r['q'] for r in poses],it,[r['gyro'] for r in imus],[r['acc'] for r in imus])
                result['clockUncertaintyMs']=max(source_error,fc_error)*1000
                with self.lock:
                    if epoch!=self.epoch:continue
                    result=self.stability.update(result);revision=self.revision+1
                if result['converged']:
                    candidate=dict(schemaVersion=1,revision=revision,epoch=epoch,
                        assetId=self.config['assetId'],assetCommitId=self.config['assetCommitId'],agentId=self.config['agentId'],systemId=self.config['systemId'],
                        calibrationRunId=self.config['runId'],rigidBodyName=self.config['tracker'],
                        capturedAt=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),**result)
                    atomic_json(self.directory/('candidate-%d.json'%revision),candidate)
                with self.lock:
                    if epoch!=self.epoch:continue
                    self.revision=revision;self.result=result;self.state='collecting'
            except (InsufficientData,ValueError,np.linalg.LinAlgError) as error:
                with self.lock:
                    if epoch==self.epoch:
                        self.result=dict(converged=False,reasons=[str(error)]);self.stability=Stability()
            except Exception as error:
                self.failures.append('solver failed: '+str(error));self.stop.set()
                with self.lock:self.result=dict(converged=False,reasons=['solver failed'])

    def execute(self):
        import zenoh
        self.writer.start()
        try:
            self.link=Link(self.config['systemId']);self.link.heartbeat()
            hb=self.link.wait(0,seconds=6)
            if hb[6]&128:raise RuntimeError('selected aircraft is armed')
            original=self.link.interval()
            if original==0:raise RuntimeError('original IMU message interval unavailable')
            self.original=original
            activate(self.rate_lease,self.config,self.link.component,self.original)
            self.link.set_interval(10000)
            config=zenoh.Config();config.insert_json5('mode','"client"')
            config.insert_json5('connect/endpoints',json.dumps([self.config['endpoint']]))
            config.insert_json5('scouting/multicast/enabled','false');config.insert_json5('scouting/gossip/enabled','false')
            self.session=zenoh.open(config)
            key='xgc2/localization/vrpn/v1/'+self.config['source'].encode().hex()+'/'+self.config['tracker'].encode().hex()+'/'+str(self.config.get('sensor',0))+'/pose'
            self.sub=self.session.declare_subscriber(key,self.pose)
            self.state='collecting';self.solver.start();next_sync=0;last_hb=time.monotonic();last_sample=0
            self.record(dict(kind='configuration',**self.config,originalIntervalUs=self.original))
            while not self.stop.is_set():
                now=time.monotonic()
                if now>=next_sync:
                    stamp=time.monotonic_ns();self.pending[stamp]=stamp;self.link.timesync(stamp);next_sync=now+.5
                    self.pending={k:v for k,v in self.pending.items() if stamp-k<5e9}
                message=self.link.receive()
                if now-last_hb>4:raise RuntimeError('flight-controller heartbeat lost')
                if message is None:continue
                kind,payload,wall,mono=message
                if kind==0:
                    last_hb=now
                    if payload[6]&128:raise RuntimeError('aircraft armed during calibration')
                elif kind==105:
                    payload=payload.ljust(63,b'\0')
                    if struct.unpack_from('<H',payload,60)[0]&0x3f!=0x3f:continue
                    sample=struct.unpack_from('<Q',payload)[0]
                    if sample<=last_sample:
                        if sample==last_sample:continue
                        raise RuntimeError('flight-controller clock reset')
                    last_sample=sample
                    values=struct.unpack_from('<6f',payload,8)
                    if not np.isfinite(values).all():continue
                    row=dict(kind='imu',rx_ns=wall,mono_ns=mono,sample_us=sample,
                        gyro=[values[3],-values[4],-values[5]],acc=[values[0],-values[1],-values[2]])
                    with self.lock:
                        self.imus.append(row);self.last_imu=mono/1e9;self.counts['imu']+=1
                        while self.imus and mono-self.imus[0]['mono_ns']>93e9:self.imus.popleft()
                    self.record(row)
                elif kind==111:
                    tc,ts=struct.unpack_from('<qq',payload.ljust(16,b'\0'))
                    if tc and ts in self.pending:
                        row=dict(t1=self.pending.pop(ts),t3=mono,fc=tc)
                        with self.lock:self.sync.append(row);self.last_sync=mono/1e9
                        self.record(dict(kind='timesync',**row))
        except Exception as error:
            self.failures.append(str(error))
        finally:
            self.stop.set()
            if self.link is not None and self.original is not None:
                try:
                    self.link.set_interval(self.original);mark_restored(self.rate_lease);self.restored=True
                except Exception as error:self.failures.append('restore failed: '+str(error))
            if self.link is not None:
                try:self.link.close()
                except Exception as error:self.failures.append('MAVLink cleanup: '+str(error))
            if self.original is None:
                try:mark_restored(self.rate_lease);self.restored=True
                except Exception as error:self.failures.append('recovery receipt failed: '+str(error))
            if self.solver.is_alive():self.solver.join()
            for label,action in [('subscriber',self.sub.undeclare if self.sub is not None else None),
                                 ('session',self.session.close if self.session is not None else None)]:
                if action is not None:
                    try:action()
                    except Exception as error:self.failures.append(label+' cleanup: '+str(error))
            self.state='stopped';self.record(dict(kind='end',rateRestored=self.restored,failures=self.failures))
            try:self.queue.put(None,timeout=2)
            except queue.Full:self.failures.append('writer did not drain')
            self.writer.join(timeout=5)
            atomic_json(self.directory/'cleanup.json',dict(rateRestored=self.restored,failures=self.failures))
            self.publish();self.lease_lock.close()
        return 1 if self.failures else 0


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True,type=Path)
    args=parser.parse_args();capture=Capture(json.loads(args.config.read_text()))
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:capture.stop.set())
    return capture.execute()


if __name__=='__main__':raise SystemExit(main())
