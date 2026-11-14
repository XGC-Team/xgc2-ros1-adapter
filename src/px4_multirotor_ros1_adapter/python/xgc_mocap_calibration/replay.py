"""Replay archived field samples through the same online estimator."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from .solver import solve, Stability, InsufficientData


def affine(x,y):
    x,y=np.asarray(x),np.asarray(y)
    center=np.median(x);a=np.column_stack([x-center,np.ones(len(x))])
    z=np.linalg.lstsq(a,y,rcond=None)[0]
    z=least_squares(lambda z:a@z-y,z,loss='soft_l1',f_scale=.002).x
    return z[0],z[1]-z[0]*center


def archived(path):
    rows=[json.loads(x) for x in (path/'onboard.jsonl').read_text().splitlines()]
    low=[r for r in rows if r['kind']=='pose'];imu=[r for r in rows if r['kind']=='imu']
    sync=[r for r in rows if r['kind']=='timesync']
    raw=[json.loads(x) for x in (path/'core-raw-pose.jsonl').read_text().splitlines()]
    pose=[r for r in raw if r['kind']=='pose']
    base=low[0]['rx_ns'];origin=pose[0]['source_ns']
    slope,intercept=affine([(r['source_ns']-origin)/1e9 for r in low],[(r['rx_ns']-base)/1e9 for r in low])
    rtt=np.array([(r['t3_ns']-r['t1_ns'])/1e9 for r in sync]);use=rtt<=np.percentile(rtt,25)
    cs,ci=affine(np.array([r['t2_ns']/1e9 for r in sync])[use],np.array([((r['t1_ns']-base)+(r['t3_ns']-base))/2e9 for r in sync])[use])
    # Archived HIGHRES_IMU is FRD; the asset contract and live ROS IMU are FLU.
    flip=np.array([1.,-1.,-1.])
    return (np.array([(r['source_ns']-origin)/1e9*slope+intercept for r in pose]),
            np.array([r['p'] for r in pose]),np.array([r['q'] for r in pose]),
            np.array([r['sample_us']/1e6*cs+ci for r in imu]),
            np.array([r['gyro'] for r in imu])*flip,np.array([r['acc'] for r in imu])*flip)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path)
    parser.add_argument('--window',type=float,default=90);parser.add_argument('--step',type=float,default=10)
    args=parser.parse_args();t,p,q,it,g,f=archived(args.archive); stability=Stability()
    for end in np.arange(max(t[0],it[0])+15,min(t[-1],it[-1]),args.step):
        pm=(t>=end-args.window)&(t<=end);im=(it>=end-args.window-.5)&(it<=end+.5)
        try:result=stability.update(solve(t[pm],p[pm],q[pm],it[im],g[im],f[im]))
        except InsufficientData as error:result=dict(converged=False,reasons=[str(error)])
        print(json.dumps(dict(endSeconds=float(end),**result),allow_nan=False),flush=True)


if __name__=='__main__':main()
