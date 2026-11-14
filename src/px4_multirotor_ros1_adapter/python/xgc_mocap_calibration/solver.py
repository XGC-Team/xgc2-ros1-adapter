"""Sequential delay, rotation and integrated lever estimation in a moving window.

Input time is a common receive clock in seconds; gyro/acceleration are body FLU.
The returned transform is T_marker_body, never a world offset or PX4 delay.
"""
from dataclasses import dataclass
import numpy as np
from scipy.ndimage import gaussian_filter1d, convolve1d
from scipy.optimize import least_squares, minimize_scalar
from scipy.spatial.transform import Rotation, Slerp


class InsufficientData(ValueError):
    pass


def interpolate(t, source_t, values):
    return np.column_stack([np.interp(t, source_t, values[:, k]) for k in range(values.shape[1])])


def robust_rotation(a, b):
    weights = np.ones(len(a))
    for _ in range(6):
        rotation, _ = Rotation.align_vectors(a, b, weights=weights)
        residual = np.linalg.norm(a - rotation.apply(b), axis=1)
        scale = max(.025, 1.4826 * np.median(np.abs(residual - np.median(residual))))
        weights = np.minimum(1., 1.5 * scale / np.maximum(residual, 1e-9))
    return rotation, residual, weights


def solve(pose_t, position, quaternion, imu_t, gyro, acceleration):
    """Solve one immutable window, retaining gaps as exclusions, never filling holds."""
    t, p, q, it, g, f = [np.asarray(x, dtype=float) for x in
                         (pose_t, position, quaternion, imu_t, gyro, acceleration)]
    if len(t) < 120 or len(it) < 120 or min(t[-1], it[-1]) - max(t[0], it[0]) < 8:
        raise InsufficientData('collecting: need at least eight seconds of overlapping motion')
    if not all(np.isfinite(x).all() for x in (t,p,q,it,g,f)):
        raise InsufficientData('nonfinite input')
    dt, idt = np.diff(t), np.diff(it)
    if np.any(dt <= 0) or np.any(idt <= 0):
        raise InsufficientData('clock reset or duplicate timestamps')
    r = Rotation.from_quat(q)
    wt = (t[:-1] + t[1:]) / 2
    w = (r[:-1].inv() * r[1:]).as_rotvec() / dt[:,None]
    speed = np.linalg.norm(np.diff(p,axis=0),axis=1)/dt
    invalid = (dt > .12) | (dt < .002) | (speed > 8) | (np.linalg.norm(w,axis=1)>12)
    same = (np.linalg.norm(np.diff(p,axis=0),axis=1)<1e-12) & (np.linalg.norm(w,axis=1)<1e-10)
    edges = np.diff(np.r_[False,same,False].astype(int))
    holds = [(t[a],t[b]) for a,b in zip(np.flatnonzero(edges==1), np.flatnonzero(edges==-1)) if t[b]-t[a]>=.1]
    pose_bad = [(t[j],t[j+1]) for j in np.flatnonzero(invalid)] + holds
    imu_bad = [(it[j],it[j+1]) for j in np.flatnonzero(idt>.12)]

    def available(times, pad, intervals):
        keep = np.ones(len(times),dtype=bool)
        for a,b in intervals: keep &= (times<a-pad) | (times>b+pad)
        return keep

    start, end = max(t[0],it[0])+.4, min(t[-1],it[-1])-.4
    grid = np.arange(start,end,.005)
    keep = available(grid,.06,pose_bad)
    if keep.sum() < 400:
        raise InsufficientData('too few continuous, fresh motion samples')
    aw = gaussian_filter1d(np.interp(grid,wt,np.linalg.norm(w,axis=1)),2)
    if np.std(aw[keep])<.08:
        raise InsufficientData('rotate with varying angular speed to identify delay')
    def score(lag):
        bw = gaussian_filter1d(np.interp(grid-lag,it,np.linalg.norm(g,axis=1)),2)
        paired=keep & available(grid-lag,.06,imu_bad)
        if paired.sum()<400 or np.std(bw[paired])<.05: return -1.
        return float(np.corrcoef(aw[paired],bw[paired])[0,1])
    lags = np.arange(-.1,.201,.002)
    scores = np.array([score(lag) for lag in lags])
    best = int(np.argmax(scores))
    if best in (0,len(lags)-1):
        raise InsufficientData('delay estimate reached search boundary')
    lag = float(minimize_scalar(lambda x:-score(x),bounds=(lags[best]-.002,lags[best]+.002),method='bounded').x)
    correlation = score(lag)
    motion = (~invalid) & available(wt,.03,pose_bad) & available(wt-lag,.03,imu_bad) & (wt>start) & (wt<end) & (np.linalg.norm(w,axis=1)>.15)
    if motion.sum()<100: raise InsufficientData('insufficient rotational excitation')
    a, b = w[motion], interpolate(wt[motion]-lag,it,g)
    rkb, residual, weights = robust_rotation(a,b)
    excitation = np.linalg.eigvalsh((b*weights[:,None]).T@b/weights.sum())
    split_rotation = []
    for part in np.array_split(np.arange(len(a)),3):
        rr,_,_ = robust_rotation(a[part],b[part])
        split_rotation.append(float(np.degrees((rkb.inv()*rr).magnitude())))

    # Integrated identity over +/-150 ms; no numerical second derivative of
    # measured position or angular acceleration is used.
    physical = t-lag
    step, half = .005, 30
    h = half*step
    tg = np.arange(max(physical[0],it[0])+.05,min(physical[-1],it[-1])-.05,step)
    rb = Slerp(physical,r)(tg).as_matrix() @ rkb.as_matrix()
    pp, ff = interpolate(tg,physical,p), interpolate(tg,it,f)
    kernel = step**2*(half-np.abs(np.arange(-half,half+1)))
    rfint = convolve1d(np.einsum('nij,nj->ni',rb,ff),kernel,axis=0,mode='nearest')
    rint = convolve1d(rb,kernel,axis=0,mode='nearest')
    ids = np.arange(half,len(tg)-half,5)
    ids = ids[available(tg[ids]+lag,h+.05,pose_bad) & available(tg[ids],h+.05,imu_bad)]
    if len(ids)<100: raise InsufficientData('not enough continuous data for lever arm')
    d2r = (rb[ids+half]-2*rb[ids]+rb[ids-half])/h**2
    yy = (pp[ids+half]-2*pp[ids]+pp[ids-half]-rfint[ids])/h**2
    matrix = np.concatenate([d2r,-rint[ids]/h**2,np.broadcast_to(np.eye(3),(len(ids),3,3))],axis=2)
    mat, y = matrix.reshape(-1,9), yy.ravel()
    def fit(mm, zz):
        x = np.linalg.lstsq(mm,zz,rcond=None)[0]
        return least_squares(lambda value:mm@value-zz,x,loss='soft_l1',f_scale=.3,max_nfev=40).x
    x = fit(mat,y)
    nuisance = mat[:,3:]
    lever = mat[:,:3]-nuisance@np.linalg.lstsq(nuisance,mat[:,:3],rcond=None)[0]
    lever_information = np.linalg.svd(lever,compute_uv=False)**2/len(ids)
    split_levers = [fit(mm.reshape(-1,9),zz.ravel())[:3]
                    for mm,zz in zip(np.array_split(matrix,2),np.array_split(yy,2))]
    split_translation = float(np.linalg.norm(split_levers[0]-split_levers[1]))
    trans = -rkb.apply(x[:3])
    rms = float(np.sqrt(np.mean((y-mat@x)**2)*3))
    normalized_rotation_residual=float(np.median(residual)/max(.1,np.median(np.linalg.norm(a,axis=1))))
    rotation_inliers=float(np.mean(residual<np.maximum(.15,.2*np.linalg.norm(a,axis=1))))
    reasons=[]
    if normalized_rotation_residual>.15 or rotation_inliers<.6: reasons.append('gyro and mocap angular motion disagree')
    if correlation<.85: reasons.append('delay correlation is weak')
    if excitation[0]<.015: reasons.append('rotate around more than one axis')
    if max(split_rotation)>2: reasons.append('rotation differs between subwindows')
    if lever_information[-1]<.05: reasons.append('lever arm is not observable on every axis')
    if split_translation>.03: reasons.append('translation differs between subwindows')
    if rms>1.0: reasons.append('acceleration residual is high')
    if np.linalg.norm(trans)>1: reasons.append('implausible mounting translation')
    return dict(translation=trans.tolist(),rotation=rkb.as_quat().tolist(),identifiedDelayMs=lag*1000,
                delayCorrelation=correlation,normalizedRotationResidual=normalized_rotation_residual,rotationInlierFraction=rotation_inliers,rotationResidualRadS=float(np.sqrt(np.mean(residual**2))),
                accelerationResidualMps2=rms,rotationExcitation=excitation.tolist(),
                leverInformation=lever_information.tolist(),splitRotationDeg=max(split_rotation),
                splitTranslationM=split_translation,excludedIntervals=len(pose_bad)+len(imu_bad),
                usableSamples=int(len(ids)),windowSeconds=float(end-start),qualified=not reasons,reasons=reasons)


@dataclass
class Stability:
    """Three separated estimates must agree, beyond same-window solver success."""
    previous: dict = None
    stable: int = 0

    def update(self, result):
        if self.previous is None:
            self.stable=0
        else:
            old=self.previous
            angle=np.degrees((Rotation.from_quat(old['rotation']).inv()*Rotation.from_quat(result['rotation'])).magnitude())
            distance=np.linalg.norm(np.array(old['translation'])-result['translation'])
            self.stable=self.stable+1 if (result['qualified'] and old['qualified'] and angle<.5 and distance<.01 and abs(old['identifiedDelayMs']-result['identifiedDelayMs'])<3) else 0
        self.previous=result
        result['stableWindows']=self.stable
        result['converged']=bool(result['qualified'] and self.stable>=3)
        if result['qualified'] and not result['converged']:
            result['reasons']=['waiting for stable consecutive windows']
        return result
