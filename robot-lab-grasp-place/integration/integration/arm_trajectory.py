"""Validated timed joint path interpolation for simulation I/O only."""
import math
import numpy as np

NAMES = ['joint'+str(i) for i in range(1,7)]


class JointPath:
    def __init__(self, trajectory, measured, limits, max_duration=25.):
        names=list(trajectory.joint_names)
        if any(names.count(name)!=1 for name in NAMES):raise ValueError('Six unique arm joints required')
        indices=[names.index(name) for name in NAMES]
        self.times=[];self.positions=[]
        for point in trajectory.points:
            if len(point.positions)!=len(names):raise ValueError('Incomplete trajectory point')
            q=np.asarray([point.positions[i] for i in indices],float)
            t=point.time_from_start.sec+point.time_from_start.nanosec/1e9
            if not np.isfinite(q).all() or not math.isfinite(t) or t<0:raise ValueError('Nonfinite path')
            if self.times and t<=self.times[-1]:raise ValueError('Trajectory must be timed and increasing')
            # Measured MuJoCo joints can overshoot a hard boundary by a few
            # microradians; MoveIt keeps that measured start in its first point.
            if np.any(q<limits[:,0]-1e-4) or np.any(q>limits[:,1]+1e-4):raise ValueError('Trajectory exceeds joint limits')
            # Project only floating-point boundary noise already accepted above.
            q=np.clip(q,limits[:,0],limits[:,1])
            self.times.append(t);self.positions.append(q)
        if len(self.times)<2 or not 0<self.times[-1]<=max_duration:raise ValueError('Empty, untimed or overlong trajectory')
        if np.max(np.abs(self.positions[0]-measured))>.02:raise ValueError('Trajectory start differs from measured joints')
        self.times=np.asarray(self.times);self.positions=np.asarray(self.positions)
        # Preserve the checked joint-space line segments. Retiming may use the
        # exported simulation joint limit; do not impose a second, lower cap.
        velocity=np.abs(np.diff(self.positions,axis=0))/np.diff(self.times)[:,None]
        maximum=np.full(6, 1.2)
        factor=max(1.,float(np.max(velocity/maximum)))
        self.times*=factor
        if self.times[-1]>max_duration:raise ValueError('Scaled trajectory exceeds motion budget')

    def sample(self, elapsed):
        return [float(np.interp(elapsed,self.times,self.positions[:,i])) for i in range(6)]

    @property
    def duration(self):return float(self.times[-1])
