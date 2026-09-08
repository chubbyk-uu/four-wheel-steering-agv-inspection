"""Local encoder-only pose for scan trials; not GNSS/IMU fusion or slip-free truth."""
import math
import numpy as np


class WheelOdometry:
    def __init__(self,radius,wheelbase,track):
        if not np.isfinite([radius,wheelbase,track]).all() or min(radius,wheelbase,track)<=0:
            raise ValueError('invalid wheel geometry')
        self.radius=radius
        self.matrix=np.array([[1,0,-y] if axis==0 else [0,1,x]
            for x,y in ((wheelbase/2,track/2),(wheelbase/2,-track/2),(-wheelbase/2,track/2),(-wheelbase/2,-track/2))
            for axis in (0,1)],float)
        self.inverse=np.linalg.pinv(self.matrix)
        self.pose=np.zeros(3);self.previous=None;self.residual_speed=0.;self.max_residual_speed=0.

    def update(self,time,drive,steer):
        drive=np.asarray(drive,float);steer=np.asarray(steer,float)
        if drive.shape!=(4,) or steer.shape!=(4,) or not np.isfinite(np.r_[time,drive,steer]).all():
            raise ValueError('invalid encoder sample')
        previous=self.previous
        if previous is not None:
            dt=time-previous[0]
            if dt<=0:return
            if dt>.2:raise ValueError('encoder feedback gap exceeds 200 ms')
            displacement=(drive-previous[1])*self.radius
            angle=(steer+previous[2])/2
            vectors=np.column_stack((displacement*np.cos(angle),displacement*np.sin(angle))).ravel()
            delta=self.inverse@vectors
            self.residual_speed=float(np.max(np.linalg.norm((self.matrix@delta-vectors).reshape(4,2),axis=1))/dt)
            self.max_residual_speed=max(self.max_residual_speed,self.residual_speed)
            # Integrate small body displacement at the midpoint orientation.
            yaw=self.pose[2]+delta[2]/2;c,s=math.cos(yaw),math.sin(yaw)
            self.pose+=np.array([c*delta[0]-s*delta[1],s*delta[0]+c*delta[1],delta[2]])
        self.previous=(time,drive.copy(),steer.copy())
