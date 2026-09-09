"""Time-parametrized single-segment SE(3) tracking on a local surface tangent.

No independent vertical velocity command. The underlying swerve controller owns
wheel branch choice, steering limits, braking and alignment.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


@dataclass
class Profile:
    distance: float
    speed: float
    accel: float
    decel: float

    def __post_init__(self):
        if not np.isfinite([self.distance,self.speed,self.accel,self.decel]).all() or self.distance<0 or min(self.speed,self.accel,self.decel)<=0:
            raise ValueError('invalid motion profile')
        self.peak=min(self.speed,math.sqrt(2*self.distance/(1/self.accel+1/self.decel)))
        self.ta=self.peak/self.accel;self.td=self.peak/self.decel
        self.da=self.peak*self.ta/2;self.dd=self.peak*self.td/2
        self.tc=max(0.,(self.distance-self.da-self.dd)/self.peak) if self.peak else 0.
        self.duration=self.ta+self.tc+self.td

    def sample(self,t):
        t=max(0.,t)
        if t>=self.duration:return self.distance,0.
        if t<self.ta:return self.accel*t*t/2,self.accel*t
        if t<self.ta+self.tc:return self.da+self.peak*(t-self.ta),self.peak
        remaining=self.duration-t
        return self.distance-self.decel*remaining*remaining/2,self.decel*remaining


class SegmentTracker:
    def __init__(self,position,quaternion,kind,displacement,angle,speed,platform,config):
        self.start=np.asarray(position,float);self.rotation=Rotation.from_quat(quaternion)
        displacement=np.asarray(displacement,float)
        if self.start.shape!=(3,) or displacement.shape!=(2,) or not np.isfinite(np.r_[self.start,displacement,angle,speed]).all():
            raise ValueError('invalid segment geometry')
        if kind not in ('translate','rotate') or not 0<speed<=platform['max_speed']:
            raise ValueError('invalid segment type or speed')
        for key,value in config.items():
            if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('invalid tracking parameter '+key)
        if not math.isfinite(platform['max_lateral_speed']) or platform['max_lateral_speed']<=0:
            raise ValueError('invalid lateral speed limit')
        self.kind=kind;self.cfg=config;self.platform=platform;self.forward_only=False
        self.length=float(np.linalg.norm(displacement)) if kind=='translate' else abs(float(angle))
        if self.length<1e-4 or (kind=='rotate' and abs(angle)>math.pi+1e-8):
            raise ValueError('segment must be nonzero; rotations limited to +/- pi')
        self.sign=1 if angle>=0 else -1
        self.axis=displacement/self.length if kind=='translate' else np.zeros(2)
        self.goal=self.start+self.rotation.apply([*displacement,0]) if kind=='translate' else self.start.copy()
        self.goal_rotation=self.rotation if kind=='translate' else self.rotation*Rotation.from_euler('z',angle)
        self.final_goal=self.goal.copy();self.final_rotation=self.goal_rotation;self.final_tangent=self.rotation
        self.terminal_filtered=None;self.outside_time=0.;self.trims=0;self.alignment_probe=None
        self.speed=speed if kind=='translate' else min(speed,config['angular_speed_rad_s'],platform['max_yaw_rate'])
        self.accel=platform['drive_accel'] if kind=='translate' else config['angular_accel_rad_s2']
        self.decel=platform['drive_decel'] if kind=='translate' else config['angular_decel_rad_s2']
        self.profile=Profile(self.length,self.speed,self.accel,self.decel)
        self.clock=0.;self.offset=0.;self.elapsed=0.;self.settled=0.
        self.state='ALIGNING';self.reason='';self.filtered=np.zeros(3);self.command=np.zeros(3)
        self.reconfigurations=0;self.was_running=False;self.last_angle=0.;self.unwrapped_angle=0.
        self.reference=self.start.copy();self.reference_rotation=self.rotation
        self.diagnostic={}

    def fault(self,reason):
        if self.state in ('FAULT','COMPLETED'):return
        self.state='FAULT';self.reason=reason;self.command=np.zeros(3)

    def start_trim(self,p,r):
        # Stop before selecting a new correction axis. Every trim gets its own
        # rest-to-rest time profile, avoiding sub-deadband requests stuck in HOLD.
        self.outside_time=0.
        # Filtered error only requests reevaluation; choose the correction from
        # the current pose, including when the filtered position error is stale.
        position_error=self.final_tangent.inv().apply(self.final_goal-p)[:2]
        heading_error=self.yaw_error(self.final_rotation,r)
        if np.linalg.norm(position_error)<=.8*self.cfg['position_tolerance_m'] and abs(heading_error)<=.8*self.cfg['heading_tolerance_rad']:
            return
        if self.trims>=self.cfg['max_terminal_trims']:
            self.fault('TERMINAL_NOT_CONVERGED');return
        self.trims+=1;self.start=p.copy();self.rotation=r
        position_ratio=np.linalg.norm(position_error)/self.cfg['position_tolerance_m']
        heading_ratio=abs(heading_error)/self.cfg['heading_tolerance_rad']
        if position_ratio>max(.8,heading_ratio):
            delta=r.inv().apply(self.final_goal-p)[:2]
            if self.forward_only and delta[0]<-self.cfg['position_deadband_m']:
                self.fault('FORWARD_ONLY_TERMINAL_OVERSHOOT');return
            self.length=float(np.linalg.norm(delta))
            if not math.isfinite(self.length) or self.length<1e-8:
                self.fault('DEGENERATE_TERMINAL_TRANSLATION');return
            self.kind='translate';self.axis=delta/self.length;self.goal=p+r.apply([*delta,0.]);self.goal_rotation=r
            self.speed=self.cfg['terminal_translation_speed_m_s'];self.accel=self.platform['drive_accel'];self.decel=self.platform['drive_decel']
        else:
            self.kind='rotate';self.length=abs(heading_error);self.sign=1 if heading_error>=0 else -1
            self.axis=np.zeros(2);self.goal=p.copy();self.goal_rotation=self.final_rotation
            self.speed=self.cfg['terminal_rotation_speed_rad_s'];self.accel=self.cfg['angular_accel_rad_s2'];self.decel=self.cfg['angular_decel_rad_s2']
        self.profile=Profile(self.length,self.speed,self.accel,self.decel)
        self.clock=0.;self.offset=0.;self.was_running=False;self.filtered[:]=0.
        self.last_angle=0.;self.unwrapped_angle=0.;self.terminal_filtered=None;self.alignment_probe=None;self.state='ALIGNING'

    @staticmethod
    def yaw_error(desired,actual):
        matrix=(actual.inv()*desired).as_matrix()
        return math.atan2(matrix[1,0],matrix[0,0])

    def update(self,position,quaternion,velocity,mode,dt,healthy=True):
        if not 0<dt<=.1:
            self.fault('INVALID_CONTROL_TIMESTEP');return self.command.copy()
        p=np.asarray(position,float);r=Rotation.from_quat(quaternion);vel=np.asarray(velocity,float)
        if not np.isfinite(np.r_[p,vel]).all():
            self.fault('INVALID_FEEDBACK');return self.command.copy()
        if not healthy:self.fault('LOCALIZATION_NOT_READY')
        if self.state in ('FAULT','COMPLETED'):
            self.command[:]=0;return self.command.copy()
        self.elapsed+=dt
        if self.elapsed>self.cfg['segment_timeout_s']:
            self.fault('SEGMENT_TIMEOUT');return self.command.copy()
        tangent_error=self.final_tangent.inv().apply(self.final_goal-p)[:2]
        heading_error=self.yaw_error(self.final_rotation,r)
        relative=self.rotation.inv()*r;matrix=relative.as_matrix();angle=math.atan2(matrix[1,0],matrix[0,0])
        self.unwrapped_angle+=wrap(angle-self.last_angle);self.last_angle=angle
        at_end=self.offset+self.profile.sample(self.clock)[0]>=self.length-1e-8
        if (self.was_running and at_end) or self.state=='STOPPING':
            self.state='STOPPING';self.command[:]=0
            raw_terminal=np.r_[tangent_error,heading_error]
            if self.terminal_filtered is None:self.terminal_filtered=raw_terminal.copy()
            self.terminal_filtered+=dt/(self.cfg['feedback_filter_s']+dt)*(raw_terminal-self.terminal_filtered)
            near=(np.linalg.norm(self.terminal_filtered[:2])<=self.cfg['position_tolerance_m']
                  and abs(self.terminal_filtered[2])<=self.cfg['heading_tolerance_rad']
                  and np.linalg.norm(tangent_error)<=2*self.cfg['position_tolerance_m']
                  and abs(heading_error)<=2*self.cfg['heading_tolerance_rad'])
            stopped=mode=='HOLD' and np.linalg.norm(vel[:2])<self.cfg['stopped_speed_m_s'] and abs(vel[2])<self.cfg['stopped_yaw_rate_rad_s']
            if stopped and near:
                self.settled+=dt;self.outside_time=0.
                if self.settled>=self.cfg['settle_time_s']:self.state='COMPLETED'
            else:
                self.settled=0.
                self.outside_time=self.outside_time+dt if stopped else 0.
                if self.outside_time>=self.cfg['terminal_dwell_s']:
                    self.start_trim(p,r)
            self.diagnostic={'goal_error_m':float(np.linalg.norm(tangent_error)),
                             'goal_heading_error_rad':heading_error,
                             'filtered_terminal_error_m':float(np.linalg.norm(self.terminal_filtered[:2])) if self.terminal_filtered is not None else None}
            return self.command.copy()
        if mode!='DRIVE':
            if self.state=='RUNNING':
                self.state='ALIGNING';self.reconfigurations+=1
            if not self.was_running and self.alignment_probe is None:
                if self.kind=='translate':
                    local=np.r_[self.axis*self.cfg['alignment_linear_probe_m_s'],0.]
                    body=r.inv().apply(self.rotation.apply(local));self.command=np.r_[body[:2],0.]
                else:self.command=np.array([0.,0.,self.sign*self.cfg['alignment_angular_probe_rad_s']])
                self.alignment_probe=self.command.copy()
            # Keep the last nonzero request stable while wheel angles settle.
            return self.command.copy()
        if self.state=='ALIGNING':
            if self.was_running:
                progress=float(np.dot(self.rotation.inv().apply(p-self.start)[:2],self.axis)) if self.kind=='translate' else self.sign*self.unwrapped_angle
                self.offset=float(np.clip(progress,0,self.length))
                self.profile=Profile(self.length-self.offset,self.speed,self.accel,self.decel)
            self.clock=0.;self.filtered[:]=0.;self.state='RUNNING';self.was_running=True
        self.clock+=dt
        distance,feedforward=self.profile.sample(self.clock);progress=self.offset+distance
        self.reference=self.start+self.rotation.apply([*(self.axis*progress),0.]) if self.kind=='translate' else self.start.copy()
        self.reference_rotation=self.rotation if self.kind=='translate' else self.rotation*Rotation.from_euler('z',self.sign*progress)
        e=self.rotation.inv().apply(self.reference-p)[:2]
        eyaw=self.yaw_error(self.reference_rotation,r)
        if np.linalg.norm(e)>self.cfg['max_tracking_error_m'] or abs(eyaw)>self.cfg['max_heading_error_rad']:
            self.fault('TRACKING_ERROR_LIMIT');return self.command.copy()
        raw=np.r_[e,eyaw];alpha=dt/(self.cfg['feedback_filter_s']+dt)
        self.filtered+=alpha*(raw-self.filtered)
        feedback=self.filtered.copy()
        for i,deadband in enumerate([self.cfg['position_deadband_m']]*2+[self.cfg['heading_deadband_rad']]):
            feedback[i]=math.copysign(max(0,abs(feedback[i])-deadband),feedback[i])
        linear=feedback[:2]*self.cfg['position_gain']
        norm=np.linalg.norm(linear)
        if norm>self.cfg['max_position_feedback_m_s']:linear*=self.cfg['max_position_feedback_m_s']/norm
        angular=float(np.clip(feedback[2]*self.cfg['heading_gain'],-self.cfg['max_heading_feedback_rad_s'],self.cfg['max_heading_feedback_rad_s']))
        if self.kind=='translate':
            forward=feedforward+float(np.dot(linear,self.axis))
            if self.forward_only:forward=max(0.,forward)
            cross=linear-self.axis*np.dot(linear,self.axis)
            bound=self.cfg['cross_command_ratio']*abs(forward)
            if np.linalg.norm(cross)>bound:cross*=bound/np.linalg.norm(cross)
            linear=self.axis*forward+cross
            # Avoid a tiny yaw correction dominating wheel direction near zero speed.
            angular=float(np.clip(angular,-bound,bound))
            if not at_end and np.linalg.norm(linear)<self.cfg['alignment_linear_probe_m_s']:
                linear=self.axis*self.cfg['alignment_linear_probe_m_s']
        else:
            angular=self.sign*feedforward+angular
            if not at_end and abs(angular)<self.cfg['alignment_angular_probe_rad_s']:
                angular=self.sign*self.cfg['alignment_angular_probe_rad_s']
            bound=self.cfg['cross_command_ratio']*abs(angular)
            if np.linalg.norm(linear)>bound:linear*=bound/np.linalg.norm(linear)
        body=r.inv().apply(self.rotation.apply([*linear,0.]))
        command=np.r_[body[:2],angular]
        # Coupled limits agree with the bottom-level allocator; final wheel limits remain there.
        scale=max(1.,np.linalg.norm(command[:2])/self.platform['max_speed'],abs(command[2])/self.platform['max_yaw_rate'],abs(command[1])/self.platform['max_lateral_speed'])
        self.command=command/scale
        self.diagnostic={'reference_speed':feedforward,'active_kind':self.kind,'along_reference_error_m':float(np.dot(e,self.axis)),'reference_position_error_m':float(np.linalg.norm(e)),
                         'reference_heading_error_rad':eyaw,'goal_error_m':float(np.linalg.norm(tangent_error)),
                         'goal_heading_error_rad':heading_error,'reference_progress':progress}
        return self.command.copy()
