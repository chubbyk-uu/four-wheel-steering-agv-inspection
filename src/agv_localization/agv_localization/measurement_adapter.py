"""Production measurement adapter: joints, raw six-axis IMU, FIX antenna positions.

No ground-truth subscription, command integration or synthetic absolute IMU yaw.
"""
from collections import deque
import json
import math
import hashlib
import tempfile
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from .common import load
from .core import (EncoderOdometry, DeliveryQueue, delay_sample, stamp_seconds,
                   calibrated_antennas, dual_gnss_pose, gravity_tilt, body_acceleration, wrap)


class MeasurementAdapter(Node):
    def __init__(self):
        super().__init__('measurement_adapter')
        self.config,self.platform,self.camera=load(self)
        self.encoder=EncoderOdometry(self.platform,self.config)
        self.mounts=calibrated_antennas(self.platform,self.config['calibration'])
        self.rotation=Rotation.from_rotvec(self.config['calibration']['imu_rotvec_rad']).as_matrix()
        self.rng=np.random.default_rng(self.config['seed']+20)
        self.timings={k:np.random.default_rng(self.config['seed']+i) for i,k in enumerate(('wheel','imu'),30)}
        noise=self.config['noise_enabled']
        self.gyro_bias=self.rng.normal(0,self.config['gyro_bias_sigma_rad_s'] if noise else 0,3)
        self.accel_bias=self.rng.normal(0,self.config['accel_bias_sigma_m_s2'] if noise else 0,3)
        self.queue=DeliveryQueue()
        self.local=deque(maxlen=250);self.pairs={}
        self.stationary=deque(maxlen=self.config['stationary_tilt_samples'])
        self.last_tilt=-math.inf;self.tilt_ready=False
        # Measured body twist history feeds the kinematic acceleration removed
        # from the accelerometer; never a command or ground-truth proxy.
        self.body_twist=deque(maxlen=400);self.gravity=deque(maxlen=200)
        self.last_motion_tilt=-math.inf;self.motion_tilts=0;self.motion_rejects=0
        self.wheel_speed=math.inf;self.gyro_speed=math.inf
        self.last_raw_imu=-math.inf
        self.ages={'wheel':-math.inf,'imu':-math.inf,'gnss':-math.inf}
        self.pubs={'wheel':self.create_publisher(Odometry,'/localization/wheel_odom',20),
                   'imu':self.create_publisher(Imu,'/localization/imu',20),
                   'tilt':self.create_publisher(Imu,'/localization/tilt',10),
                   'gnss':self.create_publisher(PoseWithCovarianceStamped,'/localization/gnss_pose',20)}
        self.contact_pub=self.create_publisher(TwistWithCovarianceStamped,'/localization/contact_velocity',20)
        self.status=self.create_publisher(String,'/localization/status',10)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        self.create_subscription(Imu,'/sensors/imu/raw',self.imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odometry/local',self.local_pose,20)
        for side in ('left','right'):
            self.create_subscription(PoseWithCovarianceStamped,'/sensors/gnss/fixed/'+side,
                                     lambda m,side=side:self.gnss(m,side),20)
        self.create_timer(.002,self.deliver)
        self.create_timer(.05,self.health)
        directory=self.declare_parameter('output_dir','').value or tempfile.mkdtemp(prefix='agv_navigation_')
        self.output=Path(directory);self.output.mkdir(parents=True,exist_ok=True)
        # Exclusive creation prevents a second run from overwriting previous navigation.
        self.archive=(self.output/'navigation.jsonl').open('x')
        self.last_archive_stamp=-math.inf
        self.create_subscription(Odometry,'/odometry/global',self.record_navigation,100)
        self.static=StaticTransformBroadcaster(self)
        self.publish_calibration()

    def publish_calibration(self):
        """Distinct calibrated frames; physical URDF frames remain display geometry."""
        c=self.config['calibration'];out=[]
        def add(name,translation,rotation):
            msg=TransformStamped();msg.header.frame_id='base_link';msg.child_frame_id=name
            msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=map(float,translation)
            q=rotation.as_quat()
            msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=q.tolist()
            out.append(msg)
        for side,point in zip(('left','right'),self.mounts):
            add('gnss_'+side+'_calibrated',point,Rotation.identity())
        cam=self.camera
        height=cam['nominal_width_m']*cam['focal_length_m']/(cam['width']*cam['pixel_pitch_m'])
        # Right perturbation at optical-frame origin, expressed in optical axes.
        optical=Rotation.from_euler('xyz',[math.pi,0,math.pi/2])
        add('camera_optical_calibrated',np.array([cam['camera_x_m'],0,height-cam['base_nominal_height_m']])+
            optical.apply(c['camera_translation_m']),optical*Rotation.from_rotvec(c['camera_rotvec_rad']))
        for side,sign in [('left',1),('right',-1)]:
            rot=Rotation.from_euler('x',math.pi)
            add('lidar_'+side+'_calibrated',np.array([.79,sign*.585,.015])+rot.apply(c['lidar_'+side+'_translation_m']),
                rot*Rotation.from_rotvec(c['lidar_'+side+'_rotvec_rad']))
        add('imu_calibrated',c['imu_translation_m'],Rotation.from_matrix(self.rotation))
        records=[{'frame_id':m.child_frame_id,'translation_m':[m.transform.translation.x,m.transform.translation.y,m.transform.translation.z],
                  'orientation_xyzw':[m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w]} for m in out]
        wheel_calibration={'radii_m':self.encoder.radius.tolist(),
            'positions_xy_m':[[-float(self.encoder.matrix[i*2,2]),float(self.encoder.matrix[i*2+1,2])] for i in range(4)],
            'steer_zero_error_rad':self.encoder.zero.tolist()}
        # Matrix stores -y and x; persist conventional [x,y] positions.
        wheel_calibration['positions_xy_m']=[[xy[1],xy[0]] for xy in wheel_calibration['positions_xy_m']]
        self.calibration_id='agv-cal-'+hashlib.sha256(json.dumps([records,wheel_calibration],sort_keys=True).encode()).hexdigest()[:16]
        (self.output/'calibration.json').write_text(json.dumps({'calibration_id':self.calibration_id,
            'wheel_calibration':wheel_calibration,'optical_intrinsic_id':self.camera['calibration_id'],'parent':'base_link','estimated_frames':records,
            'convention':'right local translation + rotation vector perturbation; GNSS common shelf about its center'},indent=2)+'\n')
        self.static.sendTransform(out)

    def record_navigation(self,msg):
        t=stamp_seconds(msg.header.stamp)
        if t<=self.last_archive_stamp:return
        self.last_archive_stamp=t
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        v=msg.twist.twist.linear;w=msg.twist.twist.angular
        self.archive.write(json.dumps({'time_s':t,'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
            'position_m':[p.x,p.y,p.z],'orientation_xyzw':[q.x,q.y,q.z,q.w],
            'linear_velocity_m_s':[v.x,v.y,v.z],'angular_velocity_rad_s':[w.x,w.y,w.z],
            'pose_covariance':list(msg.pose.covariance),'twist_covariance':list(msg.twist.covariance),
            'calibration_id':self.calibration_id},allow_nan=False)+'\n')

    def destroy_node(self):
        if hasattr(self,'archive'):
            self.archive.flush();self.archive.close()
        return super().destroy_node()

    def joints(self,msg):
        positions=dict(zip(msg.name,msg.position))
        names=('fl','fr','rl','rr')
        try:
            drive=[positions[n+'_drive_joint'] for n in names]
            steer=[positions[n+'_steer_joint'] for n in names]
        except KeyError:return
        value=self.encoder.update(stamp_seconds(msg.header.stamp),drive,steer)
        if value is None:return
        twist,cov,residual=value
        self.wheel_speed=float(np.linalg.norm(twist))
        moment=stamp_seconds(msg.header.stamp)
        if not self.body_twist or moment>self.body_twist[-1][0]:
            self.body_twist.append((moment,float(twist[0]),float(twist[1]),float(twist[2])))
        out=Odometry();out.header.stamp=msg.header.stamp;out.header.frame_id='odom';out.child_frame_id='base_link'
        out.pose.pose.orientation.w=1.;out.pose.covariance=(np.eye(6)*1e6).ravel().tolist()
        out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.angular.z=twist.tolist()
        full=np.eye(6)*1e6; full[np.ix_([0,1,5],[0,1,5])]=cov
        out.twist.covariance=full.ravel().tolist()
        self.queue.push(stamp_seconds(msg.header.stamp)+delay_sample(self.config,'wheel',self.timings['wheel']),'wheel',out)

    def imu(self,msg):
        t=stamp_seconds(msg.header.stamp)
        if t<=self.last_raw_imu:return
        self.last_raw_imu=t
        noise=self.config['noise_enabled']
        gyro=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        accel=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        if not np.isfinite(np.r_[gyro,accel]).all():return
        if noise:
            gyro+=self.gyro_bias+self.rng.normal(0,self.config['gyro_sigma_rad_s'],3)
            accel+=self.accel_bias+self.rng.normal(0,self.config['accel_sigma_m_s2'],3)
        gyro=self.rotation@gyro;accel=self.rotation@accel
        out=Imu();out.header.stamp=msg.header.stamp;out.header.frame_id='base_link'
        out.orientation_covariance[0]=-1.  # Ignore Gazebo's ideal attitude entirely.
        out.angular_velocity.x,out.angular_velocity.y,out.angular_velocity.z=gyro.tolist()
        out.linear_acceleration.x,out.linear_acceleration.y,out.linear_acceleration.z=accel.tolist()
        gv=self.config['gyro_sigma_rad_s']**2+self.config['gyro_bias_sigma_rad_s']**2 if noise else 1e-10
        av=self.config['accel_sigma_m_s2']**2+self.config['accel_bias_sigma_m_s2']**2 if noise else 1e-10
        out.angular_velocity_covariance=(np.eye(3)*gv).ravel().tolist()
        out.linear_acceleration_covariance=(np.eye(3)*av).ravel().tolist()
        self.queue.push(t+delay_sample(self.config,'imu',self.timings['imu']),'imu',out)

    def publish_tilt(self,msg,vector,variance):
        angles=gravity_tilt(vector)
        out=Imu();out.header=msg.header
        q=Rotation.from_euler('xyz',[*angles,0]).as_quat()
        out.orientation.x,out.orientation.y,out.orientation.z,out.orientation.w=q.tolist()
        out.orientation_covariance=np.diag([variance,variance,1e6]).ravel().tolist()
        out.angular_velocity_covariance[0]=-1.;out.linear_acceleration_covariance[0]=-1.
        self.pubs['tilt'].publish(out);self.tilt_ready=True

    def tilt(self,msg):
        t=stamp_seconds(msg.header.stamp)
        g=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        a=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        # Measured encoder speed AND gyro AND gravity magnitude; never a command proxy.
        still=(self.wheel_speed<=.01 and np.linalg.norm(g)<=.01 and abs(np.linalg.norm(a)-9.81)<=.1)
        if still:
            self.gravity.clear();self.stationary_tilt(msg,t,a)
        else:
            self.stationary.clear();self.motion_tilt(msg,t,a)

    def stationary_tilt(self,msg,t,a):
        """Zero-velocity tilt update: the preferred source when it applies."""
        if self.stationary and t-self.stationary[-1][0]>.025:self.stationary.clear()
        self.stationary.append((t,a))
        if len(self.stationary)<self.stationary.maxlen or t-self.last_tilt<1.:return
        n=len(self.stationary)
        variance=((self.config['accel_sigma_m_s2']**2/n+self.config['accel_bias_sigma_m_s2']**2)/9.81**2
                  if self.config['noise_enabled'] else 1e-10)
        self.publish_tilt(msg,np.mean([v for _,v in self.stationary],axis=0),variance)
        self.last_tilt=t;self.stationary.clear()

    def motion_tilt(self,msg,t,a):
        """Gravity reference while driving, with wheel-derived kinematics removed.

        Standard AHRS practice: the accelerometer measures specific force, so an
        independent velocity source must supply the kinematic term before the
        residual can be read as gravity. The twist slope describes the window
        mean time, so the accelerometer is averaged over the SAME span and the
        result is stamped there; pairing it with the newest sample would bias
        pitch by jerk times half the window. Samples whose residual magnitude
        disagrees with gravity are rejected, never silently trusted.
        """
        if not self.config['motion_tilt_enabled']:self.gravity.clear();return
        if self.gravity and t-self.gravity[-1][0]>.025:self.gravity.clear()
        self.gravity.append((t,a,msg))
        if t-self.last_motion_tilt<self.config['motion_tilt_interval_s']:return
        value=body_acceleration(self.body_twist,self.config['motion_tilt_window_s'])
        if value is None or (self.body_twist and t-self.body_twist[-1][0]>.05):return
        kinematic,sigma,centre,(low,high)=value
        # Transient dynamics: coast on the gyro rather than trust a wheel model
        # that tyre slip and suspension pitch invalidate.
        if float(np.linalg.norm(kinematic))>self.config['motion_tilt_max_accel_m_s2']:
            self.motion_rejects+=1;return
        window=[r for r in self.gravity if low<=r[0]<=high]
        if len(window)<3:return
        vector=np.mean([v for _,v,_ in window],axis=0)-kinematic
        deviation=abs(float(np.linalg.norm(vector))-9.81)
        if deviation>self.config['motion_tilt_reject_m_s2']:self.motion_rejects+=1;return
        n=len(window)
        # Slip and suspension pitch scale with the compensation applied, so the
        # trust in a driving sample must fall as the kinematic term grows.
        kinematic_variance=(sigma**2+self.config['motion_tilt_extra_sigma_m_s2']**2+deviation**2
                            +(self.config['motion_tilt_accel_fraction']*float(np.linalg.norm(kinematic)))**2)
        noise=(self.config['accel_sigma_m_s2']**2/n+self.config['accel_bias_sigma_m_s2']**2
               if self.config['noise_enabled'] else 0.)
        variance=(noise+kinematic_variance)/9.81**2
        # Stamp at the window mean time; robot_localization accepts lagged data.
        anchor=min(window,key=lambda r:abs(r[0]-centre))[2]
        try:self.publish_tilt(anchor,vector,variance)
        except ValueError:self.motion_rejects+=1;return
        self.last_motion_tilt=t;self.motion_tilts+=1

    def local_pose(self,msg):
        self.local.append(msg)

    def gnss(self,msg,side):
        if msg.header.frame_id!='map':return
        t=stamp_seconds(msg.header.stamp)
        if t<=self.ages['gnss']:return
        self.pairs.setdefault(t,{})[side]=msg
        # Independent of pair arrival ordering; bounded synchronization by acquisition stamp.
        for old in sorted(self.pairs)[:-8]:del self.pairs[old]
        if len(self.pairs.get(t,{}))!=2:return
        pair=self.pairs.pop(t)
        if not self.tilt_ready:return
        local=next((m for m in reversed(self.local) if stamp_seconds(m.header.stamp)<=t+1e-7),None)
        if local is None or t-stamp_seconds(local.header.stamp)>.1:return
        q=local.pose.pose.orientation
        tilt=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[:2]
        points=[];cov=np.zeros((6,6))
        for i,side in enumerate(('left','right')):
            p=pair[side].pose.pose.position;points.append([p.x,p.y,p.z])
            cov[i*3:i*3+3,i*3:i*3+3]=np.array(pair[side].pose.covariance).reshape(6,6)[:3,:3]
        rho=self.config['gnss_correlation']
        cross=np.diag(rho*np.sqrt(np.diag(cov[:3,:3])*np.diag(cov[3:,3:])))
        cov[:3,3:]=cross;cov[3:,:3]=cross.T
        try:
            value,variance=dual_gnss_pose(points,self.mounts,tilt,cov)
            # Propagate uncertainty of the tilt used for heading/lever-arm correction.
            jac=np.empty((4,2));eps=1e-5
            for k in range(2):
                d=np.zeros(2);d[k]=eps
                a,_=dual_gnss_pose(points,self.mounts,tilt+d,cov)
                b,_=dual_gnss_pose(points,self.mounts,tilt-d,cov)
                jac[:,k]=(a-b)/(2*eps);jac[3,k]=wrap(a[3]-b[3])/(2*eps)
            tilt_cov=np.array(local.pose.covariance).reshape(6,6)[3:5,3:5]
            variance+=jac@tilt_cov@jac.T
        except ValueError:return
        out=PoseWithCovarianceStamped();out.header=msg.header
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=value[:3].tolist()
        orientation=Rotation.from_euler('xyz',[*tilt,value[3]]).as_quat()
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=orientation.tolist()
        full=np.eye(6)*1e6;full[np.ix_([0,1,2,5],[0,1,2,5])]=variance
        out.pose.covariance=full.ravel().tolist()
        self.pubs['gnss'].publish(out);self.ages['gnss']=t

    def deliver(self):
        now=self.get_clock().now().nanoseconds*1e-9
        for kind,msg in self.queue.pop(now):
            self.pubs[kind].publish(msg)
            self.ages[kind]=max(self.ages[kind],stamp_seconds(msg.header.stamp))
            if kind=='imu':self.tilt(msg)
            if kind=='wheel' and self.config['assume_continuous_ground_contact']:
                contact=TwistWithCovarianceStamped();contact.header.stamp=msg.header.stamp;contact.header.frame_id='base_link'
                covariance=np.eye(6)*1e6
                covariance[2,2]=self.config['contact_normal_speed_sigma_m_s']**2
                contact.twist.covariance=covariance.ravel().tolist()
                self.contact_pub.publish(contact)

    def health(self):
        self.archive.flush()
        now=self.get_clock().now().nanoseconds*1e-9
        age={k:now-v if math.isfinite(v) else None for k,v in self.ages.items()}
        local_ok=all(age[k] is not None and 0<=age[k]<.1 for k in ('wheel','imu'))
        local_filter_age=now-stamp_seconds(self.local[-1].header.stamp) if self.local else math.inf
        global_filter_age=now-self.last_archive_stamp
        filters_ok=-.02<=local_filter_age<.1 and -.02<=global_filter_age<.1
        fix_ok=age['gnss'] is not None and 0<=age['gnss']<.5
        state='READY' if local_ok and filters_ok and fix_ok and self.tilt_ready else 'NOT_READY'
        self.status.publish(String(data=json.dumps({'state':state,'measurement_age_s':age,
                    'filter_age_s':{'local':local_filter_age if math.isfinite(local_filter_age) else None,
                                    'global':global_filter_age if math.isfinite(global_filter_age) else None},
                    'tilt_initialized':self.tilt_ready,'delivery_queue_peak':self.queue.peak,
                    'motion_tilt_updates':self.motion_tilts,'motion_tilt_rejects':self.motion_rejects,
                    'stop_required':state!='READY'})))


def main():
    rclpy.init();node=MeasurementAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
