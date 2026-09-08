"""Compile a validated flat rectangle into stopped transfers and continuous passes."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .planner import PlanningError


def check_position(plan,position):
    road=plan['request']['road'];r=Rotation.from_euler('z',road['yaw_rad'])
    xy=r.inv().apply(np.asarray(position)-road['origin_xyz_m'])[:2]
    xmin,xmax,ymin,ymax=plan['request']['drivable_bounds_xy_m'];radius=plan['sweep_radius_m']
    if not xmin+radius<=xy[0]<=xmax-radius or not ymin+radius<=xy[1]<=ymax-radius:
        raise PlanningError('current vehicle swept envelope leaves declared drivable bounds')


def compile_steps(plan,current_position,current_quaternion):
    if plan['frame_id']!='map':raise PlanningError('execution requires an explicit map-frame road transform')
    check_position(plan,current_position)
    entry=plan['tracks'][0]['base_entry_pose']
    # Rectangular inset is convex: checked endpoints also bound the straight approach.
    steps=[dict(kind='APPROACH',track_id=0,start={'position':list(current_position),'orientation_xyzw':list(current_quaternion)},
                end=entry,speed=min(.5,plan['scan_speed_m_s']))]
    segments=plan['segments'];i=0
    while i<len(segments):
        seg=segments[i];end=seg['points'][-1]['pose'];kind=seg['kind']
        if kind=='ACCELERATE':
            group=segments[i:i+3]
            if [v['kind'] for v in group]!=['ACCELERATE','SCAN','RUNOUT_BRAKE']:
                raise PlanningError('invalid continuous scan group')
            end=group[-1]['points'][-1]['pose'];kind='PASS';i+=2
        steps.append(dict(kind=kind,track_id=seg['track_id'],start=seg['points'][0]['pose'],end=end,
                          speed=plan['scan_speed_m_s'] if kind=='PASS' else min(.5,plan['scan_speed_m_s'])))
        i+=1
    return steps


def segment_arguments(step,position,quaternion):
    """Current pose to absolute planned endpoint; fixed map target survives residual errors."""
    p=np.asarray(position);r=Rotation.from_quat(quaternion)
    goal=np.asarray(step['end']['position']);target=Rotation.from_quat(step['end']['orientation_xyzw'])
    yaw=math.atan2((r.inv()*target).as_matrix()[1,0],(r.inv()*target).as_matrix()[0,0])
    # Heading alignment happens while stopped, before a translation/pass.
    if abs(yaw)>.025:
        return dict(kind='rotate',displacement=[0.,0.],angle=yaw,speed=.25)
    delta=r.inv().apply(goal-p)[:2]
    if np.linalg.norm(delta)>.035 and step['kind']!='ROTATE_180':
        return dict(kind='translate',displacement=delta,angle=0.,speed=step['speed'])
    return None
