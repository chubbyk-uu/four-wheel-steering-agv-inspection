"""Bind operator requests to the actually loaded, world-baked flat road."""
from copy import deepcopy
import math


def rectangle(value):
    if not isinstance(value,(list,tuple)) or len(value)!=4 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in value):
        raise ValueError('invalid scene bounds')
    if value[0]>=value[1] or value[2]>=value[3]:raise ValueError('invalid scene bounds ordering')
    return list(value)


def contains(outer,inner):
    a,b,c,d=rectangle(outer);x,y,z,w=rectangle(inner)
    return a-1e-9<=x and y<=b+1e-9 and c-1e-9<=z and w<=d+1e-9


def scene_bounds(scene):
    if scene.get('transform')!='identity_world_baked' or scene.get('frame')!='world':
        raise ValueError('operator requires an explicitly world-baked scene')
    inspection=rectangle(scene['inspection_bounds_xy_m'])
    drive=rectangle(scene.get('drivable_bounds_xy_m',inspection))
    optical=rectangle(scene['optical_valid_bounds_xy_m'])
    if not contains(drive,inspection) or not contains(optical,drive):
        raise ValueError('scene inspection/drivable/optical bounds are inconsistent')
    return dict(inspection_bounds=inspection,bounds=drive,optical_bounds=optical)


def bind_request(request,road):
    result=deepcopy(request)
    result['drivable_bounds_xy_m']=list(road['bounds'])
    result['optical_bounds_xy_m']=list(road['optical_bounds'])
    return result


def check_request_scene(request,road):
    if road is None:return
    registration=request['road']
    if registration['frame_id']!='map' or registration['origin_xyz_m']!=[0,0,0] or registration['yaw_rad']!=0:
        raise ValueError('loaded scene requires the current identity road-to-map registration')
    if not contains(road['bounds'],request['drivable_bounds_xy_m']) or not contains(road['optical_bounds'],request['optical_bounds_xy_m']):
        raise ValueError('request bounds exceed the loaded scene; load the matching road')
    region=request['region'];x,y=region['start_xy_m']
    if not contains(road['inspection_bounds'],[x,x+region['length_m'],y,y+region['width_m']]):
        raise ValueError('acquisition region leaves the loaded inspection area')
