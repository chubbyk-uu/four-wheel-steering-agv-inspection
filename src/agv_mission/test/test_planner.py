import copy
import math
from pathlib import Path
import pytest
import yaml
from agv_mission.planner import PlanningError, Vehicle, plan

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def request_data():
    return yaml.safe_load((ROOT/'config/rectangle_demo.yaml').read_text())


@pytest.fixture
def vehicle():
    config = ROOT.parent/'agv_description/config'
    return Vehicle.from_configs(yaml.safe_load((config/'platform.yaml').read_text()),
                                yaml.safe_load((config/'linescan.yaml').read_text()))


def test_full_coverage_and_camera_offset(request_data, vehicle):
    p = plan(request_data, vehicle)
    assert p['track_count'] == 4
    assert p['actual_track_spacing_m'] == pytest.approx(1)
    intervals = [(t['center_road_y_m']-.75, t['center_road_y_m']+.75) for t in p['tracks']]
    assert intervals[0][0] == -2.25
    assert intervals[-1][1] == 2.25
    assert all(b[0] <= a[1] for a,b in zip(intervals, intervals[1:]))
    for seg in p['segments']:
        for pt in seg['points']:
            x,y,z = pt['pose']['position']
            q = pt['pose']['orientation_xyzw']
            angle = 2*math.atan2(q[2],q[3])
            assert pt['scan_center_xyz_m'] == pytest.approx([x+1.15*math.cos(angle),y+1.15*math.sin(angle),0])
            assert z == vehicle.base_height
        if seg['kind'] == 'SCAN':
            track = p['tracks'][seg['track_id']]
            assert seg['points'][0]['scan_center_xyz_m'] == pytest.approx(track['scan_start_xyz_m'])
            assert seg['points'][-1]['scan_center_xyz_m'] == pytest.approx(track['scan_end_xyz_m'])
    for a,b in zip(p['segments'],p['segments'][1:]):
        assert a['points'][-1]['pose'] == b['points'][0]['pose']
        assert a['points'][-1]['s_m'] == b['points'][0]['s_m']
    assert [s['kind'] for s in p['segments']].count('ENTRY') == 0
    assert all(not s['capture'] for s in p['segments'] if s['kind'] != 'SCAN')


@pytest.mark.parametrize('width,count',[(.2,1),(1.5,1),(1.51,2),(4,4),(10,10)])
def test_width_cases(request_data, vehicle, width, count):
    request_data['region']['start_xy_m'][1] = -width/2
    request_data['region']['width_m'] = width
    request_data['drivable_bounds_xy_m'] = [-20,40,-20,20]
    request_data['optical_bounds_xy_m'] = [-20,40,-20,20]
    p = plan(request_data,vehicle)
    assert p['track_count'] == count
    assert 0 <= p['actual_track_spacing_m'] <= 1
    assert p['tracks'][0]['center_road_y_m']-.75 <= -width/2+1e-10
    assert p['tracks'][-1]['center_road_y_m']+.75 >= width/2-1e-10


def test_tracking_budget_preserves_fixed_spacing(request_data, vehicle):
    request_data['coverage_error_m'] = .1
    p = plan(request_data,vehicle)
    assert p['actual_track_spacing_m'] == pytest.approx(1)
    first,last = p['tracks'][0],p['tracks'][-1]
    assert first['center_road_y_m']+.1-.75 <= -2
    assert last['center_road_y_m']-.1+.75 >= 2
    assert p['guaranteed_overlap_m'] == pytest.approx(.3)


def test_road_transform_is_not_user_heading(request_data, vehicle):
    baseline = plan(request_data,vehicle)
    request_data['road']['origin_xyz_m'] = [23,-7,2]
    request_data['road']['yaw_rad'] = math.pi/2
    changed = plan(request_data,vehicle)
    for a,b in zip(baseline['segments'][0]['points'],changed['segments'][0]['points']):
        x,y,z = a['pose']['position']
        assert b['pose']['position'] == pytest.approx([23-y,-7+x,2+z])
        assert b['surface_normal'] == [0,0,1]
        assert b['tangent'] == pytest.approx([0,1,0])


def test_braking_and_acceleration_distinct(request_data, vehicle):
    from dataclasses import replace
    vehicle=replace(vehicle,accel=.8,decel=1.)
    # Fast enough that both distances clear the sensor over-run floor below.
    request_data['scan_speed_m_s']=1.
    p=plan(request_data,vehicle)
    assert p['lead_distance_m'] == pytest.approx(1/(2*.8)+.1)
    assert p['runout_distance_m'] == pytest.approx(1/(2*1)+.1)


def test_runout_contains_the_discardable_sensor_tail(request_data, vehicle):
    # Capture may not close at the region edge. The sensor drops a closing image
    # shorter than min_tail_rows, so the last archived row trails the closing
    # point by that much, and the audit shrinks each span by the declared
    # uncertainty on top. Closing 0.10 m past a 13 m pass against a 0.37 m
    # discardable tail left 0.26-0.31 m of every pass unverified.
    assert vehicle.discardable_tail_m == pytest.approx(1000*.0003662109375)
    request_data['coverage_error_m']=.1
    request_data['scan_speed_m_s']=1.
    p=plan(request_data,vehicle)
    assert p['scan_overrun_distance_m'] == pytest.approx(vehicle.discardable_tail_m+.2)
    assert p['runout_distance_m'] >= p['scan_overrun_distance_m']
    # Where the braking distance is the shorter of the two, the over-run sets the
    # run-out: the vehicle must still drive far enough to verify the region tail.
    request_data['scan_speed_m_s']=.25
    slow=plan(request_data,vehicle)
    assert slow['runout_distance_m'] == pytest.approx(slow['scan_overrun_distance_m']+.1)
    assert slow['runout_distance_m'] > .25**2/(2*vehicle.decel)+.1
    # Capture closes strictly before the tracker stops, in every case.
    for report in (p,slow):
        assert report['runout_distance_m'] > report['scan_overrun_distance_m']
    # The head only has to clear the audit's own shrink, not a discarded tail.
    assert slow['lead_distance_m'] >= request_data['coverage_error_m']
    request_data['scan_speed_m_s'] = vehicle.max_speed
    with pytest.raises(PlanningError,match='swept vehicle'):
        plan(request_data,vehicle)


@pytest.mark.parametrize('key,value', [('scan_speed_m_s',3),('scan_speed_m_s',float('nan')),
    ('track_spacing_m',0),('track_spacing_m',1.6),('coverage_error_m',.8),
    ('vehicle_sweep_radius_m',1.2),('sample_step_m',False),('sample_step_m',1e-9),
    ('longitudinal_margin_m',-1),('scan_speed_m_s','0.5')])
def test_reject_bad_inputs(request_data,vehicle,key,value):
    request_data[key]=value
    with pytest.raises(PlanningError):
        plan(request_data,vehicle)


def test_reject_boundaries_and_unsupported_surface(request_data, vehicle):
    for key,value in [('drivable_bounds_xy_m',[0,15,-2,2]),
                      ('optical_bounds_xy_m',[6,14,-2,2])]:
        r=copy.deepcopy(request_data)
        r[key]=value
        with pytest.raises(PlanningError):
            plan(r,vehicle)
    request_data['road']['surface']='slope'
    with pytest.raises(PlanningError,match='surface'):
        plan(request_data,vehicle)


def test_unknown_keys_rejected(request_data,vehicle):
    request_data['speed']=10
    with pytest.raises(PlanningError,match='unknown keys'):
        plan(request_data,vehicle)


def test_analytic_envelope_independent_of_sampling(request_data,vehicle):
    request_data['sample_step_m']=1000
    request_data['drivable_bounds_xy_m']=[0,14,-5,5]
    with pytest.raises(PlanningError,match='swept vehicle'):
        plan(request_data,vehicle)


def test_forward_runout_meets_next_entry_without_reverse(request_data,vehicle):
    p=plan(request_data,vehicle)
    assert p['turn_runout_distance_m']==pytest.approx(2*vehicle.camera_x+p['lead_distance_m'])
    turns=[s for s in p['segments'] if s['kind']=='ROTATE_180']
    for turn in turns:
        assert turn['points'][-1]['pose']==p['tracks'][turn['track_id']]['base_entry_pose']
    assert not any(s['kind']=='ENTRY' for s in p['segments'])


@pytest.mark.parametrize('group,key,value', [
    ('camera','led_length_m',1.8),('camera','camera_x_m',1.5),
    ('camera','led_forward_offset_m',.2),('platform','wheelbase',1.8),
    ('platform','gnss_baseline',2.),('platform','body_width',float('nan'))])
def test_changed_accessories_require_envelope_reaudit(group,key,value):
    directory=ROOT.parent/'agv_description/config'
    platform=yaml.safe_load((directory/'platform.yaml').read_text())
    camera=yaml.safe_load((directory/'linescan.yaml').read_text())
    (camera if group=='camera' else platform)[key]=value
    with pytest.raises(PlanningError,match='envelope'):
        Vehicle.from_configs(platform,camera)
