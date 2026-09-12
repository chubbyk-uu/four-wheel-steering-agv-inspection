import copy
import math
import pytest
import yaml
from budget_full_road import budget,ROOT
from road_layout import layout,display_regions
from agv_mission.planner import plan,Vehicle,PlanningError


def test_existing_tile_bounds_unchanged():
    old=layout(20)
    assert old['tiles_x']==44 and old['tiles_y']==24
    assert old['optical_valid_bounds_xy_m']==pytest.approx([-1.024,21.504,-6.144,6.144])


def test_full_region_all_speeds_fit_with_tracking_allowance():
    report,request=budget()
    config=ROOT/'src/agv_description/config'
    vehicle=Vehicle.from_configs(yaml.safe_load((config/'platform.yaml').read_text()),yaml.safe_load((config/'linescan.yaml').read_text()))
    for speed in (.5,1.,2.,vehicle.rated_scan_speed):
        request['scan_speed_m_s']=speed;p=plan(request,vehicle)
        assert p['track_count']==10
        assert [t['center_road_y_m'] for t in p['tracks']]==[i-.5 for i in range(-4,6)]
        left,right,bottom,top=request['drivable_bounds_xy_m'];r=p['sweep_radius_m']+.1
        for segment in p['segments']:
            for point in segment['points']:
                x,y,_=point['pose']['position']
                assert left+r<=x<=right-r and bottom+r<=y<=top-r
        # Capture starts during acceleration, not at the ROI boundary. Check
        # every straight-pass point plus uncertainty against the optical mesh.
        lo,hi,by,ey=request['optical_bounds_xy_m']
        for segment in p['segments']:
            if segment['kind'] not in ('ACCELERATE','SCAN','RUNOUT_BRAKE'):continue
            for point in segment['points']:
                x,y,_=point['scan_center_xyz_m']
                assert lo+.1<=x<=hi-.1
                assert by+.78+.1<=y<=ey-.78-.1
    bad=copy.deepcopy(request);bad['drivable_bounds_xy_m']=[0,100,-5,5]
    with pytest.raises(PlanningError,match='swept vehicle'):plan(bad,vehicle)
    assert report['raw_output_bytes_per_run_budget']>4096*math.ceil(1000/(1.5/4096))


def test_display_partitions_cover_apron_without_gaps_or_large_textures():
    bounds=layout(100,8,1.5)['optical_valid_bounds_xy_m']
    regions=display_regions(100,bounds)
    area=sum((x1-x0)*(y1-y0) for x0,x1,y0,y1 in regions)
    assert area==pytest.approx((bounds[1]-bounds[0])*(bounds[3]-bounds[2]))
    assert max(x1-x0 for x0,x1,y0,y1 in regions)<=5
    assert max(round((y1-y0)/.004) for x0,x1,y0,y1 in regions)<2048
