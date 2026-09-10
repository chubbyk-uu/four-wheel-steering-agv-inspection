#!/usr/bin/env python3
"""Reproducible spatial/storage budget; no texture allocation or simulation."""
import argparse
import json
import math
from pathlib import Path
import sys
import yaml
from road_layout import layout, display_regions

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/agv_mission'))
from agv_mission.planner import Vehicle,plan


def budget():
    config=ROOT/'src/agv_description/config'
    platform=yaml.safe_load((config/'platform.yaml').read_text())
    camera=yaml.safe_load((config/'linescan.yaml').read_text())
    vehicle=Vehicle.from_configs(platform,camera)
    request=yaml.safe_load((ROOT/'src/agv_mission/config/rectangle_demo.yaml').read_text())
    request.update(mission_id='full_road_100x10',scan_speed_m_s=platform['max_speed'],coverage_error_m=.1)
    request['road']['frame_id']='map'
    request['region']=dict(start_xy_m=[0.,-5.],length_m=100.,width_m=10.)
    request['drivable_bounds_xy_m']=[-100,200,-100,100]
    request['optical_bounds_xy_m']=[-100,200,-100,100]
    preview=plan(request,vehicle)
    points=[p['pose']['position'] for s in preview['segments'] for p in s['points']]
    r=preview['sweep_radius_m'];error=request['coverage_error_m']
    # Add tracking-error allowance outside the planner's nominal swept disc,
    # and 0.25m construction slack, then round up to a practical half metre.
    min_x=min(p[0] for p in points)-r;max_x=max(p[0] for p in points)+r
    min_y=min(p[1] for p in points)-r;max_y=max(p[1] for p in points)+r
    end=math.ceil((max(-min_x,max_x-100)+error+.25)*2)/2
    side=math.ceil((max(-5-min_y,max_y-5)+error+.25)*2)/2
    road=layout(100,end,side)
    request['drivable_bounds_xy_m']=road['drivable_bounds_xy_m']
    request['optical_bounds_xy_m']=road['optical_valid_bounds_xy_m']
    preview=plan(request,vehicle)
    pitch=camera['nominal_width_m']/camera['width']
    # Gate opens at PASS entry (includes lead), closes 0.10m beyond ROI end.
    # Additional .20m/track below is output budgeting slack, not permission
    # for controller error or a substitute for measured trigger counts.
    captured=preview['track_count']*(100+preview['lead_distance_m']+.1+.2)
    raw=math.ceil(captured/pitch)*camera['width']
    blocks=math.ceil(raw/(camera['width']*camera['block_rows']))+preview['track_count']
    # Current metadata embeds the entire scene contract in EVERY image.
    # Reserve 4MiB/block plus 512MiB navigation/diagnostics per long session.
    metadata=blocks*4*2**20+512*2**20
    regions=display_regions(100,road['optical_valid_bounds_xy_m'])
    pixels=sum(round((x1-x0)/.004)*round((y1-y0)/.004) for x0,x1,y0,y1 in regions)
    cache=32*(2048+4)**2*3
    result=dict(schema='agv.full_road_budget.v1',status='ESTIMATE_NOT_RUNTIME_ACCEPTANCE',
        road=road,non_acquisition_end_buffer_m=end,non_acquisition_side_buffer_m=side,
        nominal_swept_bounds_xy_m=[min_x,max_x,min_y,max_y],tracking_allowance_m=error,construction_slack_m=.25,
        track_count=preview['track_count'],track_centers_y_m=[t['center_road_y_m'] for t in preview['tracks']],
        speed_m_s=request['scan_speed_m_s'],trigger_hz=request['scan_speed_m_s']/pitch,
        lead_m=preview['lead_distance_m'],turn_runout_m=preview['turn_runout_distance_m'],
        first_entry_pose=preview['tracks'][0]['base_entry_pose'],total_base_translation_m=preview['total_base_translation_m'],
        raw_output_bytes_per_run_budget=raw,metadata_navigation_bytes_per_run_budget=metadata,raw_output_basis='10 passes, lead + ROI + gate end + 0.20m/track slack; no rescan, ROS bag or TIFF',
        disk=dict(texture_bytes=road['texture_bytes'],source_geometry_display_temporary_allowance_bytes=15*2**30,
                  three_runs_plus_one_full_rescan_bytes=4*(raw+metadata),total_new_bytes_budget=road['texture_bytes']+15*2**30+4*(raw+metadata),
                  note='Existing assets/captures excluded; no raw duplicate ROS bag; retain additional free space'),
        gpu=dict(texture_cache_bytes=cache,gz_color_normal_rgba_mip_estimate_bytes=math.ceil(pixels*8*4/3),
                 rviz_rgba_mip_upper_bound_bytes=len(regions)*1024**2*4*4//3,
                 static_geometry_bvh_scratch_allowance_bytes=2*2**30,
                 camera_output_queue_robot_allowance_bytes=512*2**20,
                 driver_windows_gui_allowance_bytes=3*2**30,total_operating_ceiling_bytes=10*2**30,
                 note='Texture-cache bytes exact; other categories estimates/ceilings, not measured. OptiX currently retains GAS build scratch.'),
        host=dict(bake_rss_ceiling_bytes=12*2**30,runtime_rss_ceiling_bytes=20*2**30,
                  note='Resident process memory, not OS reclaimable file cache; monitor actual peaks; bake and simulation separately'),
        geometry=dict(reference_20m_triangles=1522958,estimated_100m_triangles=8000000,
                      note='Linear extrapolation plus apron; actual generated counts and BVH allocation required'),
        acceptance_pending=['asset hashes and shared gutters','actual GZ/RViz road alignment','GPU/host memory and cache under long motion',
                            '100x10 coverage and rescan','repeated runs and fault matrix','independent 11 kHz'])
    return result,request


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path);p.add_argument('--request',type=Path);a=p.parse_args()
    result,request=budget()
    if a.output:a.output.write_text(json.dumps(result,indent=2)+'\n')
    if a.request:a.request.write_text(yaml.safe_dump(request,sort_keys=False))
    print(json.dumps(dict(tracks=result['track_count'],road=result['road'],raw_gb=result['raw_output_bytes_per_run_budget']/1e9,
                         new_disk_gb=result['disk']['total_new_bytes_budget']/1e9)))

if __name__=='__main__':main()
