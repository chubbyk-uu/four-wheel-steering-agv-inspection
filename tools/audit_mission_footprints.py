#!/usr/bin/env python3
"""Independent sampled ground-footprint audit using explicitly isolated render truth.

This does not generate production pose tags and is not a pixel-level proof.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def audit(mission):
    plan=json.loads((mission/'plan.json').read_text());manifest=json.loads((mission/'capture_manifest.json').read_text())
    road=plan['request']['road'];inverse=Rotation.from_euler('z',road['yaw_rad']).inv();origin=np.array(road['origin_xyz_m'])
    from yaml import safe_load
    camera=safe_load((mission.parent/'camera.yaml').read_text())
    scale=camera['width']*camera['pixel_pitch_m']/(2*camera['focal_length_m'])
    x0,y0=plan['request']['region']['start_xy_m'];length=plan['request']['region']['length_m'];width=plan['request']['region']['width_m']
    corridors=[]
    for track in plan['tracks']:
        sides=[]
        for block in manifest['blocks']:
            if block['track_id']!=track['id']:continue
            meta=json.loads(Path(block['image']).with_suffix('.json').read_text())
            for tag in meta['pose_tags']:
                p=np.array(tag['camera_position_world_m']);r=np.array(tag['camera_rotation_world'])
                center=p+r[:,2]*(origin[2]-p[2])/r[2,2]
                c=inverse.apply(center-origin)
                if not x0<=c[0]<=x0+length:continue
                edges=[]
                for u in (-scale,scale):
                    ray=r@np.array([u,0,1]);hit=p+ray*(origin[2]-p[2])/ray[2]
                    edges.append(inverse.apply(hit-origin)[1])
                sides.append(sorted(edges))
        if not sides:raise ValueError('no in-region sampled footprints')
        a=np.array(sides);corridors.append({'track_id':track['id'],'samples':len(sides),
            'intersection_y_m':[float(a[:,0].max()),float(a[:,1].min())]})
    intervals=sorted(c['intersection_y_m'] for c in corridors)
    edge=intervals[0][1];ok=intervals[0][0]<=y0;overlaps=[]
    for left,right in intervals[1:]:overlaps.append(edge-left);ok=ok and left<=edge;edge=max(edge,right)
    ok=ok and edge>=y0+width
    report={'passed':bool(ok),'scope':'sampled nominal ground footprint corridors from independent rendering truth; no pixel-level coverage claim',
            'corridors':corridors,'minimum_sampled_overlap_m':min(overlaps) if overlaps else None}
    (mission/'footprint_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    if not ok:raise ValueError('sampled lateral coverage gap')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mission',type=Path,required=True);a=p.parse_args();print(json.dumps(audit(a.mission)))
