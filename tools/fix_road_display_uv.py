#!/usr/bin/env python3
"""Migrate legacy display OBJ V direction; leave geometry and all texels intact."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/agv_linescan'))
from agv_linescan.shared_scene import digest,validate


def main():
    p=argparse.ArgumentParser();p.add_argument('manifest',type=Path);a=p.parse_args()
    m=validate(a.manifest);root=a.manifest.parent;changed=[]
    for asset in m['assets']:
        if asset.get('material')!='ground':continue
        projection=asset.get('display_uv_projection',m.get('display_uv_projection'))
        if projection.get('v_direction')=='decreasing_world_y':continue
        path=root/asset['mesh'];lines=path.read_text().splitlines(keepends=True)
        for i,line in enumerate(lines):
            if line.startswith('vt '):
                values=line.split();lines[i]=f'vt {values[1]} {1-float(values[2]):.9f}\n'
        temp=path.with_suffix('.obj.tmp');temp.write_text(''.join(lines));temp.replace(path)
        asset['sha256']=digest(path)
        asset['display_uv_projection']={**projection,'v_direction':'decreasing_world_y'}
        changed.append(asset['name'])
    if changed:
        m['display_uv']='PNG rows increase with world y; OBJ V decreases with world y'
        temp=a.manifest.with_suffix('.json.tmp');temp.write_text(json.dumps(m,indent=2)+'\n');temp.replace(a.manifest)
    validate(a.manifest);print(json.dumps(dict(updated=changed,texels_and_geometry_unchanged=True)))


if __name__=='__main__':main()
