#!/usr/bin/env python3
"""Create a compact test scene using the original geometry/display and GPU recipe."""
import argparse,hashlib,json,os,sys
from pathlib import Path
import xml.etree.ElementTree as ET
from probe_runtime_material import export
ROOT=Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.shared_scene import validate


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--recipe',type=Path);a=p.parse_args()
    source=a.scene.resolve();out=a.output.resolve();m=validate(source)
    if a.recipe:
        out.mkdir(parents=True,exist_ok=False);recipe=json.loads(a.recipe.read_text())
        if recipe['source_scene_sha256']!=hashlib.sha256(source.read_bytes()).hexdigest():raise ValueError('recipe belongs to different reference scene')
        for name in recipe['payload_sha256']:os.link(a.recipe.parent/name,out/name)
        (out/'recipe.json').write_bytes(a.recipe.read_bytes())
    else:export(source,out)
    names=set(m['display_materials'])
    for asset in m['assets']:
        names.add(asset['mesh']);names.add(asset['collision_proxy']['mesh'])
    for name in names:os.link(source.parent/name,out/name)
    tree=ET.parse(source.parent/m['world'])
    for element in tree.iter():
        if element.tag in ('uri','albedo_map','normal_map'):element.text=str(out/Path(element.text).name)
    tree.write(out/'world.sdf',encoding='unicode');m['world_sha256']=hashlib.sha256((out/'world.sdf').read_bytes()).hexdigest()
    material=m['ground_material'];material.pop('tiles');material['schema']='agv.ground_material.recipe.v1'
    material['recipe']=dict(file='recipe.json',sha256=hashlib.sha256((out/'recipe.json').read_bytes()).hexdigest())
    m['profile']='runtime_recipe_road_probe';m['reference_scene_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
    (out/'manifest.json').write_text(json.dumps(m,indent=2)+'\n');validate(out/'manifest.json')
    print(json.dumps(dict(bytes=sum(p.stat().st_size for p in out.iterdir() if p.is_file()),scope='Logical standalone size; local unchanged assets are hard linked, no high-resolution baked tiles')))

if __name__=='__main__':main()
