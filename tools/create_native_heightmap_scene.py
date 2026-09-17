#!/usr/bin/env python3
"""Replace tiled road collision meshes with one checked native GZ heightmap."""
import argparse, json, os, sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.shared_scene import digest, validate


def resample(field, rows, columns):
    z=np.asarray(field['z'],float);x=np.asarray(field['x'],float);y=np.asarray(field['y'],float)
    tx=np.linspace(x[0],x[-1],columns);ty=np.linspace(y[0],y[-1],rows)
    along_x=np.asarray([np.interp(tx,x,row) for row in z])
    output=np.asarray([np.interp(ty,y,along_x[:,column]) for column in range(columns)]).T
    return tx,ty,output


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--samples',type=int,default=257)
    parser.add_argument('--collision-detector',choices=('ode','fcl','bullet'),default='ode')
    args=parser.parse_args()
    # DART / gz-physics 7 crashes in ImageHeightmap::FillHeightMap for a
    # rectangular 2^n+1 image (confirmed with 513x257).  Keep one square size
    # explicit here instead of exposing independent dimensions which validate
    # as SDF but fail in the physics backend.
    if args.samples<3 or (args.samples-1)&(args.samples-2):
        raise ValueError('square heightmap samples must be 2^n+1')
    source=args.source.resolve();manifest=validate(source/'manifest.json')
    field_path=source/'road_heightfield.json';field=json.loads(field_path.read_text())
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    for path in source.iterdir():
        if path.is_file():os.link(path,output/path.name)
    tx,ty,z=resample(field,args.samples,args.samples)
    low,high=float(z.min()),float(z.max());span=high-low
    encoded=np.rint((z-low)/span*65535).astype(np.uint16)
    # Image row zero is the +Y edge in GZ heightmaps.
    image_path=output/'road_collision_heightmap.png'
    Image.fromarray(np.flipud(encoded),mode='I;16').save(image_path)
    source_field=output/field_path.name
    world_path=output/manifest['world'];world_path.unlink();tree=ET.parse(source/manifest['world']);world=tree.getroot().find('world')
    for element in world.iter():
        if element.text and str(source) in element.text:
            element.text=element.text.replace(str(source),str(output))
    physics=world.find('physics');dart=physics.find('dart')
    if dart is None:dart=ET.SubElement(physics,'dart')
    detector=dart.find('collision_detector')
    if detector is None:detector=ET.SubElement(dart,'collision_detector')
    detector.text=args.collision_detector
    for model in world.findall('model'):
        if model.get('name') in {a['name'] for a in manifest['assets']}:
            for collision in list(model.find('link').findall('collision')):model.find('link').remove(collision)
    name='road_native_heightmap_collision'
    size=[float(tx[-1]-tx[0]),float(ty[-1]-ty[0]),span]
    position=[float((tx[0]+tx[-1])/2),float((ty[0]+ty[-1])/2),low]
    model=ET.SubElement(world,'model',name=name);ET.SubElement(model,'static').text='true'
    # gz-physics ignores <heightmap><pos>: the offset reaches DART only through
    # the model pose.  Written into <pos> the collision surface stays centred on
    # the world origin, so a road spanning x=[-9.3,109.1] is supported only to
    # x=59.2 and the vehicle falls through beyond it -- which a 20 m probe can
    # never reach.  Keep the placement here and leave <pos> at zero.
    ET.SubElement(model,'pose').text=' '.join(f'{value:.12g}' for value in position+[0.,0.,0.])
    link=ET.SubElement(model,'link',name='road');collision=ET.SubElement(link,'collision',name='collision')
    shape=ET.SubElement(ET.SubElement(collision,'geometry'),'heightmap')
    ET.SubElement(shape,'uri').text=str(image_path)
    ET.SubElement(shape,'size').text=' '.join(f'{value:.12g}' for value in size)
    ET.SubElement(shape,'pos').text='0 0 0'
    tree.write(world_path,encoding='unicode')
    manifest['world_sha256']=digest(world_path)
    manifest['profile']=manifest['profile']+'_native_heightmap'
    # The per-block proxies are no longer loaded by the world.  Keep them
    # declared and checked against the optical mesh, but say so, so nobody
    # reads a stale collision_proxy as the surface the wheels are on.
    for asset in manifest['assets']:
        if 'collision_proxy' in asset:
            asset['collision_proxy']['role']='optical_reference_only'
    manifest['physics_heightmap']={
        'model_name':name,'image':image_path.name,'sha256':digest(image_path),
        'source_heightfield':source_field.name,'source_sha256':digest(source_field),
        'encoding':'png_uint16_min_to_max_v1','samples_xy':[args.samples,args.samples],
        'size_m':size,'position_m':position,'collision_detector':args.collision_detector,
        'cell_size_m':[size[0]/(args.samples-1),size[1]/(args.samples-1)],
        'max_quantization_error_m':span/65535/2,
        'source_grid_shape':[len(field['y']),len(field['x'])],
    }
    manifest_path=output/'manifest.json';manifest_path.unlink();manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    validate(manifest_path)
    print(json.dumps({'passed':True,'samples':args.samples,'height_range_m':[low,high],
        'max_quantization_error_m':span/65535/2,'collision_detector':args.collision_detector}))


if __name__=='__main__':main()
