#!/usr/bin/env python3
"""Isolate true groove geometry cost, without claiming textured/full-system acceptance."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import Delaunay
from scipy.ndimage import binary_dilation
from PIL import Image, ImageDraw
import xacro
from bake_concrete_road import prepare_crack, sha
ROOT=Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.robot_scene import export, split_visual_links


def geometry():
    # Sparse local 0.25 mm support from the existing AI image, not random paths.
    step=.00025
    _,alpha,stats=prepare_crack(0,step)
    mask=binary_dilation(alpha>0,iterations=2)
    iy,ix=np.nonzero(mask)
    x=1.4+(ix+.5)*step;y=(iy+.5)*step-.07
    z=-.002*np.clip(alpha[iy,ix],0,1)  # synthetic 2 mm depth, not measured
    points=np.column_stack((x,y,z))
    # Coarse flat support plus exact 8 mm joint edges, 3 mm deep, 1 mm bevels.
    xx,yy=np.meshgrid(np.linspace(0,5,51),np.linspace(-2.5,2.5,51))
    flat=np.column_stack((xx.ravel(),yy.ravel(),np.zeros(xx.size)))
    flat=flat[~((flat[:,0]>=1.4)&(flat[:,0]<=3.6)&(abs(flat[:,1])<=.071))]
    flat=flat[abs(flat[:,0]-2.5)>.005]
    xx,yy=np.meshgrid([2.496,2.497,2.503,2.504],np.linspace(-2.5,2.5,501))
    joint=np.column_stack((xx.ravel(),yy.ravel(),np.zeros(xx.size)))
    points=np.vstack((points,flat,joint))
    _,ids=np.unique(points[:,:2],axis=0,return_index=True);points=points[ids]
    d=abs(points[:,0]-2.5)
    points[:,2]=np.minimum(points[:,2],-.003*np.clip((.004-d)/.001,0,1))
    faces=Delaunay(points[:,:2]).simplices
    a,b,c=points[faces[:,0],:2],points[faces[:,1],:2],points[faces[:,2],:2]
    signed=(b[:,0]-a[:,0])*(c[:,1]-a[:,1])-(b[:,1]-a[:,1])*(c[:,0]-a[:,0])
    assert np.all(signed>0), 'upward winding and nondegenerate triangles'
    assert abs(signed.sum()/2-25)<1e-7, 'single surface covers 5x5 m without overlaid plane'
    assert points[:,2].min()>=-.00300001 and points[:,2].max()==0
    return points,faces,stats


def write_scene(out,vertices,faces):
    out.mkdir();obj=out/'terrain.obj'
    with obj.open('w') as f:
        np.savetxt(f,vertices,fmt='v %.9f %.9f %.9f')
        np.savetxt(f,faces+1,fmt='f %d %d %d')
    # Both visual and collision point to the exact same mesh; not launched in this test.
    sdf=ET.Element('sdf',version='1.9');world=ET.SubElement(sdf,'world',name='groove_fixture')
    model=ET.SubElement(world,'model',name='terrain');ET.SubElement(model,'static').text='true'
    link=ET.SubElement(model,'link',name='terrain')
    for kind in ('visual','collision'):
        item=ET.SubElement(link,kind,name=kind)
        mesh=ET.SubElement(ET.SubElement(item,'geometry'),'mesh');ET.SubElement(mesh,'uri').text=obj.name
    ET.ElementTree(sdf).write(out/'world.sdf',encoding='unicode')
    manifest=dict(schema='agv.shared.static_scene.v1',units='m',frame='world',transform='identity_world_baked',
                  assets=[dict(name='terrain',mesh=obj.name,sha256=sha(obj),triangles=len(faces),linear_reflectance=.65)])
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    robot=split_visual_links(xacro.process_file(str(ROOT/'src/agv_description/urdf/agv.urdf.xacro')).toxml())
    rp=export(robot,out/'robot');r=json.loads(rp.read_text())
    r['groups']=[g for g in r['groups'] if g['name']==r['led_emitters']['link']]
    rp.write_text(json.dumps(r))
    v,f,stats=geometry();vf=v.copy();vf[:,2]=0
    simple=np.array([[0,-2.5,0],[5,-2.5,0],[5,2.5,0],[0,2.5,0]])
    fixtures={'plane':(simple,np.array([[0,1,2],[0,2,3]])),'flat_dense':(vf,f),'grooves':(v,f)}
    report=dict(scope='isolated incremental geometry cost; no normal/roughness/color textures or GUI/physics/ROS/archive',
                crack=stats,crack_depth_m=.002,joint_width_m=.008,joint_depth_m=.003,
                area_m2=25,geometry_checks='positive winding, nondegenerate, projected area 25 m2, depth bounds; shared visual/collision OBJ',
                memory_scope='explicit cudaMalloc allocations including GAS build scratch retained by current implementation; excludes opaque driver/context and Gazebo',
                target_lines_per_second=11000,fixtures={})
    for name,(vertices,faces) in fixtures.items():
        folder=out/name;write_scene(folder,vertices,faces)
        cmd=[str(ROOT/'build/agv_linescan/benchmark_grooves'),str(folder/'manifest.json'),str(rp),
             str(ROOT/'build/agv_linescan/agv_optix_scan.ptx'),str(ROOT/'src/agv_description/config/linescan.yaml'),str(folder/'images')]
        with (folder/'run.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
        report['fixtures'][name]=dict(triangles=len(faces),vertices=len(vertices),timing=json.loads((folder/'images/timing.json').read_text()))
    report['test_order']=[list(fixtures),list(reversed(fixtures))]
    for name in reversed(fixtures):
        folder=out/name
        cmd=[str(ROOT/'build/agv_linescan/benchmark_grooves'),str(folder/'manifest.json'),str(rp),
             str(ROOT/'build/agv_linescan/agv_optix_scan.ptx'),str(ROOT/'src/agv_description/config/linescan.yaml'),str(folder/'images_repeat')]
        with (folder/'repeat.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
        report['fixtures'][name]['reverse_order_timing']=json.loads((folder/'images_repeat/timing.json').read_text())
    metrics=[]
    for n in (4,8,16,32,64):
        flat=np.array(Image.open(out/f'plane/images/samples_{n}.pgm'),dtype=np.int16)
        control=np.array(Image.open(out/f'flat_dense/images/samples_{n}.pgm'),dtype=np.int16)
        groove=np.array(Image.open(out/f'grooves/images/samples_{n}.pgm'),dtype=np.int16)
        delta=np.abs(groove-flat)
        assert np.abs(control-flat).max()<=1, 'flat topology changes radiometry unexpectedly'
        assert delta.max()>10 and np.count_nonzero(delta>2)>100, 'grooves not optically visible'
        metrics.append(dict(samples=n,flat_control_max_dn=int(np.abs(control-flat).max()),changed_pixels=int(np.count_nonzero(delta>2)),max_dn=int(delta.max())))
    report['image_checks']=metrics
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    # Same true-scale crop around crack/joint crossing; no DN enhancement.
    panels=[]
    for name in ('plane','grooves'):
        im=Image.open(out/f'{name}/images/samples_16.pgm').crop((1792,3840,2304,4352)).convert('RGB')
        panel=Image.new('RGB',(512,540),'white');panel.paste(im,(0,28));ImageDraw.Draw(panel).text((8,8),name+' / 16 LED samples',(0,0,0));panels.append(panel)
    image=Image.new('RGB',(1024,540));image.paste(panels[0]);image.paste(panels[1],(512,0));image.save(out/'comparison.png')
    print(json.dumps({name:[dict(samples=m['samples'],lines_s=round(m['active_lines_per_second']),MiB=round(m['allocated_device_bytes']/2**20,2),p99_ms=round(m['p99_batch_ms'],2)) for m in data['timing']['measurements']] for name,data in report['fixtures'].items()}))
if __name__=='__main__':main()
