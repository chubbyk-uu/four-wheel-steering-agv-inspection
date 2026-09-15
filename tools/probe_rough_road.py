#!/usr/bin/env python3
"""Bounded real collision-road experiment. No suspension force/pose injection."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/agv_linescan'))

def height(x,y):
    rng=np.random.default_rng(20260915)
    wavelengths=np.geomspace(1.,6.,48)
    angles=rng.uniform(-np.pi,np.pi,48);phases=rng.uniform(0,2*np.pi,48)
    z=np.zeros(np.broadcast_shapes(np.shape(x),np.shape(y)))
    for w,a,p in zip(wavelengths,angles,phases):
        z+=np.sin(2*np.pi/w*(x*np.cos(a)+y*np.sin(a))+p)
    z*=.0003*np.sqrt(2/48)
    taper=np.sin(np.clip((np.asarray(x)-2)/3,0,1)*np.pi/2)**2
    return .001*np.tanh(z/.001)*taper

def generate(out,step):
    from agv_linescan.shared_scene import digest,validate
    assert np.isfinite(step) and (step==0 or .05<=step<=.5)
    out=out.resolve();out.mkdir(parents=True,exist_ok=True)
    if step==0:
        x,y=np.meshgrid([0,50],[-2,2]);z=np.zeros_like(x)
    else:
        x,y=np.meshgrid(np.linspace(0,50,round(50/step)+1),np.linspace(-2,2,round(4/step)+1));z=height(x,y)
    ny,nx=x.shape
    i,j=np.meshgrid(np.arange(ny-1),np.arange(nx-1),indexing='ij');a=(i*nx+j).ravel()
    faces=np.concatenate((np.stack((a,a+1,a+nx+1),1),np.stack((a,a+nx+1,a+nx),1)))+1
    mesh=out/'terrain.obj'
    with mesh.open('w') as f:
        np.savetxt(f,np.column_stack((x.ravel(),y.ravel(),z.ravel())),fmt='v %.9f %.9f %.9f')
        vertices=np.column_stack((x.ravel(),y.ravel(),z.ravel()))
        tri=vertices[faces-1].astype(float);normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normal/=np.linalg.norm(normal,axis=1)[:,None]
        assert np.all(normal[:,2]>0)
        np.savetxt(f,normal,fmt='vn %.9f %.9f %.9f')
        for n,face in enumerate(faces,1):f.write('f '+' '.join(f'{v}//{n}' for v in face)+'\n')
    tree=ET.parse(ROOT/'src/agv_bringup/worlds/flat.sdf');world=tree.getroot().find('world')
    for m in list(world.findall('model')):world.remove(m)
    m=ET.SubElement(world,'model',name='terrain');ET.SubElement(m,'static').text='true';link=ET.SubElement(m,'link',name='link')
    for kind in ('visual','collision'):
        item=ET.SubElement(link,kind,name=kind);g=ET.SubElement(ET.SubElement(item,'geometry'),'mesh');ET.SubElement(g,'uri').text=str(mesh);ET.SubElement(g,'scale').text='1 1 1'
        if kind=='visual':
            material=ET.SubElement(item,'material')
            for tag in ('ambient','diffuse'):ET.SubElement(material,tag).text='.5 .5 .5 1'
    tree.write(out/'world.sdf',encoding='unicode')
    manifest=dict(schema='agv.shared.static_scene.v1',units='m',frame='world',transform='identity_world_baked',profile='rough_road_probe',length_m=50,width_m=4,world='world.sdf',world_sha256=digest(out/'world.sdf'),display_materials={},assets=[dict(name='terrain',mesh=mesh.name,sha256=digest(mesh),triangles=len(faces),linear_reflectance=.5)])
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2));validate(out/'manifest.json')
    (out/'geometry.json').write_text(json.dumps(dict(step_m=step,triangles=len(faces),mesh_bytes=mesh.stat().st_size,height_rms_m=float(np.std(z)),height_min_m=float(z.min()),height_max_m=float(z.max()),seed=20260915,nominal_wavelength_m=[1,6],same_mesh_visual_collision_optix=True),indent=2))
    return out/'manifest.json'

def evaluate(out):
    import rclpy
    from scipy.spatial.transform import Rotation
    from scipy.signal import welch
    from validate_motion import Evaluator
    class Recorder(Evaluator):
        def __init__(self):self.rows=[];self.phase='rest';super().__init__()
        def on_odom(self,m):
            super().on_odom(m)
            q=m.pose.pose.orientation;r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')
            joints=dict(zip(self.joints.name,self.joints.position)) if self.joints else {}
            self.rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.pose.position.x,m.pose.pose.position.z,m.twist.twist.linear.x,*r,*[joints.get(c+'_suspension_joint',0) for c in ('fl','fr','rl','rr')],time.monotonic(),self.phase])
    rclpy.init();n=Recorder()
    try:
        deadline=time.monotonic()+100
        while not(n.odom and n.joints and n.state):
            assert time.monotonic()<deadline,'feedback missing';rclpy.spin_once(n,timeout_sec=.1)
        n.run_for(6,(0,0,0));n.phase='accel';n.run_for(7,(10/3.6,0,0))
        n.phase='cruise';n.run_for(8,(10/3.6,0,0));n.phase='brake';n.run_for(7,(0,0,0))
        assert n.state=='HOLD' and abs(n.odom.twist.twist.linear.x)<.001
        data=np.array([r[:-1] for r in n.rows if r[-1]=='cruise'],float)
        dt=np.diff(data[:,0]);assert len(data)>300 and dt.max()<.05
        result=dict(passed=True,final_state=n.state,samples=len(data),speed_m_s=float(np.median(data[:,3])),rtf=float((data[-1,0]-data[0,0])/(data[-1,11]-data[0,11])),signals={})
        for name,col,scale in [('z_mm',2,1000),('roll_deg',4,180/np.pi),('pitch_deg',5,180/np.pi)]:
            a=data[:,col]*scale;f,p=welch(a,fs=1/np.median(dt),nperseg=200)
            result['signals'][name]=dict(std=float(np.std(a)),peak_to_peak=float(np.ptp(a)),max_deviation_from_mean=float(np.max(abs(a-a.mean()))),dominant_hz=float(f[1:][np.argmax(p[1:])]))
        result['max_suspension_abs_m']=float(np.abs(data[:,7:11]).max());assert result['max_suspension_abs_m']<.045
        assert max(result['signals'][n]['max_deviation_from_mean'] for n in ('roll_deg','pitch_deg'))<.2
        result['odom_gap_max_s']=float(dt.max())
        (out/'samples.json').write_text(json.dumps(n.rows));(out/'summary.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result))
    finally:n.command((0,0,0));n.destroy_node();rclpy.shutdown()

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--step',type=float,default=.2);p.add_argument('--evaluate',action='store_true');p.add_argument('--generate-only',action='store_true');a=p.parse_args()
    if a.evaluate:evaluate(a.output);return
    manifest=generate(a.output,a.step)
    if a.generate_only:print(manifest);return
    env=dict(os.environ,ROS_DOMAIN_ID='98',GZ_PARTITION='rough_'+str(os.getpid()))
    with (a.output/'sim.log').open('w') as log:
        sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','headless:=true','rviz:=false','linescan:=false','scene_manifest:='+str(manifest),'spawn_x:=2','spawn_y:=0'],env=env,stdout=log,stderr=log,start_new_session=True)
        try:subprocess.run([sys.executable,__file__,'--evaluate','--output',str(a.output)],env=env,check=True,timeout=240)
        finally:
            if sim.poll() is None:
                os.killpg(sim.pid,signal.SIGINT)
                try:sim.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
if __name__=='__main__':main()
