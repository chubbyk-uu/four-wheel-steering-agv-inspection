#!/usr/bin/env python3
"""Capture the real SDF/Ogre2 road from above for material coordinate audits."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage
from validate_rectangle_execution import stop_tree as stop


def main():
    p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--allow-mismatch',action='store_true',help='save a failing baseline without rejecting it')
    a=p.parse_args()
    m=json.loads(a.manifest.read_text());world=ET.parse(a.manifest.parent/m['world'])
    w=world.getroot().find('world')
    w.append(ET.fromstring('<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'))
    w.append(ET.fromstring('''<model name="audit_camera"><static>true</static><pose>7.5 0 10 0 1.5707963267948966 0</pose><link name="link"><sensor name="camera" type="camera"><always_on>true</always_on><update_rate>2</update_rate><topic>/road_alignment</topic><camera><horizontal_fov>1.2</horizontal_fov><image><width>1400</width><height>1400</height><format>R8G8B8</format></image><clip><near>0.1</near><far>100</far></clip></camera></sensor></link></model>'''))
    os.environ.update(GZ_PARTITION='road_alignment_'+str(os.getpid()),ROS_DOMAIN_ID='73',
                      GALLIUM_DRIVER='d3d12',MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA')
    a.output.mkdir(parents=True,exist_ok=False);processes=[]
    with tempfile.TemporaryDirectory() as temp, (a.output/'render.log').open('w') as log:
        path=Path(temp)/'world.sdf';world.write(path)
        try:
            processes.append(subprocess.Popen(['gz','sim','-r','-s',str(path)],stdout=log,stderr=log,start_new_session=True))
            processes.append(subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
                '/road_alignment@sensor_msgs/msg/Image[gz.msgs.Image'],stdout=log,stderr=log,start_new_session=True))
            rclpy.init();node=Node('road_alignment_capture');messages=[]
            node.create_subscription(RosImage,'/road_alignment',lambda msg:messages.append(msg),qos_profile_sensor_data)
            deadline=time.monotonic()+90
            while len(messages)<3 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
            assert messages,'no Ogre2 road image'
            msg=messages[-1];assert msg.encoding=='rgb8',msg.encoding
            image=Image.frombytes('RGB',(msg.width,msg.height),bytes(msg.data),'raw','RGB',msg.step)
            image.save(a.output/'ogre_road.png')
            rgb=np.asarray(image).astype(float)
            row=rgb[msg.height//2]
            yellow=(row[:,0]>row[:,2]*2)&(row[:,1]>row[:,2]*1.8)&(row[:,0]>100)
            edges=np.diff(np.r_[False,yellow,False].astype(int))
            intervals=list(zip(np.where(edges==1)[0],np.where(edges==-1)[0]))
            scale=2*10*math.tan(.6)/msg.width
            centers=[-(float(lo+hi)/2-msg.width/2)*scale for lo,hi in intervals if hi-lo>=3]
            matched=len(centers)==2 and max(abs(x-y) for x,y in zip(sorted(centers),[-.15,.15]))<2*scale
            report=dict(passed=matched,yellow_centers_world_y_m=centers,meters_per_pixel=scale,
                        expected_yellow_centers_m=[-.15,.15],camera_world_xyz_m=[7.5,0,10])
            if m.get('inspection_paint'):
                from road_test_markings import masks
                xs=7.5+(msg.height/2-np.arange(msg.height)-.5)*scale
                ys=(msg.width/2-np.arange(msg.width)-.5)*scale
                yellow_mask,_=masks(xs,ys,m['inspection_paint']['polygons'])
                actual=(rgb[:,:,0]>rgb[:,:,2]*2)&(rgb[:,:,1]>rgb[:,:,2]*1.8)&(rgb[:,:,0]>100)
                roi=(xs[:,None]>8.5)&(xs[:,None]<12.5)&(ys[None,:]>-4.3)&(ys[None,:]<-.4)
                expected=yellow_mask.T&roi;actual&=roi
                iou=float((actual&expected).sum()/max(1,(actual|expected).sum()))
                report['box_mask_iou']=iou
                report['passed']=matched and iou>.93
                assert iou>.93 or a.allow_mismatch,'rendered box differs from metric paint polygons'
            (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
            node.destroy_node();rclpy.shutdown()
            assert matched or a.allow_mismatch, 'rendered yellow lines do not match shared road coordinates'
        finally:
            for process in processes:stop(process)


if __name__=='__main__':main()
