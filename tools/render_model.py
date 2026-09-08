#!/usr/bin/env python3
"""Render a stationary model with Gazebo Ogre2; no AI image or ground texture substitute."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import xacro
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from PIL import Image as PILImage


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output', default='docs/images/agv_stage1.png')
    parser.add_argument('--led-off', action='store_true', help='Render a comparison without the attached LED lights')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='agv_render_') as temp:
        temp=Path(temp)
        urdf=temp/'agv.urdf'
        urdf.write_text(xacro.process_file(str(root/'src/agv_description/urdf/agv.urdf.xacro')).toxml())
        converted=subprocess.run(['gz','sdf','-p',str(urdf)],capture_output=True,text=True,check=True)
        model=ET.fromstring(converted.stdout).find('model')
        for plugin in list(model.findall('plugin')):model.remove(plugin)
        if args.led_off:
            for link in model.findall('link'):
                for light in list(link.findall('light')):
                    if light.get('name', '').startswith('led_strip_'): link.remove(light)
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text='0 0 0.65 0 0 0'
        world=ET.parse(root/'src/agv_bringup/worlds/flat.sdf').getroot()
        w=world.find('world');w.append(model)
        w.append(ET.fromstring('''<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'''))
        w.append(ET.fromstring('''<model name="observer"><static>true</static><pose>3.2 -4.2 2.7 0 0.36 2.17</pose><link name="link"><sensor name="camera" type="camera"><always_on>true</always_on><update_rate>2</update_rate><topic>/model_preview</topic><camera><horizontal_fov>0.90</horizontal_fov><image><width>1280</width><height>960</height><format>R8G8B8</format></image><clip><near>0.05</near><far>100</far></clip></camera></sensor></link></model>'''))
        path=temp/'render.sdf';ET.ElementTree(world).write(path)
        os.environ['GZ_PARTITION']='agv_render_'+str(os.getpid())
        os.environ['ROS_DOMAIN_ID']='74'
        os.environ['ROS_LOG_DIR']=str(temp/'roslogs')
        processes=[]
        log=open('/tmp/agv_render.log','w')
        try:
            processes.append(subprocess.Popen(['gz','sim','-r','-s','--headless-rendering',str(path)],stdout=log,stderr=log,start_new_session=True))
            processes.append(subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','/model_preview@sensor_msgs/msg/Image[gz.msgs.Image'],stdout=log,stderr=log,start_new_session=True))
            rclpy.init();node=Node('model_snapshot');messages=[]
            node.create_subscription(Image,'/model_preview',messages.append,qos_profile_sensor_data)
            deadline=time.monotonic()+60
            while len(messages)<3 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
            if not messages:raise RuntimeError('No rendered frame; inspect /tmp/agv_render.log')
            m=messages[-1]
            if m.encoding!='rgb8':raise RuntimeError(m.encoding)
            img=PILImage.frombytes('RGB',(m.width,m.height),bytes(m.data),'raw','RGB',m.step)
            img.save(args.output)
            node.destroy_node();rclpy.shutdown()
            print('Saved Gazebo rendering:',args.output)
        finally:
            for p in processes:
                if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
            for p in processes:
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            log.close()
if __name__=='__main__':main()
