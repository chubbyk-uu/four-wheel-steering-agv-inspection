"""Bounded whole-block correction, preserving acquisition timestamps and tags."""
import hashlib
import json
import os
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from agv_linescan.calibration import Correction


class CorrectionNode(Node):
    def __init__(self):
        super().__init__('linescan_correction')
        profile=Path(self.declare_parameter('profile','').value)
        config=Path(self.declare_parameter('capture_config','').value)
        self.output=Path(self.declare_parameter('output_dir','/tmp/agv_corrected').value)
        self.correction=Correction(json.loads(profile.read_text()))
        self.correction.check_capture(yaml.safe_load(config.read_text()))
        self.output.mkdir(parents=True,exist_ok=False)
        (self.output/'calibration.json').write_text(profile.read_text())
        self.images={};self.metadata={};self.arrival={};self.next_block=0;self.last_line=None
        self.last_segment=None;self.last_stamp=-1;self.error='';self.rows=0;self.peak=0;self.max_seconds=0.;self.max_latency=0.
        self.pub=self.create_publisher(Image,'/linescan/image_corrected',2)
        self.meta_pub=self.create_publisher(String,'/linescan/corrected_metadata',10)
        self.status_pub=self.create_publisher(String,'/linescan/correction_status',10)
        self.create_subscription(Image,'/linescan/image_raw',lambda m:self.receive('image',m),4)
        self.create_subscription(String,'/linescan/block_metadata',lambda m:self.receive('metadata',m),4)
        self.create_timer(.02,self.process)
        self.create_timer(1.,self.report)

    def fail(self,error):
        if not self.error:
            self.error=str(error);self.get_logger().error(self.error)
            self.images.clear();self.metadata.clear();self.arrival.clear();self.report()

    def receive(self,kind,message):
        if self.error:return
        try:
            if kind=='image':
                stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
                target=self.images;value=message
            else:
                value=json.loads(message.data);stamp=round(value['last']['time_s']*10**9);target=self.metadata
            if stamp<=self.last_stamp or stamp in target:raise ValueError('duplicate/out-of-order correction input')
            target[stamp]=value;self.arrival.setdefault(stamp,time.monotonic())
            self.peak=max(self.peak,len(self.arrival))
            if len(self.arrival)>4:raise RuntimeError('bounded correction queue overflow')
        except Exception as e:self.fail(e)

    def process(self):
        if self.error or not self.arrival:return
        try:
            stamp=min(self.arrival)
            if time.monotonic()-self.arrival[stamp]>5:raise TimeoutError('image/metadata pairing or processing timeout')
            if stamp not in self.images or stamp not in self.metadata:return
            start=time.monotonic();message=self.images[stamp];meta=self.metadata[stamp]
            # The sensor has two asynchronous archive writers. A short final
            # block can finish before its preceding full block; wait within the
            # same bounded queue/deadline instead of misreporting it as lost.
            if meta['block_id']>self.next_block:return
            if meta['block_id']!=self.next_block:raise ValueError('duplicate/out-of-order block')
            if self.last_segment==meta['segment_id'] and meta['first']['global_line']!=self.last_line+1:
                raise ValueError('line discontinuity within scan segment')
            if (message.encoding!='mono8' or message.width!=self.correction.width or message.step!=message.width or
                    len(message.data)!=message.width*message.height or message.height!=meta['rows']):
                raise ValueError('incompatible image payload')
            result_meta=self.correction.metadata(meta)
            raw=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.width)
            pixels,quality=self.correction.apply(raw);result_meta.update(quality)
            result_meta['source_pixel_sha256']=hashlib.sha256(message.data).hexdigest()
            result_meta['corrected_pixel_sha256']=hashlib.sha256(pixels).hexdigest()
            path=self.output/f'block_{self.next_block:06d}.pgm'
            with path.open('xb') as f:
                f.write(f'P5\n{message.width} {message.height}\n255\n'.encode());f.write(pixels.tobytes());f.flush();os.fsync(f.fileno())
            result_meta['corrected_image_fsync']=True
            with path.with_suffix('.json').open('x') as f:
                json.dump(result_meta,f);f.write('\n');f.flush();os.fsync(f.fileno())
            corrected=Image();corrected.header=message.header;corrected.width=message.width;corrected.height=message.height
            corrected.encoding='mono8';corrected.step=message.width;corrected.data=pixels.tobytes()
            self.pub.publish(corrected);self.meta_pub.publish(String(data=json.dumps(result_meta)))
            self.max_seconds=max(self.max_seconds,time.monotonic()-start)
            self.max_latency=max(self.max_latency,time.monotonic()-self.arrival[stamp])
            self.rows+=message.height;self.next_block+=1;self.last_line=meta['last']['global_line']
            self.last_segment=meta['segment_id'];self.last_stamp=stamp
            del self.images[stamp],self.metadata[stamp],self.arrival[stamp]
        except Exception as e:self.fail(e)

    def report(self):
        report=dict(ready=not self.error,error=self.error,blocks=self.next_block,rows=self.rows,
            queue_blocks=len(self.arrival),queue_peak=self.peak,max_correction_write_publish_seconds=self.max_seconds,
            max_receive_to_publish_seconds=self.max_latency,corrected_image_and_metadata_fsync=True,
            calibration_id=self.correction.profile['calibration_id'])
        (self.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        self.status_pub.publish(String(data=json.dumps(report)))


def main():
    rclpy.init();node=CorrectionNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.report();node.destroy_node();rclpy.try_shutdown()
