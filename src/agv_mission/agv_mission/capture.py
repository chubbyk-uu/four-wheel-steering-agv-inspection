"""Asynchronous per-pass capture handshake and traceable capture intervals."""
import json
import time
from std_msgs.msg import String
from std_srvs.srv import SetBool


class CaptureGate:
    def __init__(self,node,output,enabled):
        self.node=node;self.enabled=enabled;self.active=None if enabled else False;self.future=None;self.target=False
        self.track=None;self.intervals=[];self.error='';self.started_wall=0
        self.client=node.create_client(SetBool,'/linescan/set_enabled') if enabled else None
        self.output=output
        self.events=(output/'capture_events.jsonl').open('x')
        self.blocks=(output/'capture_blocks.jsonl').open('x')
        node.create_subscription(String,'/linescan/status',self.event,100)
        node.create_subscription(String,'/linescan/block_metadata',self.block,100)
    def now(self):return self.node.get_clock().now().nanoseconds*1e-9
    def event(self,msg):
        if not self.enabled:return
        value=json.loads(msg.data);value['received_time_s']=self.now()
        self.events.write(json.dumps(value)+'\n');self.events.flush()
        # All interruptions are archived. Spatial coverage is checked separately;
        # end-of-pass braking outside the ROI is not mislabeled as a missing strip.
        if value.get('reason','').startswith('terrain_sampling_failure') or value.get('reason') in ('pose_gap','pose_outside_history','time_reset'):
            self.error='CAMERA_'+value['reason']
    def block(self,msg):self.blocks.write(msg.data+'\n');self.blocks.flush()
    def request(self,value,track=None):
        if not self.enabled:return True
        if self.future is not None:return False
        if self.active==value:return True
        if not self.client.service_is_ready():self.error='CAMERA_SERVICE_UNAVAILABLE';return False
        self.target=value;self.started_wall=time.monotonic()
        if value:self.track=track
        self.future=self.client.call_async(SetBool.Request(data=value));return False
    def poll(self):
        if self.future is None:return
        if not self.future.done():
            if time.monotonic()-self.started_wall>10:self.error='CAMERA_HANDSHAKE_TIMEOUT'
            return
        response=self.future.result();self.future=None
        if not response.success:self.error='CAMERA_HANDSHAKE_FAILED: '+response.message;return
        self.active=self.target
        if self.active:self.intervals.append({'track_id':self.track,'enabled_ack_time_s':self.now(),'archive':response.message})
        elif self.intervals:self.intervals[-1]['disabled_ack_time_s']=self.now()
        (self.output/'capture_intervals.json').write_text(json.dumps(self.intervals,indent=2)+'\n')
    def close(self):self.events.close();self.blocks.close()
