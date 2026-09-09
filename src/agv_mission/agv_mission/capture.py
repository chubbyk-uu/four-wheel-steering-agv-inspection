"""Asynchronous per-pass capture handshake and traceable capture intervals."""
import json
import time
import math
from std_msgs.msg import String
from std_srvs.srv import SetBool


class CaptureGate:
    def __init__(self,node,output,enabled,state_timeout=.5):
        self.node=node;self.enabled=enabled;self.active=None if enabled else False;self.future=None;self.target=False
        self.track=None;self.intervals=[];self.error='';self.started_wall=0;self.close_reason='requested'
        self.close_failed=False
        self.state_timeout=state_timeout;self.heartbeat=None;self.heartbeat_wall=0;self.enable_wall=0;self.enable_time=0
        self.client=node.create_client(SetBool,'/linescan/set_enabled') if enabled else None
        self.output=output
        self.events=(output/'capture_events.jsonl').open('x')
        self.blocks=(output/'capture_blocks.jsonl').open('x')
        node.create_subscription(String,'/linescan/status',self.event,100)
        node.create_subscription(String,'/linescan/state',self.on_state,20)
        node.create_subscription(String,'/linescan/block_metadata',self.block,100)
    def now(self):return self.node.get_clock().now().nanoseconds*1e-9
    def on_state(self,msg):
        if not self.enabled or self.now()<=0:return
        try:
            value=json.loads(msg.data)
            if not isinstance(value['enabled'],bool) or not math.isfinite(value['time_s']) or value['time_s']>self.now()+.1:
                raise ValueError('invalid camera state')
            if self.now()-value['time_s']>self.state_timeout:return
            if self.heartbeat and value['time_s']<=self.heartbeat['time_s']:return
            self.heartbeat=value;self.heartbeat_wall=time.monotonic()
        except (ValueError,KeyError,TypeError):self.error='CAMERA_INVALID_STATE'
    def check_state(self):
        if not self.enabled or self.active is not True or self.future is not None:return
        wall=time.monotonic()
        if self.heartbeat and self.heartbeat['time_s']>self.enable_time:
            if not self.heartbeat['enabled']:self.error='CAMERA_DISABLED_UNEXPECTEDLY'
        if wall-max(self.enable_wall,self.heartbeat_wall)>self.state_timeout:
            self.error='CAMERA_STATE_TIMEOUT'
    def event(self,msg):
        if not self.enabled:return
        value=json.loads(msg.data);value['received_time_s']=self.now()
        self.events.write(json.dumps(value)+'\n');self.events.flush()
        # All interruptions are archived. Spatial coverage is checked separately;
        # end-of-pass braking outside the ROI is not mislabeled as a missing strip.
        reason=value.get('reason','')
        expected=('capture_toggle','sampler_recovery','tail_discarded')
        # A sensor segment break must never silently turn into ACQUIRED.
        if reason and reason not in expected:
            self.error='CAMERA_'+value['reason']
    def block(self,msg):self.blocks.write(msg.data+'\n');self.blocks.flush()
    def request(self,value,track=None,reason='requested'):
        if not self.enabled:return True
        if self.future is not None:return False
        if not value and self.close_failed:return False
        if self.active==value:return True
        if not self.client.service_is_ready():self.error='CAMERA_SERVICE_UNAVAILABLE';return False
        self.target=value;self.started_wall=time.monotonic()
        if value:self.track=track
        else:self.close_reason=reason
        self.future=self.client.call_async(SetBool.Request(data=value));return False
    def poll(self):
        self.check_state()
        if self.future is None:return
        if not self.future.done():
            if time.monotonic()-self.started_wall>10:self.error='CAMERA_HANDSHAKE_TIMEOUT'
            return
        try:response=self.future.result()
        except Exception as exc:
            self.future=None;self.close_failed=not self.target;self.error='CAMERA_HANDSHAKE_FAILED: '+str(exc);return
        self.future=None
        if not response.success:
            self.close_failed=not self.target;self.error='CAMERA_HANDSHAKE_FAILED: '+response.message;return
        self.active=self.target
        if self.active:
            self.enable_wall=time.monotonic();self.enable_time=self.now()
            self.intervals.append({'track_id':self.track,'enabled_ack_time_s':self.now(),'archive':response.message})
        elif self.intervals:self.intervals[-1].update(disabled_ack_time_s=self.now(),end_reason=self.close_reason)
        (self.output/'capture_intervals.json').write_text(json.dumps(self.intervals,indent=2)+'\n')
    def close(self):self.events.close();self.blocks.close()
