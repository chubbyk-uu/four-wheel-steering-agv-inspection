"""Own one isolated mission process; exchange state and commands only over ROS."""
import os,signal,subprocess,sys,time
from types import SimpleNamespace
from std_msgs.msg import String
from std_srvs.srv import SetBool


class ExecutionProcess:
    def __init__(self,node,request,output,use_sim_time):
        self.output=output;self.execution_id=output.parent.name;self.snapshot={}
        self.capture=SimpleNamespace(active=None,pending=False,future=None,close_failed=False)
        self.started=time.monotonic();self.failure=''
        self.fault_pub=node.create_publisher(String,'/mission/external_fault',10)
        self.close_client=node.create_client(SetBool,'/linescan/set_enabled')
        self.node=node;self.log=(output.parent/'executor.log').open('w')
        args=[sys.executable,'-c','from agv_mission.execution_node import main; main()',
              '--ros-args','-p','use_sim_time:='+str(use_sim_time).lower(),
              '-p','request:='+str(request),'-p','output_dir:='+str(output),'-p','capture:=true']
        try:self.process=subprocess.Popen(args,stdout=self.log,stderr=self.log,start_new_session=True)
        except Exception:
            self.log.close();node.destroy_publisher(self.fault_pub);node.destroy_client(self.close_client);raise

    @property
    def state(self):return 'FAULT' if self.failure else self.snapshot.get('state','STARTING')
    @property
    def reason(self):return self.failure or self.snapshot.get('reason','')

    def receive(self,status):
        if status.get('execution_id')!=self.execution_id:return False
        self.snapshot=status
        if not self.failure:
            self.capture.active=status.get('capture_active')
            # `capture_pending` belongs to the worker process. `future` is
            # reserved for the broker's own close-camera service call.
            self.capture.pending=bool(status.get('capture_pending',False))
        return True

    def fault(self,reason):
        import json
        self.fault_pub.publish(String(data=json.dumps(dict(execution_id=self.execution_id,reason=reason))))

    def poll(self):
        if self.process.poll() is not None and not self.failure:
            self.failure='EXECUTOR_PROCESS_EXITED';self.capture.active=None;self.capture.pending=False;self.capture.future=None
            self.started=time.monotonic()
        if not self.failure:return
        # The independent chassis command watchdog stops motion after process loss.
        # The broker separately closes the camera; do not declare editable before acknowledgement.
        if self.capture.future is not None and self.capture.future.done():
            try:
                if self.capture.future.result().success:self.capture.active=False
                else:self.capture.close_failed=True
            except Exception:self.capture.close_failed=True
            self.capture.future=None;self.capture.pending=False
        if self.capture.active is not False and time.monotonic()-self.started>10:
            if self.capture.future is not None:self.capture.future.cancel();self.capture.future=None
            self.capture.close_failed=True
        if self.capture.active is not False and self.capture.future is None and not self.capture.close_failed and self.close_client.service_is_ready():
            self.capture.future=self.close_client.call_async(SetBool.Request(data=False))

    def destroy_node(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM)
                try:self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait()
        self.log.close();self.node.destroy_publisher(self.fault_pub);self.node.destroy_client(self.close_client)
