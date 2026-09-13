"""RViz/CLI task broker. Owns at most one executor, never publishes motion itself."""
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time
import uuid
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Time
from .planner import plan,Vehicle
from .preview import messages,coverage_messages
from .execution_process import ExecutionProcess
from .callback_trace import TracedExecutor
from .capture_audit import audit_capture
from .coverage import combine
from .road_display import road_messages
from .scene_bounds import bind_request,check_request_scene

TERMINAL={'ACQUIRED','COMPLETED','CANCELED','FAULT'}


def archived_through(path,until):
    """Has the navigation archive been flushed past this moment?"""
    try:
        with path.open('rb') as handle:
            handle.seek(0,2);handle.seek(max(0,handle.tell()-8192))
            lines=[line for line in handle.read().splitlines() if line.strip()]
    except OSError:return False
    # The writer is appending, so the final line can be a partial one.
    for line in reversed(lines):
        try:return json.loads(line)['time_s']>=until
        except (ValueError,KeyError):continue
    return False


def audit_when_archived(mission,navigation,output,platform,camera,uncertainty,until,timeout=15.):
    """Audit once navigation has been written through the mission's last moment.

    The mission's own files are on disk before it reports a terminal state, but
    navigation belongs to another process writing on its own flush interval.
    Reading early does not fail loudly: a trailing block falls outside the
    execution trace, is excluded as EXECUTION_TRACE_INCOMPLETE, and the report
    grows an unverified span that never happened -- with a rescan request for it.
    This runs on the audit pool, never on a callback thread.
    """
    deadline=time.monotonic()+timeout
    while until is not None and time.monotonic()<deadline:
        if archived_through(Path(navigation)/'navigation.jsonl',until):break
        time.sleep(.2)
    return audit_capture(mission,navigation,output,platform,camera,uncertainty)


def edited_request(template,fields):
    request=deepcopy(template)
    allowed={'mission_id','start_x','start_y','length','width','spacing','speed','error'}
    if set(fields)-allowed:raise ValueError('unknown editor field')
    for key,target in (('length','length_m'),('width','width_m')):
        if key in fields:request['region'][target]=fields[key]
    for i,key in enumerate(('start_x','start_y')):
        if key in fields:request['region']['start_xy_m'][i]=fields[key]
    for key,target in (('mission_id','mission_id'),('spacing','track_spacing_m'),('speed','scan_speed_m_s'),('error','coverage_error_m')):
        if key in fields:request[target]=fields[key]
    return request


class Operator(Node):
    def __init__(self,executor):
        super().__init__('mission_operator')
        self.executor=executor;self.child=None;self.last={};self.pending=None;self.job=None
        share=Path(get_package_share_directory('agv_mission'));desc=Path(get_package_share_directory('agv_description'))/'config'
        self.platform=yaml.safe_load((desc/'platform.yaml').read_text());self.camera=yaml.safe_load((desc/'linescan.yaml').read_text())
        self.vehicle=Vehicle.from_configs(self.platform,self.camera)
        self.request=yaml.safe_load((share/'config/rectangle_demo.yaml').read_text());self.request['road']['frame_id']='map'
        self.request['region'].update(start_xy_m=[6.,-1.],length_m=3.,width_m=2.)
        self.request['coverage_error_m']=.1;self.request['mission_id']='inspection'
        self.response_id='';self.preview=None;self.coverage=None;self.message='请设置区域并预览轨迹';self.ok=True
        self.last_image=None;self.block_keys=set();self.captured_rows=0;self.coverage_sent=None
        self.create_subscription(String,'/linescan/block_metadata',self.on_block,20)
        self.root=Path(self.declare_parameter('output_root','local_data/operator').value)
        self.navigation=Path(self.declare_parameter('navigation_dir','').value)
        road_path=self.declare_parameter('road_display','').value
        self.road=json.loads(Path(road_path).read_text()) if road_path else None
        if self.road:self.request=bind_request(self.request,self.road)
        self.pool=ThreadPoolExecutor(max_workers=1);self.ids=set();self.mode='';self.mode_wall=0.
        self.create_subscription(String,'/motion_state',self.motion,10)
        self.create_subscription(String,'/mission/status',self.execution_status,qos_profile_sensor_data)
        self.create_subscription(String,'/mission/operator/request',self.receive,10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(String,'/mission/operator/state',qos)
        self.road_pub=self.create_publisher(MarkerArray,'/mission/road/markers',qos)
        self.coverage_pub=self.create_publisher(MarkerArray,'/mission/coverage/markers',qos)
        self.path_pub=self.create_publisher(NavPath,'/mission/preview/base_path',qos)
        self.marker_pub=self.create_publisher(MarkerArray,'/mission/preview/markers',qos)
        self.control_clients={k:self.create_client(Trigger,'/mission/'+k) for k in ('start','pause','resume','cancel')}
        # Static displays use transient-local QoS; late RViz subscribers receive
        # the retained sample. Do not rebuild/publish the full road/plan in the
        # same runtime callback loop as the controller.
        if self.road:self.road_pub.publish(road_messages(self.road))
        self.callback_stats={}
        self.create_timer(.1,self.timed_tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_block(self,msg):
        if not self.child:return
        value=json.loads(msg.data);key=value['last']['time_s']
        self.last_image={k:value[k] for k in ('width','rows','end_reason')}
        if key not in self.block_keys:self.block_keys.add(key);self.captured_rows+=value['rows']
    def execution_status(self,msg):
        value=json.loads(msg.data)
        if self.child and self.child.receive(value):self.last=value
    def motion(self,m):self.mode=m.data;self.mode_wall=time.monotonic()
    def stopped(self):return self.mode=='HOLD' and time.monotonic()-self.mode_wall<.5
    def editable(self):
        return self.child is None or (self.child.state in TERMINAL|{'READY'} and self.stopped() and self.child.capture.active is False and not getattr(self.child.capture,'pending',False) and self.child.capture.future is None)
    def release(self):
        if self.child:
            if hasattr(self.executor,'trace'):
                (self.child.output.parent/'broker_callback_trace.json').write_text(json.dumps(self.executor.trace.snapshot(),indent=2)+'\n')
            self.child.destroy_node();self.child=None;self.last={}
    def receive(self,msg):
        try:
            command=json.loads(msg.data);identity=command['id'];action=command['action']
            if not isinstance(identity,str) or not identity:raise ValueError('request ID required')
            if identity in self.ids:return
            self.ids.add(identity);self.response_id=identity
            if len(self.ids)>2048:raise ValueError('request limit reached; restart idle operator')
            if self.pending or self.job:raise ValueError('上一操作尚未完成')
            if action in self.control_clients:
                if not self.child:raise ValueError('请先预览并准备任务')
                if action=='start' and self.count_publishers('/cmd_vel')!=1:raise ValueError('检测到其他速度发布者，不能开始')
                client=self.control_clients[action]
                if not client.service_is_ready():raise ValueError('执行器服务尚未就绪')
                self.pending=client.call_async(Trigger.Request());self.pending_wall=time.monotonic();self.message='正在处理：'+action;return
            if not self.editable():raise ValueError('运行中不能更换任务；请先取消并等待停车和归档')
            if action=='load':
                candidate=yaml.safe_load(Path(command['path']).read_text());plan(candidate,self.vehicle);check_request_scene(candidate,self.road)
                if candidate['road']['frame_id']!='map':raise ValueError('执行任务必须使用map坐标系')
                self.release();self.request=candidate;self.preview=None;self.clear_preview();self.message='已载入，请预览轨迹'
            elif action in ('preview','save','prepare'):
                candidate=edited_request(self.request,command.get('fields',{}));preview=plan(candidate,self.vehicle);check_request_scene(candidate,self.road)
                if action=='save':
                    with Path(command['path']).open('x') as f:yaml.safe_dump(candidate,f,sort_keys=False,allow_unicode=True)
                    self.message='请求已保存'
                elif action=='preview':
                    self.release();self.request=candidate;self.preview=preview;self.coverage=None
                    self.message=f"预览通过：{len(preview['tracks'])}道，行距{candidate['track_spacing_m']:.2f} m";self.show_preview()
                else:
                    if self.preview!=preview:raise ValueError('参数已更改，请重新预览')
                    if not self.stopped():raise ValueError('等待底盘HOLD')
                    self.release()
                    if self.count_publishers('/cmd_vel')!=0:raise ValueError('检测到其他速度发布者')
                    if not (self.navigation/'navigation.jsonl').is_file():raise ValueError('未配置有效导航归档目录；请用巡检启动入口')
                    folder=self.root/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);folder.mkdir(parents=True)
                    request_path=folder/'request.yaml';request_path.write_text(yaml.safe_dump(candidate))
                    self.child=ExecutionProcess(self,request_path,folder/'mission',self.get_parameter('use_sim_time').value)
                    self.last_image=None;self.block_keys=set();self.captured_rows=0;self.coverage=None;self.message='任务已准备；定位连续就绪后可开始'
            elif action=='audit':
                if not self.child or self.child.state not in TERMINAL:raise ValueError('请等待本任务结束或取消后再审计')
                directory=self.child.output;output=directory.parent/('audit_'+uuid.uuid4().hex[:6])
                self.job=self.pool.submit(audit_when_archived,directory,self.navigation,output,
                                          self.platform,self.camera,.1,self.child.snapshot.get('time_s'))
                self.message='正在核对原图、融合标签及覆盖范围';self.audit_output=output
            elif action=='load_coverage':
                coverage=json.loads(Path(command['path']).read_text())
                if coverage.get('schema')!='agv.mission.coverage.v1':raise ValueError('不是覆盖审计报告')
                coverage_messages(coverage,Time())
                self.coverage=coverage;self.message='已载入覆盖报告'
            elif action=='merge_coverage':
                if not self.coverage:raise ValueError('先载入父任务覆盖报告')
                child=json.loads(Path(command['path']).read_text());self.coverage=combine(self.coverage,[child]);self.message='补扫覆盖已合并'
                self.root.mkdir(parents=True,exist_ok=True)
                (self.root/('coverage_'+uuid.uuid4().hex[:8]+'.json')).write_text(json.dumps(self.coverage,indent=2)+'\n')
            elif action=='rescan':
                if not self.coverage:raise ValueError('先运行或载入覆盖审计')
                candidates=self.coverage.get('rescan_candidates',[]);index=int(command['index'])
                if index<0 or index>=len(candidates) or candidates[index]['status']!='PREVIEW_ONLY':raise ValueError('该补扫请求不可执行')
                candidate=deepcopy(candidates[index]['request']);preview=plan(candidate,self.vehicle);check_request_scene(candidate,self.road)
                self.release();self.request=candidate;self.preview=preview;self.show_preview();self.message='补扫已预览；准备后作为新任务执行'
            else:raise ValueError('unknown action')
            self.ok=True
        except Exception as exc:self.ok=False;self.message=str(exc)
    def clear_preview(self):
        from visualization_msgs.msg import Marker
        clear=Marker(action=Marker.DELETEALL);clear.header.frame_id='map'
        self.marker_pub.publish(MarkerArray(markers=[clear]));path=NavPath();path.header.frame_id='map';self.path_pub.publish(path)
    def show_preview(self):
        path,markers=messages(self.preview,Time());self.path_pub.publish(path);self.marker_pub.publish(markers)
    def timed_tick(self):
        start=time.monotonic()
        try:self.tick()
        finally:
            duration=time.monotonic()-start
            self.callback_stats={'last_tick_wall_s':duration,'max_tick_wall_s':max(duration,self.callback_stats.get('max_tick_wall_s',0.))}
    def tick(self):
        if self.child:self.child.poll()
        if self.child and self.child.state=='RUNNING' and self.count_publishers('/cmd_vel')!=1:
            self.child.fault('COMPETING_COMMAND_PUBLISHER')
        if self.pending and not self.pending.done() and time.monotonic()-self.pending_wall>3.:
            self.pending.cancel();self.pending=None;self.ok=False;self.message='执行器服务超时，任务已锁存故障'
            if self.child:self.child.fault('OPERATOR_SERVICE_TIMEOUT')
        if self.pending and self.pending.done():
            try:r=self.pending.result();self.ok=r.success;self.message=r.message
            except Exception as exc:self.ok=False;self.message=str(exc)
            self.pending=None
        if self.job and self.job.done():
            try:self.coverage=self.job.result();self.ok=True;self.message='覆盖审计完成，报告已保存到任务目录'
            except Exception as exc:self.ok=False;self.message=str(exc)
            self.job=None
        if self.coverage is not self.coverage_sent:
            if self.coverage:self.coverage_pub.publish(coverage_messages(self.coverage,Time()))
            else:
                from visualization_msgs.msg import Marker
                clear=Marker(action=Marker.DELETEALL);clear.header.frame_id='map';self.coverage_pub.publish(MarkerArray(markers=[clear]))
            self.coverage_sent=self.coverage
        status=dict(self.last)
        status.update(captured_blocks=len(self.block_keys),captured_rows=self.captured_rows,last_image=self.last_image)
        if self.child:
            status['state']=self.child.state
            status['reason']=self.child.reason
            status['ready_to_start']=self.child.state=='READY' and self.child.snapshot.get('ready_to_start',False)
            if self.child.failure:
                status.update(motion_state=self.mode,capture_active=self.child.capture.active,
                    capture_pending=getattr(self.child.capture,'pending',False) or self.child.capture.future is not None,capture_close_failed=self.child.capture.close_failed,
                    capture_sensor_enabled=False if self.child.capture.active is False else None)
        else:status={'state':'IDLE'}
        self.pub.publish(String(data=json.dumps(dict(max_requested_speed_m_s=self.platform['rated_scan_speed'],response_id=self.response_id,request=self.request,preview_valid=self.preview is not None,editable=self.editable(),busy=bool(self.pending or self.job),ok=self.ok,message=self.message,status=status,coverage=self.coverage))))
    def close(self):
        self.release();self.pool.shutdown(wait=True);self.destroy_node()


def main():
    rclpy.init();executor=TracedExecutor();node=Operator(executor);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        node.close();executor.shutdown();rclpy.try_shutdown()
