#!/usr/bin/env python3
"""Verify the flight recorder: it must fire, be complete, and not delay stopping.

The residual gate is lowered so the rejection is certain. This proves the recorder
works; it does not reproduce the real fault and is not evidence about its cause.
"""
import glob,json,os,shutil,signal,subprocess,sys,tempfile,time
import yaml

ROOT='/home/jerry/robot_ws/4WIDS_agv'
# headless + the render backend segfaults in Ogre on startup (already recorded);
# optix on a prepared scene is the path the real captures use.
SCENE=ROOT+'/local_data/rough_markings_3mm_v2/manifest.json'
SP=os.environ.get('AGV_FLIGHT_VERIFY_DIR',tempfile.mkdtemp(prefix='agv_flight_verify_'))
OUT=SP+'/flight_verify'
def log(*a):print(*a,flush=True)

shutil.rmtree(OUT,ignore_errors=True);os.makedirs(OUT)
cfg=yaml.safe_load(open(ROOT+'/src/agv_description/config/linescan.yaml'))
cfg.update(projected_encoder=True,max_scan_residual_m_s=1e-9,
           max_scan_lateral_m_s=.28,max_yaw_rate_rad_s=.08)
cfg['flight_recorder']={'span_s':2.0,'max_samples':4000,'half_limit_cooldown_s':10.0,'max_dumps':3}
cam=OUT+'/camera.yaml';yaml.safe_dump(cfg,open(cam,'w'))
log('gate lowered to',cfg['max_scan_residual_m_s'])

launch=subprocess.Popen(['bash','-c',
  'cd %s && source /opt/ros/jazzy/setup.bash && source install/setup.bash && '
  'exec python3 tools/with_mesa_runtime.py -- ros2 launch agv_bringup sim.launch.py '
  'headless:=true linescan:=true linescan_backend:=optix scene_manifest:=%s spawn_x:=2 '
  'camera_config:=%s capture_dir:=%s'%(ROOT,SCENE,cam,OUT)],
  stdout=open(OUT+'/sim.log','w'),stderr=subprocess.STDOUT,preexec_fn=os.setsid)
def dead():return launch.poll() is not None

end=time.time()+240;ready=False
while time.time()<end and not dead():
    try:
        if open(OUT+'/sim.log',errors='replace').read().count('Successfully switched controllers')>=3:ready=True;break
    except OSError:pass
    time.sleep(2)
if not ready:
    log('SIM FAILED',launch.poll());log(open(OUT+'/sim.log',errors='replace').read()[-1500:]);sys.exit(1)
log('controllers up')

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
rclpy.init()
class N(Node):
    def __init__(s):
        super().__init__('flight_verify',parameter_overrides=[Parameter('use_sim_time',value=True)])
        s.pub=s.create_publisher(TwistStamped,'/cmd_vel',10)
        s.cli=s.create_client(SetBool,'/linescan/set_enabled')
        s.x=None;s.speed=0.
        s.create_subscription(Odometry,'/ground_truth/odom',s.o,20)
    def o(s,m):s.x=m.pose.pose.position.x;s.speed=m.twist.twist.linear.x
    def drive(s,vx,seconds):
        end=time.time()+seconds
        while time.time()<end:
            m=TwistStamped();m.header.frame_id='base_link'
            m.header.stamp=s.get_clock().now().to_msg();m.twist.linear.x=float(vx)
            s.pub.publish(m);rclpy.spin_once(s,timeout_sec=.02);time.sleep(.02)
            if dead():raise SystemExit('sim died')
n=N()
t=time.time()
while n.x is None and time.time()-t<60:rclpy.spin_once(n,timeout_sec=.1)
log('odom up x=%.2f'%n.x)
if not n.cli.wait_for_service(timeout_sec=30):log('NO set_enabled SERVICE');sys.exit(1)
fut=n.cli.call_async(SetBool.Request(data=True))
while not fut.done():rclpy.spin_once(n,timeout_sec=.1)
log('capture enabled:',fut.result().success,fut.result().message)

def events():
    f=glob.glob(OUT+'/session_cpp_*/events.jsonl')
    if not f:return []
    return [json.loads(l) for l in open(f[0]) if l.strip()]

try:
    n.drive(1.0,6)
    fault_wall=None
    for _ in range(200):
        n.drive(1.0,.5)
        if any(e.get('reason')=='unsupported_scan_motion' for e in events()):
            fault_wall=time.time();log('fault seen at x=%.2f speed=%.2f'%(n.x,n.speed));break
    if fault_wall is None:log('NO FAULT TRIGGERED');sys.exit(1)
    # Stopping must not wait on the diagnostic write.
    n.drive(0.,0.2)
    stop_wall=None
    for _ in range(100):
        n.drive(0.,.2)
        if abs(n.speed)<.02:stop_wall=time.time();break
    log('stop latency after fault: %.2f s'%((stop_wall or time.time())-fault_wall))
finally:
    try:os.killpg(launch.pid,signal.SIGINT);launch.wait(timeout=40)
    except Exception:
        try:os.killpg(launch.pid,signal.SIGKILL)
        except OSError:pass
    files=glob.glob(OUT+'/session_cpp_*/flight_*.json')
    log('flight files:',files)
    if files:
        d=json.load(open(files[0]))
        log('schema',d['schema'],'reason',d['reason'],'samples',d['samples'],'window %.3f s'%d['window_s'])
        r=d['records'][-1]
        log('last record keys:',sorted(r))
        log('pass flags:',r['pass'])
        log('per-wheel residual:',[round(v,6) for v in r['wheel_residual_m_s']])
        log('suspension:',[round(v,5) for v in r['suspension_m']],'rate',[round(v,5) for v in r['suspension_rate_m_s']])
        log('body v:',[round(v,4) for v in r['body_velocity_m_s']],'yaw rate',round(r['body_yaw_rate_rad_s'],5))
        log('residual stats:',json.dumps(d['residual_statistics'])[:260])
        log('dump cap: files=%d (limit 3)'%len(files))
        ev=[json.loads(l) for l in open(glob.glob(OUT+'/session_cpp_*/events.jsonl')[0]) if l.strip()]
        sup=[e for e in ev if 'flight_recorder_suppressed' in e]
        log('suppressed reported in %d events, last=%s'%(len(sup),sup[-1].get('flight_recorder_suppressed') if sup else None))
    else:
        log('FAIL: no flight recorder file written')
