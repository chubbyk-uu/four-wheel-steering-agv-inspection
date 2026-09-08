#!/usr/bin/env python3
"""Production correction node throughput from recorded raw blocks; no live renderer."""
import argparse,copy,hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/agv_linescan'))
from agv_linescan.calibration import Correction


def digest(data):return hashlib.sha256(data).hexdigest()


def durable(path,data):
    with path.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())


def run(a):
    import rclpy
    from sensor_msgs.msg import Image as RosImage
    from std_msgs.msg import String
    source=Path(a.archive).resolve();out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False);(out/'raw').mkdir()
    config=source/'calibration.yaml';correction=Correction(json.loads(Path(a.profile).read_text()));correction.check_capture(yaml.safe_load(config.read_text()))
    blocks=[];oracles=[]
    for path in sorted(source.glob('block_*.json')):
        meta=json.loads(path.read_text())
        if meta['rows']!=4096:continue
        correction.metadata(meta)
        raw=np.array(Image.open(path.with_suffix('.pgm')));fixed,_=correction.apply(raw)
        blocks.append((path.with_suffix('.pgm'),meta));oracles.append((digest(raw),digest(fixed)))
    if len(blocks)<2:raise ValueError('Need at least two full source blocks')
    rclpy.init();node=rclpy.create_node('correction_replay_verifier')
    ipub=node.create_publisher(RosImage,'/linescan/image_raw',2);mpub=node.create_publisher(String,'/linescan/block_metadata',2)
    status={};pending={};seen=set();error=[];published=[]
    def image_cb(m):
        try:
            assert pending and m.header.stamp.sec*10**9+m.header.stamp.nanosec==pending['stamp']
            assert (m.width,m.height,m.step,m.encoding)==(4096,4096,4096,'mono8')
            assert digest(m.data)==pending['corrected_hash']
            assert digest(np.array(Image.open(out/'corrected'/pending['name'].replace('.json','.pgm'))))==pending['corrected_hash']
            seen.add('image')
        except Exception as e:error.append(str(e) or 'corrected image mismatch')
    def meta_cb(m):
        try:
            v=json.loads(m.data);expected=pending['metadata'];on_disk=json.loads((out/'corrected'/pending['name']).read_text())
            assert v==on_disk
            assert all(v[k]==expected[k] for k in ('block_id','segment_id','rows','first','last','pose_tags','reference'))
            assert v['source_pixel_sha256']==pending['raw_hash'] and v['corrected_pixel_sha256']==pending['corrected_hash']
            assert v['corrected_image_fsync'] and v['rows_preserved'];seen.add('metadata')
        except Exception as e:error.append(str(e) or 'corrected metadata mismatch')
    node.create_subscription(RosImage,'/linescan/image_corrected',image_cb,2)
    node.create_subscription(String,'/linescan/corrected_metadata',meta_cb,2)
    node.create_subscription(String,'/linescan/correction_status',lambda m:status.update(json.loads(m.data)),10)
    def spin_until(condition,timeout):
        end=time.monotonic()+timeout
        while not condition():
            rclpy.spin_once(node,timeout_sec=.002)
            if error or status.get('error'):raise RuntimeError(error or status)
            if time.monotonic()>end:raise TimeoutError('correction replay delivery timeout')
    log=(out/'node.log').open('w');proc=subprocess.Popen(['ros2','run','agv_linescan','linescan_correction','--ros-args',
        '-p','profile:='+str(Path(a.profile).resolve()),'-p','capture_config:='+str(config),'-p','output_dir:='+str(out/'corrected')],stdout=log,stderr=log,start_new_session=True)
    try:
        spin_until(lambda:status.get('ready') and ipub.get_subscription_count()>0 and mpub.get_subscription_count()>0,30)
        start=time.monotonic();wait=0.;latencies=[];rows=0
        first_line=blocks[0][1]['first']['global_line'];first_time=blocks[0][1]['first']['time_s']
        cycle_duration=blocks[-1][1]['last']['time_s']-first_time+1.
        for cycle in range(a.cycles):
            for j,(path,original) in enumerate(blocks):
                block=cycle*len(blocks)+j;meta=copy.deepcopy(original);meta['block_id']=block;meta['segment_id']=cycle
                meta['replay_scope']='recorded path repeated; arrival paced independently of original timestamps'
                for tag in [meta['first'],meta['last'],*meta['pose_tags']]:
                    tag['global_line']+=cycle*len(blocks)*4096-first_line;tag['time_s']+=cycle*cycle_duration
                raw=np.array(Image.open(path));raw_bytes=raw.tobytes();name=f'block_{block:06d}'
                durable(out/'raw'/(name+'.pgm'),b'P5\n4096 4096\n255\n'+raw_bytes)
                durable(out/'raw'/(name+'.json'),json.dumps(meta).encode()+b'\n')
                pending.clear();seen.clear();stamp=round(meta['last']['time_s']*1e9)
                pending.update(stamp=stamp,metadata=meta,name=name+'.json',raw_hash=oracles[j][0],corrected_hash=oracles[j][1])
                message=RosImage();message.header.stamp.sec=stamp//10**9;message.header.stamp.nanosec=stamp%10**9
                message.header.frame_id='camera_optical_frame';message.width=4096;message.height=4096;message.step=4096;message.encoding='mono8';message.data=raw_bytes
                t=time.monotonic();ipub.publish(message);mpub.publish(String(data=json.dumps(meta)))
                spin_until(lambda:seen=={'image','metadata'},10);latencies.append(time.monotonic()-t);rows+=4096;published.append(time.monotonic()-start)
                due=start+rows/a.rate;t=time.monotonic()
                while time.monotonic()<due:rclpy.spin_once(node,timeout_sec=max(0.,min(.002,due-time.monotonic())))
                wait+=time.monotonic()-t
        elapsed=time.monotonic()-start
        spin_until(lambda:status.get('blocks',0)==len(published),3)
        report=dict(scope='Recorded concrete raw image replay through production correction node; source read, raw image/metadata fsync, ROS delivery, correction, corrected image/metadata fsync, independent pixel and metadata verification. No OptiX generation, GZ physics, GUI, stitching or directory fsync in this benchmark.',
            lines=rows,blocks=len(published),cycles=a.cycles,source_full_blocks=len(blocks),wall_seconds=elapsed,
            effective_lines_per_second=rows/elapsed,active_lines_per_second=rows/(elapsed-wait),target_hz=a.rate,
            receive_latency_seconds_max=max(latencies),block_interval_seconds_max=float(np.diff(published).max()),
            correction=status,all_corrected_pixels_match_offline=True,all_metadata_preserved=True,
            full_pipeline_11khz_passed=False,correction_replay_11khz_passed=rows/elapsed>=11000 and elapsed>=30)
        (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
        assert report['correction_replay_11khz_passed'],report
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close();node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser();p.add_argument('--archive',required=True);p.add_argument('--profile',required=True);p.add_argument('--output',required=True)
    p.add_argument('--cycles',type=int,default=3);p.add_argument('--rate',type=float,default=11200);p.add_argument('--domain',type=int,default=95);a=p.parse_args()
    if not 1<=a.cycles<=10 or not 11000<=a.rate<=12000:raise ValueError('unsupported benchmark budget')
    os.environ['ROS_DOMAIN_ID']=str(a.domain)
    if not any(os.environ.get(k) for k in ('FASTDDS_DEFAULT_PROFILES_FILE','FASTRTPS_DEFAULT_PROFILES_FILE')):
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(ROOT/'src/agv_bringup/config/fastdds_linescan.xml')
    run(a)


if __name__=='__main__':main()
