#!/usr/bin/env python3
"""Flat-road acceleration response; diagnostic truth never enters control."""
import argparse
import json
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def evaluate(output, platform, modes, repeats):
    import rclpy
    from scipy.spatial.transform import Rotation
    from validate_motion import Evaluator

    class Recorder(Evaluator):
        def __init__(self):
            self.rows = []
            self.joint_rows = []
            self.phase = 'startup'
            super().__init__()

        def on_odom(self, m):
            super().on_odom(m)
            q = m.pose.pose.orientation
            roll, pitch, yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            self.rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                              m.pose.pose.position.x, m.pose.pose.position.z,
                              m.twist.twist.linear.x, roll, pitch, yaw, self.phase])

        def on_joints(self, m):
            super().on_joints(m)
            pos = dict(zip(m.name, m.position))
            self.joint_rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                *[pos.get(c + '_suspension_joint', float('nan')) for c in ('fl','fr','rl','rr')],
                *[pos.get(c + '_steer_joint', float('nan')) for c in ('fl','fr','rl','rr')], self.phase])

        def phase_run(self, label, seconds, speed):
            self.phase = label
            start = self.get_clock().now().nanoseconds * 1e-9
            deadline = time.monotonic() + max(30, seconds * 5)
            while True:
                t = self.get_clock().now().nanoseconds * 1e-9 - start
                if t >= seconds:
                    return
                assert time.monotonic() < deadline, 'simulation stalled'
                self.command((speed(t) if callable(speed) else speed, 0, 0))
                rclpy.spin_once(self, timeout_sec=.01)

    rclpy.init()
    n = Recorder()
    try:
        deadline = time.monotonic() + 100
        while not (n.odom and n.joints and n.state):
            assert time.monotonic() < deadline, 'feedback missing'
            rclpy.spin_once(n, timeout_sec=.1)
        vmax = 10/3.6
        for mode in modes:
            for repeat in range(repeats):
                prefix = f'{mode}_{repeat}'
                n.phase_run(prefix+'_rest', 6, 0)
                assert n.state == 'HOLD'
                n.phase_run(prefix+'_accel', 6, (lambda t: vmax*t/6) if mode=='ramp6' else vmax)
                n.phase_run(prefix+'_cruise', 5, vmax)
                assert abs(n.odom.twist.twist.linear.x-vmax)<.03
                n.phase_run(prefix+'_brake', 6, (lambda t: vmax*(1-t/6)) if mode=='ramp6' else 0)
                n.phase_run(prefix+'_settle', 6, 0)
                assert n.state=='HOLD' and abs(n.odom.twist.twist.linear.x)<.001
        output.mkdir(parents=True, exist_ok=True)
        (output/'samples.json').write_text(json.dumps(dict(
            odom_columns=['time','x','z','vx','roll','pitch','yaw','phase'], odom=n.rows,
            joint_columns=['time','fl','fr','rl','rr','fl_steer','fr_steer','rl_steer','rr_steer','phase'],
            joints=n.joint_rows, final_state=n.state, modes=modes, repeats=repeats,
            configuration=__import__('yaml').safe_load(platform.read_text()),
            source_sha256=hashlib.sha256(Path('src/agv_description/urdf/agv.urdf.xacro').read_bytes()).hexdigest())))
        analyze(output, n.rows, n.joint_rows, n.state, modes, repeats, __import__('yaml').safe_load(platform.read_text()))
    finally:
        n.command((0,0,0)); n.destroy_node(); rclpy.shutdown()


def analyze(output, odom_rows, joint_rows, final_state, modes, repeats, config):
    import numpy as np
    vmax = 10/3.6
    report = dict(schema='agv.accel_pitch.v1', passed=True, final_state=final_state,
                  stop_definition='First forward velocity <= 0.02 m/s; settle until pitch stays within 0.01 degree of rest',
                  configuration=config,
                  scope='Headless flat road, specified suspension, actual tyre 0.40 m; no imaging or GUI acceptance. Joint spring forces are estimates, not measured tyre contact loads.', cases=[])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, len(modes), figsize=(6*len(modes),9), sharex='col', squeeze=False)
    for col, mode in enumerate(modes):
        for repeat in range(repeats):
            prefix=f'{mode}_{repeat}'
            rows=np.array([r[:-1] for r in odom_rows if r[-1].startswith(prefix)],float)
            phases=np.array([r[-1] for r in odom_rows if r[-1].startswith(prefix)])
            jr=np.array([r[:-1] for r in joint_rows if r[-1].startswith(prefix)],float)
            rest=rows[phases==prefix+'_rest'][-100:]
            reference=float(np.mean(rest[:,5]))
            pitch=np.degrees(rows[:,5]-reference)
            # Gradient of measured truth velocity, not the commanded slope.
            accel=np.gradient(rows[:,3],rows[:,0])
            from scipy.spatial.transform import Rotation
            rotation=Rotation.from_euler('xyz',rows[:,4:7]).as_matrix()
            def clearance(points):
                return (np.einsum('nij,kj->nki',rotation,np.array(points))[:,:,2]+rows[:,2,None]).min(axis=1)
            battery=clearance([[x,y,-.3984] for x in (-.36,.36) for y in (-.55,.55)])
            # Conservative LED bounding box; actual tilted lamp is inside this box.
            import yaml
            camera=yaml.safe_load(Path('src/agv_description/config/linescan.yaml').read_text())
            lx=camera['camera_x_m']+camera['led_forward_offset_m']
            led=clearance([[x,y,camera['led_height_m']-camera['base_nominal_height_m']-.03] for x in (lx-.03,lx+.03) for y in (-camera['led_length_m']/2,camera['led_length_m']/2)])
            wheel_gaps=[]
            for i,(x,y) in enumerate(((config['wheelbase']/2,config['track']/2),(config['wheelbase']/2,-config['track']/2),(-config['wheelbase']/2,config['track']/2),(-config['wheelbase']/2,-config['track']/2))):
                q=np.interp(rows[:,0],jr[:,0],jr[:,i+1])
                center=np.column_stack((np.full(len(rows),x),np.full(len(rows),y),q-.45))
                wz=(rotation@center[:,:,None])[:,2,0]+rows[:,2]
                axis_z=rotation[:,2,1]
                support=.2*np.sqrt(1-axis_z**2)+config['wheel_width']/2*np.abs(axis_z)
                wheel_gaps.append(wz-support)
            evaluation=rows[:,0]>=rest[0,0]
            # Exclude initial spawn settlement, retain all commanded motion.
            case=dict(name=prefix, static_pitch_deg=float(np.degrees(reference)), static_base_height_m=float(rest[:,2].mean()),
                      minimum_battery_clearance_m=float(battery[evaluation].min()),
                      minimum_led_bounding_box_clearance_m=float(led[evaluation].min()),
                      wheel_bottom_gap_range_m=[float(np.array(wheel_gaps)[:,evaluation].min()),float(np.array(wheel_gaps)[:,evaluation].max())],
                      max_suspension_abs_m=float(np.abs(jr[jr[:,0]>=rest[0,0],1:5]).max()), phases={})
            assert case['minimum_battery_clearance_m']>.15
            assert case['minimum_led_bounding_box_clearance_m']>.15
            assert case['max_suspension_abs_m']<config['suspension_travel']-.005
            assert case['wheel_bottom_gap_range_m'][1]<.002

            for phase in ('accel','cruise','brake','settle'):
                mask=phases==prefix+'_'+phase
                active=mask & (rows[:,3]>.3) & (rows[:,3]<vmax-.3)
                stable=active & (np.abs(accel)>.1)
                ids=np.flatnonzero(mask)
                jm=(jr[:,0]>=rows[ids[0],0]) & (jr[:,0]<=rows[ids[-1],0])
                assert jm.any() and mask.any() and np.isfinite(rows[mask]).all()
                case['phases'][phase]=dict(
                    measured_accel_median_m_s2=float(np.median(accel[stable])) if stable.any() else None,
                    pitch_offset_min_deg=float(pitch[mask].min()), pitch_offset_max_deg=float(pitch[mask].max()),
                    active_pitch_median_deg=float(np.median(pitch[stable])) if stable.any() else None,
                    z_range_mm=float(np.ptp(rows[mask,2])*1000),
                    suspension_min_mm=(jr[jm,1:5].min(axis=0)*1000).tolist(),
                    suspension_max_mm=(jr[jm,1:5].max(axis=0)*1000).tolist())
            # Operational stop threshold: first forward speed <= 20 mm/s.
            # Include subsequent undershoot and rebound; do not wait for near-exact zero.
            br=np.flatnonzero(phases==prefix+'_brake')[0]
            stopped=np.flatnonzero((np.arange(len(rows))>=br)&(rows[:,3]<=.02))
            stop=stopped[0]
            unsettled=np.flatnonzero((np.arange(len(rows))>=stop)&(np.abs(pitch)>.01))
            case['settle_after_vx_below_0_02_to_0_01deg_s']=float(rows[unsettled[-1]+1,0]-rows[stop,0]) if len(unsettled) and unsettled[-1]+1<len(rows) else (0. if not len(unsettled) else None)
            case['minimum_vx_during_braking_m_s']=float(rows[br:,3].min())
            case['max_actual_steer_deg']=float(np.degrees(np.abs(jr[:,5:9]).max()))
            if mode == 'controller_limit':
                assert abs(case['phases']['accel']['measured_accel_median_m_s2']-.8)<.02
                assert abs(case['phases']['brake']['measured_accel_median_m_s2']+1)<.02
            report['cases'].append(case)
            t=rows[:,0]-rows[0,0]
            axes[0,col].plot(t,rows[:,3],label=f'run {repeat+1}')
            axes[1,col].plot(t,pitch)
            axes[2,col].plot(jr[:,0]-rows[0,0],(jr[:,1:3].mean(axis=1)-jr[:,3:5].mean(axis=1))*1000)
        axes[0,col].set_title(mode)
        axes[0,col].legend()
        axes[2,col].set_xlabel('Simulation time (s)')
    for row,label in enumerate(('Speed (m/s)','Pitch relative to rest (deg)','Front - rear travel (mm)')):
        axes[row,0].set_ylabel(label)
        for ax in axes[row]: ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(output/'response.png',dpi=150); plt.close(fig)
    report['effective_platform_canonical_sha256']=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()
    captured=json.loads((output/'samples.json').read_text())
    report['captured_robot_source_sha256']=captured.get('source_sha256')
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS: acceleration/braking runs, final HOLD; '+str(output/'summary.json'))


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); p.add_argument('--evaluate',action='store_true'); p.add_argument('--analyze',action='store_true'); p.add_argument('--platform',type=Path,default=Path('src/agv_description/config/platform.yaml')); p.add_argument('--modes',nargs='+',choices=['ramp6','controller_limit'],default=['ramp6','controller_limit']); p.add_argument('--repeats',type=int,default=2); a=p.parse_args()
    if a.analyze:
        data=json.loads((a.output/'samples.json').read_text())
        if 'configuration' not in data:
            raise ValueError('Legacy samples lack the captured configuration; do not replay against a changed default platform')
        analyze(a.output,data['odom'],data['joints'],data['final_state'],data['modes'],data['repeats'],data['configuration']); return
    if a.evaluate: evaluate(a.output, a.platform, a.modes, a.repeats); return
    a.output.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ,ROS_DOMAIN_ID='96',GZ_PARTITION='agv_pitch_'+str(os.getpid()))
    with (a.output/'sim.log').open('w') as log:
        sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','headless:=true','rviz:=false','linescan:=false','actual_wheel_diameter:=0.40','platform:='+str(a.platform.resolve())],env=env,stdout=log,stderr=log,start_new_session=True)
        try:
            subprocess.run([sys.executable,__file__,'--evaluate','--output',str(a.output),'--platform',str(a.platform),'--repeats',str(a.repeats),'--modes',*a.modes],env=env,check=True,timeout=300)
        finally:
            if sim.poll() is None:
                os.killpg(sim.pid,signal.SIGINT)
                try: sim.wait(timeout=20)
                except subprocess.TimeoutExpired: os.killpg(sim.pid,signal.SIGKILL); sim.wait()
if __name__=='__main__': main()
