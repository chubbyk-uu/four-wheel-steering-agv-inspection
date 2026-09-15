#!/usr/bin/env python3
"""Compare the measured flat/coarse/fine probes without re-running simulation."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.signal import welch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from probe_rough_road import height

p=argparse.ArgumentParser();p.add_argument('--input',type=Path,default=Path('local_data/rough_road_probe'));p.add_argument('--output',type=Path,required=True);a=p.parse_args()
report=dict(schema='agv.rough_road_probe.v1',scope='50x4 m untextured same-mesh terrain; 550 kg current soft suspension, rated 10 km/h, 8 s steady samples, one run per resolution. Not ISO classification or 100 m production acceptance.',cases={})
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
for name in ('flat_valid','coarse','fine'):
 d=a.input/name;r=json.loads((d/'summary.json').read_text());r['geometry']=json.loads((d/'geometry.json').read_text())
 raw=json.loads((d/'samples.json').read_text());data=np.array([row[:-1] for row in raw if row[-1]=='cruise'],float)
 for ax,(label,col,scale) in zip(axes,(('Height (mm)',2,1000),('Roll (deg)',4,180/np.pi),('Pitch (deg)',5,180/np.pi))):
  signal=(data[:,col]-data[:,col].mean())*scale;ax.plot(data[:,1],signal,label=name);ax.set_ylabel(label);ax.grid(alpha=.3)
  f,power=welch(signal,fs=50,nperseg=200)
  r.setdefault('power_fraction_above_8hz',{})[label]=float(power[f>8].sum()/max(power.sum(),1e-40))
 report['cases'][name]=r
axes[0].legend();axes[-1].set_xlabel('Road X (m)');fig.tight_layout();fig.savefig(a.input/'comparison.png',dpi=150);plt.close(fig)
rng=np.random.default_rng(88);x=rng.uniform(5,45,20000);y=rng.uniform(-1.5,1.5,20000);truth=height(x,y)
report['geometric_interpolation_error']={}
for step in (.2,.1):
 x0=np.floor(x/step)*step;y0=np.floor(y/step)*step;u=(x-x0)/step;v=(y-y0)/step
 h00=height(x0,y0);h10=height(x0+step,y0);h11=height(x0+step,y0+step);h01=height(x0,y0+step)
 h=np.where(v<=u,h00*(1-u)+h10*(u-v)+h11*v,h00*(1-v)+h11*u+h01*(v-u))
 report['geometric_interpolation_error'][str(step)]=dict(rms_mm=float(np.sqrt(np.mean((h-truth)**2))*1000),p99_mm=float(np.quantile(abs(h-truth),.99)*1000))
# Same random field sampled at two mesh resolutions; coarse vertices must nest.
xx,yy=np.meshgrid(np.arange(0,50.01,.2),np.arange(-2,2.01,.2))
assert np.isfinite(height(xx,yy)).all() and np.max(abs(height(xx,yy)))<=.001
for key in ('z_mm','roll_deg','pitch_deg'):
 coarse=report['cases']['coarse']['signals'][key]['std'];fine=report['cases']['fine']['signals'][key]['std']
 assert abs(coarse/fine-1)<.15,(key,coarse,fine)
report['rms_resolution_difference_below_15_percent']=True
report['limitations']=['RTF is capped near 1; this does not quantify spare CPU/GPU capacity','50 Hz truth and 8 s windows do not exclude >25 Hz modes or rare peaks','No tyre compliance, contact-force measurement, surface texture or flexible camera mount','Initial unnormaled OBJ was rejected by DART and caused startup failure; corrected face normals are now explicit']
report['capture_checks']={}
for name in ('coarse','fine'):
 path=a.input/('capture_'+name+'.json')
 if not path.exists():continue
 r=json.loads(path.read_text())
 report['capture_checks'][name]={k:r[k] for k in ('passed','received_blocks','rows','real_time_factor','ros_all_blocks_byte_identical','final_motion_state','display_sync','sampling_queue_high_water_lines','realtime_required','realtime_target_met','realtime_floor')}
 report['capture_checks'][name]['scope']='GZ GUI + RViz + OptiX, uniform material, 10 km/h, 7 s warmup and commanded 10 m plus braking. Automated GUI checks only.'
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print('PASS: coarse/fine steady RMS differences <15%; '+str(a.output))
