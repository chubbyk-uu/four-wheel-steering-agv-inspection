#!/usr/bin/env python3
"""Compare same-field 10 cm mesh amplitude experiments and optional capture runs."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--input',type=Path,default=Path('local_data/rough_amplitude_probe'));p.add_argument('--baseline',type=Path,default=Path('local_data/rough_road_probe/fine'));p.add_argument('--output',type=Path,required=True);a=p.parse_args()
report=dict(schema='agv.rough_amplitude.v1',scope='Same 50x4 m field, 10 cm mesh, seed 20260915, 550 kg soft suspension; 10 km/h, 8 s steady truth at 50 Hz; one run per amplitude; no production-road acceptance.',cases={})
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
for name,d in [('baseline_0.784mm',a.baseline),('peak_1.5mm',a.input/'peak15'),('peak_3mm',a.input/'peak30')]:
 r=json.loads((d/'summary.json').read_text());r['geometry']=json.loads((d/'geometry.json').read_text());assert r['geometry']['step_m']==.1 and r['geometry']['triangles']==40000
 if 'all_motion_min_battery_clearance_m' in r:
  r['all_motion_min_battery_height_above_z0_m']=r.pop('all_motion_min_battery_clearance_m')
  r['all_motion_battery_clearance_lower_bound_m']=r['all_motion_min_battery_height_above_z0_m']-max(abs(r['geometry']['height_min_m']),abs(r['geometry']['height_max_m']))
 raw=json.loads((d/'samples.json').read_text());data=np.array([row[:-1] for row in raw if row[-1]=='cruise'],float)
 for ax,(label,col,scale) in zip(axes,(('Height (mm)',2,1000),('Roll (deg)',4,180/np.pi),('Pitch (deg)',5,180/np.pi))):
  ax.plot(data[:,1],(data[:,col]-data[:,col].mean())*scale,label=name);ax.set_ylabel(label);ax.grid(alpha=.3)
 report['cases'][name]=r
axes[0].legend();axes[-1].set_xlabel('Road X (m)');fig.tight_layout();fig.savefig(a.input/'comparison.png',dpi=150);plt.close(fig)
# Verify geometry scaling, not an assumed linear vehicle response.
def vertices(path):
 return np.array([[float(v) for v in line.split()[1:]] for line in path.read_text().splitlines() if line.startswith('v ')])
v1=vertices(a.input/'peak15/terrain.obj');v2=vertices(a.input/'peak30/terrain.obj')
assert np.array_equal(v1[:,:2],v2[:,:2]) and np.max(abs(v2[:,2]-2*v1[:,2]))<=1.01e-9
report['same_field_double_amplitude_verified']=True
report['capture_checks']={}
for name in ('peak15','peak30'):
 path=a.input/(name+'_capture.json')
 if not path.exists():continue
 r=json.loads(path.read_text())
 report['capture_checks'][name]={k:r[k] for k in ('passed','received_blocks','rows','real_time_factor','ros_all_blocks_byte_identical','final_motion_state','display_sync','realtime_required','realtime_target_met','realtime_floor')}
report['limitations']=['Uniform material, no road texture or defect feature-distortion validation','RTF capped near 1 does not quantify remaining CPU/GPU capacity or actual memory delta','0.2 degree steady deviation is an observation target, not a mechanical safety limit; passed denotes completed measurement, not that amplitude meets this target','Sampled suspension travel and battery clearance do not prove tyre contact forces or absence of brief liftoff']
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print('PASS: measured amplitude comparison and geometric 2x scaling verified')
