#!/usr/bin/env python3
"""Fetch the selected lossless color texture through an explicit proxy; verify provider hash."""
import argparse,hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--proxy',default=None)
p.add_argument('--asset',choices=['gravel_concrete_03','brushed_concrete_03'],default='gravel_concrete_03');a=p.parse_args()
out=ROOT/'assets/road/source';out.mkdir(parents=True,exist_ok=True)
curl=['curl','--fail','--silent','--show-error','--location','--retry','2','--connect-timeout','15']
if a.proxy:curl += ['--proxy',a.proxy]
meta=json.loads(subprocess.run(curl+['--max-time','60','https://api.polyhaven.com/files/'+a.asset],
                              check=True,capture_output=True).stdout)
asset=meta['Diffuse']['8k']['png'];target=out/(a.asset+'_diff_8k.png')
def digest(path):
    h=hashlib.md5()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
if target.exists():
    if digest(target)!=asset['md5']:raise RuntimeError('Existing source hash mismatch; refusing overwrite')
else:
    part=target.with_suffix('.png.part')
    subprocess.run(curl+['--max-time','600',asset['url'],'-o',str(part)],check=True)
    if digest(part)!=asset['md5']:raise RuntimeError('Downloaded source hash mismatch')
    part.rename(target)
(out/(a.asset+'_files.json')).write_text(json.dumps(meta,indent=2)+'\n')
print('Verified '+a.asset+' 8K PNG; '+str(target))
