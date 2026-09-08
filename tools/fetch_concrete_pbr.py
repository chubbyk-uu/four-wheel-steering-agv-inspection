#!/usr/bin/env python3
"""Restore matching 8K NormalGL/roughness maps; never bake lighting/AO into albedo."""
import argparse,json,subprocess,zipfile,shutil
from pathlib import Path
from PIL import Image
from bake_concrete_road import sha
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--proxy');a=p.parse_args()
 folder=ROOT/'assets/road/source';record=folder/'Concrete047A_pbr_download.json'
 names=[f'Concrete047A_8K-PNG_{m}.png' for m in ('NormalGL','Roughness')]
 if record.exists():
  r=json.loads(record.read_text())
  if all((folder/n).exists() and sha(folder/n)==r['maps'][n]['sha256'] for n in names):
   print('Verified cached PBR maps');return
 url='https://ambientcg.com/get?file=Concrete047A_8K-PNG.zip'
 archive=folder/'Concrete047A_8K-PNG.zip';part=archive.with_suffix('.zip.part')
 if not archive.exists():
  cmd=['curl','--fail','--silent','--show-error','--location','--retry','2','--connect-timeout','15','--max-time','900']
  if a.proxy:cmd+=['--proxy',a.proxy]
  subprocess.run(cmd+[url,'-o',str(part)],check=True)
  if not zipfile.is_zipfile(part):raise ValueError('Invalid source archive')
  part.rename(archive)
 r=dict(url=url,license='CC0',normal_convention='OpenGL tangent-space; image rows downward, UV v upward',maps={},archive_sha256=sha(archive))
 # Same archive as existing color is required: avoid mismatched supplier revisions.
 color_record=json.loads((folder/'Concrete047A_download.json').read_text())
 if r['archive_sha256']!=color_record['archive_sha256']:raise ValueError('Source archive changed; review material alignment before proceeding')
 with zipfile.ZipFile(archive) as z:
  for name in names:
   matches=[m for m in z.namelist() if Path(m).name==name]
   if len(matches)!=1:raise ValueError('Missing unique source map: '+name)
   target=folder/name;tmp=target.with_suffix('.png.part')
   with z.open(matches[0]) as source,tmp.open('wb') as dest:shutil.copyfileobj(source,dest)
   with Image.open(tmp) as im:
    if im.size!=(8192,8192):raise ValueError('Unexpected map dimensions')
    mode=im.mode
   tmp.replace(target);r['maps'][name]=dict(sha256=sha(target),resolution=[8192,8192],mode=mode)
 record.write_text(json.dumps(r,indent=2)+'\n');print('Downloaded and verified matching 8K PBR maps')
if __name__=='__main__':main()
