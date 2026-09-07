#!/usr/bin/env python3
"""Download selected ambientCG archive via proxy and extract only its color map."""
import argparse, hashlib, json, shutil, subprocess, zipfile
from pathlib import Path
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--proxy',default=None);a=p.parse_args()
out=ROOT/'assets/road/source';out.mkdir(parents=True,exist_ok=True)
url='https://ambientcg.com/get?file=Concrete047A_8K-PNG.zip'
archive=out/'Concrete047A_8K-PNG.zip';part=archive.with_suffix('.zip.part')
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
target=out/'Concrete047A_8K-PNG_Color.png'
record=out/'Concrete047A_download.json'
if target.exists() and record.exists():
    saved=json.loads(record.read_text())
    if saved['color_file']!=target.name or sha(target)!=saved['color_sha256']:
        raise ValueError('Existing color hash mismatch; refusing overwrite')
    with Image.open(target) as im:
        if list(im.size)!=saved['color_resolution']:raise ValueError('Unexpected color resolution')
    print('Verified existing Concrete047A color map; archive download not needed')
    raise SystemExit(0)
curl=['curl','--fail','--silent','--show-error','--location','--retry','2','--connect-timeout','15']
if a.proxy:curl += ['--proxy',a.proxy]
meta=json.loads(subprocess.run(curl+['--max-time','60','https://ambientcg.com/api/v2/full_json?id=Concrete047A'],capture_output=True,check=True).stdout)
(out/'Concrete047A_api.json').write_text(json.dumps(meta,indent=2)+'\n')
if not archive.exists():
    # A completed manually downloaded part can be resumed without another 1 GB transfer.
    if not part.exists() or not zipfile.is_zipfile(part):
        subprocess.run(curl+['--max-time','900',url,'-o',str(part)],check=True)
    if not zipfile.is_zipfile(part):raise ValueError('Not a ZIP archive')
    part.rename(archive)
with zipfile.ZipFile(archive) as z:
    members=[n for n in z.namelist() if Path(n).name=='Concrete047A_8K-PNG_Color.png']
    if len(members)!=1:raise ValueError('Expected exactly one color map: '+str(z.namelist()))
    target=out/'Concrete047A_8K-PNG_Color.png'
    extracted=target.with_suffix('.png.part')
    # Copy a selected member to a fixed path; ZIP CRC is checked while reading.
    with z.open(members[0]) as src,extracted.open('wb') as dst:shutil.copyfileobj(src,dst)
    with Image.open(extracted) as im:
        if im.size!=(8192,8192):raise ValueError('Unexpected color resolution')
    extracted.replace(target)
report=dict(url=url,license='CC0',archive_bytes=archive.stat().st_size,archive_sha256=sha(archive),
            color_file=target.name,color_sha256=sha(target),color_resolution=[8192,8192],
            integrity='ZIP member CRC and local SHA256; no provider digest asserted',
            scale='API dimensionX/Y/Z are unspecified (0); project comparison maps color to 2.1 m')
(out/'Concrete047A_download.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
