#!/usr/bin/env python3
"""Offline column correction followed by exact row concatenation, without navigation."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/agv_linescan'))
from agv_linescan.offline_correction import process_session


def main():
    p=argparse.ArgumentParser();p.add_argument('--mission',type=Path,required=True);p.add_argument('--profile',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reverse-preview-tracks',type=int,nargs='*',default=[])
    a=p.parse_args();width_m=json.loads(a.profile.read_text())['geometry']['output_width_m'];intervals=json.loads((a.mission/'capture_intervals.json').read_text());a.output.mkdir(parents=True,exist_ok=False)
    sources=list(dict.fromkeys(i['archive'] for i in intervals));corrected={}
    for k,source in enumerate(sources):
        dest=a.output/f'corrected_{k:02d}';process_session(source,a.profile,dest);corrected[source]=dest
    fig,axes=plt.subplots(1,len(intervals),figsize=(5*len(intervals),11),squeeze=False);reports=[]
    for index,interval in enumerate(intervals):
        pieces=[];blocks=[];previous=None;spacing=None
        for path in sorted(Path(interval['archive']).glob('block_*.json')):
            m=json.loads(path.read_text())
            if not interval['enabled_ack_time_s']-.05<=m['first']['time_s']<=m['last']['time_s']<=interval['disabled_ack_time_s']+.05:continue
            if previous is not None and m['first']['global_line']!=previous+1:raise ValueError('cannot concatenate a missing line range')
            previous=m['last']['global_line']
            if spacing is not None and spacing!=m['line_spacing_m']:raise ValueError('line spacing changed')
            spacing=m['line_spacing_m'];im=np.array(Image.open(corrected[interval['archive']]/path.with_suffix('.pgm').name))
            pieces.append(im);blocks.append(dict(block_id=m['block_id'],rows=m['rows'],first_global_line=m['first']['global_line'],last_global_line=m['last']['global_line']))
        if not pieces:raise ValueError('empty track')
        strip=np.concatenate(pieces,axis=0);track=interval['track_id'];target=a.output/f'track_{track:02d}.png';Image.fromarray(strip).save(target)
        restored=np.array(Image.open(target));offset=0
        for piece in pieces:
            assert np.array_equal(restored[offset:offset+len(piece)],piece);offset+=len(piece)
        reverse=track in a.reverse_preview_tracks;preview=strip[::-1,::-1] if reverse else strip
        ax=axes[0,index];ax.imshow(preview[::4,::4],cmap='gray',vmin=0,vmax=160,extent=[-width_m/2,width_m/2,len(strip)*spacing,0],aspect='equal')
        ax.set_title(f'Track {track}: {len(pieces)} corrected blocks\n'+('Preview rotated 180 degrees' if reverse else 'Acquisition order'))
        ax.set_xlabel('Across strip (m)');ax.set_ylabel('Distance from displayed end (m)')
        reports.append(dict(track_id=track,blocks=blocks,shape=list(strip.shape),pixel_sha256=hashlib.sha256(strip).hexdigest(),decoded_png_equals_corrected_blocks=True,preview_rotation_deg=180 if reverse else 0))
    fig.suptitle('Direct continuous strips: optical + flat-field correction only\nNo navigation projection, matching, stretching or seam blending',fontsize=12)
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(a.output/'comparison.png',dpi=150);plt.close(fig)
    result=dict(tracks=reports,calibration_id=json.loads(a.profile.read_text())['calibration_id'],scope='Exact corrected-row concatenation in acquisition order. Preview reversal is display only. Track origins are independent; this is not geographical alignment.',preview_gray_range=[0,160])
    (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
