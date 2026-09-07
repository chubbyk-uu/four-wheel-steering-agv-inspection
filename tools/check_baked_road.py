#!/usr/bin/env python3
"""Check archived pixel integrity and exact shared gutter samples; no renderer involved."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from PIL import Image
from road_materials import MATERIALS

def check(path):
    root=Path(path);m=json.loads((root/'manifest.json').read_text())
    assert m['schema']=='agv.road.baked.v1'
    assert m['slab_size_m']==[5,5]
    n,g=m['core_pixels'],m['gutter_pixels'];size=n+2*g
    assert m['texel_m']<=.0003
    checked=0;worst=0;present=set()
    previews=m.get('previews')
    if previews:
        expected_overview=np.asarray(Image.open(root/previews['overview_mono']))[::-1]
        px,py,cw,ch=previews['crop_pixels_xy']
        archived_crop=np.zeros((ch,cw),np.uint8);filled=np.zeros((ch,cw),bool)
        stride=round(previews['overview_texel_m']/m['texel_m'])
        max_overview_delta=0
    for item in m['tiles']:
        raw=(root/item['mono']).read_bytes()
        assert hashlib.sha256(raw).hexdigest()==item['sha256']
        im=np.asarray(Image.open(root/item['mono']));lab=np.asarray(Image.open(root/item['labels']))
        assert im.shape==lab.shape==(size,size)
        present.update(map(int,np.unique(lab)))
        if 'labels_sha256' in item:
            assert hashlib.sha256((root/item['labels']).read_bytes()).hexdigest()==item['labels_sha256']
        ix,iy=item['ix'],item['iy']
        if previews:
            x0,x1=max(ix*n,px),min((ix+1)*n,px+cw)
            y0,y1=max(iy*n,py),min((iy+1)*n,py+ch)
            if x0<x1 and y0<y1:
                dst=np.s_[y0-py:y1-py,x0-px:x1-px]
                archived_crop[dst]=im[y0-iy*n+g:y1-iy*n+g,x0-ix*n+g:x1-ix*n+g]
                filled[dst]=True
            small=np.uint8(im[g:g+n,g:g+n].reshape(n//stride,stride,n//stride,stride).mean(axis=(1,3))+.5)
            ox,oy=ix*n//stride,iy*n//stride
            actual=expected_overview[oy:oy+n//stride,ox:ox+n//stride]
            if actual.size:
                d=int(np.max(np.abs(actual.astype(np.int16)-small[:actual.shape[0],:actual.shape[1]].astype(np.int16))))
                max_overview_delta=max(max_overview_delta,d)
                assert d==0,('overview',ix,iy,d)
        for dx,dy in ((1,0),(0,1)):
            if ix+dx>=m['tiles_x'] or iy+dy>=m['tiles_y']:continue
            for suffix,a in (('.pgm',im),('_labels.png',lab)):
                b=np.asarray(Image.open(root/f'tile_{ix+dx}_{iy+dy}{suffix}'))
                first,second=(a[:,n:n+2*g],b[:,:2*g]) if dx else (a[n:n+2*g],b[:2*g])
                delta=int(np.max(np.abs(first.astype(np.int16)-second.astype(np.int16))))
                worst=max(worst,delta);assert delta==0,(ix,iy,dx,dy,suffix,delta)
            checked+=1
    assert present==set(m['labels'].values()),present
    for c in m['cracks']:
        assert .8-1e-5<=c['nominal_main_width_min_mm']<=c['nominal_main_width_max_mm']<=2+1e-5
        assert c['length_extent_m']>=2
        assert c['source']=='imagegen' and c['procedural_crack_paths'] is False
    result=dict(tiles=len(m['tiles']),shared_edges=checked,max_overlap_delta_dn=worst,all_labels_present=True,
                hashes_verified=True,slab_size_m=m['slab_size_m'],texel_mm=m['texel_m']*1000,
                calibrated_width_range_mm=[.8,2.0],width_validation='nominal centerline diameters only; does not certify every final raster cross-section',
                crack_extents_m=[c['length_extent_m'] for c in m['cracks']],passed=True)
    if previews:
        assert filled.all()
        preview_crop=np.asarray(Image.open(root/previews['mono_crop']))[::-1]
        assert np.array_equal(archived_crop,preview_crop)
        before=np.asarray(Image.open(root/previews['before']));after=np.asarray(Image.open(root/previews['after']))
        support=np.asarray(Image.open(root/previews['support']))>0
        assert before.shape==after.shape==(ch,cw,3)
        assert 0<support.mean()<.1
        delta=np.max(np.abs(before.astype(np.int16)-after.astype(np.int16))[~support])
        assert delta==0
        assert np.any(before[support]!=after[support])
        material=MATERIALS[m.get('material_id','gravel_concrete_03')]
        assert m['source_url']==material['url']
        assert m['source_width_m']==material['width_m']
        assert m['background_quilting']['method']=='minimum_error_patch_quilting'
        assert m.get('guide_source_width_shift_mean_abs_difference',m.get('guide_2p1m_shift_mean_abs_difference',0))>1e-5
        result.update(preview_crop_matches_archived_pixels=True,overview_matches_archive=True,
                      overview_max_delta_dn=max_overview_delta,background_outside_defects_max_delta_dn=int(delta),
                      defect_support_fraction=float(support.mean()),source_url=m['source_url'],
                      source_width_period_exact_repetition=False)
    (root/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');check(p.parse_args().directory)
