"""Sampling consistency at storage boundaries, metric mapping and reproducibility."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import pytest
from concrete_quilt import ConcreteQuilt,minimum_cut

def make(seed=17):
    rng=np.random.default_rng(19)
    source=rng.integers(40,200,(256,256,3),dtype=np.uint8)
    lut=np.arange(256,dtype=np.float32)/255
    return ConcreteQuilt(source,lut,2,2,.2,source_width=2.1,seed=seed,
                         guide_side=64,patch=40,overlap=12,candidates=8,feather=1)

def test_exact_same_samples_across_arbitrary_tile_partitions():
    quilt=make()
    xs=np.linspace(0,2,137);ys=np.linspace(-1,1,149)
    whole=quilt.sample(xs,ys)
    rows=[]
    for y in np.array_split(ys,4):
        rows.append(np.concatenate([quilt.sample(x,y) for x in np.array_split(xs,3)],axis=1))
    assert np.array_equal(whole,np.concatenate(rows,axis=0))
    assert np.isfinite(whole).all()
    assert whole.min()>0

def test_source_uses_physical_width_and_seeded_layout():
    a,b,c=make(),make(),make(seed=18)
    assert a.gsd*a.guide_side==pytest.approx(2.1)
    assert np.array_equal(a.guide,b.guide)
    assert not np.array_equal(a.guide,c.guide)
    assert all(0<=p['sx']<=24 and 0<=p['sy']<=24 for p in a.placements)
    assert np.all(a.placements[0]['alpha']==255)

def test_out_of_domain_is_not_silently_wrapped():
    q=make()
    with pytest.raises(ValueError,match='outside'):
        q.sample([-10,0],[0])

def test_cut_follows_low_error_path():
    error=np.ones((10,7));error[:,3]=0
    assert np.array_equal(minimum_cut(error),np.full(10,3))


def test_pbr_channels_replay_one_layout_at_tile_boundaries():
    q=make();xs=np.linspace(0,2,137);ys=np.linspace(-1,1,149)
    channel=255-q.source
    lut=np.arange(256,dtype=np.float32)/255
    a=q.sample(xs,ys,source=channel,lut=lut)
    b=np.concatenate([q.sample(xs,y,source=channel,lut=lut) for y in np.array_split(ys,3)])
    assert np.array_equal(a,b)
    # Complementary source patterns remain registered through every quilting seam.
    assert np.max(np.abs(a+q.sample(xs,ys)-1))<=1/255+1e-6
    assert np.array_equal(q.sample(xs,ys),q.sample(xs,ys,source=q.source,lut=q.lut))


def test_pbr_rejects_mismatched_source_resolution():
    q=make()
    with pytest.raises(ValueError,match='source dimensions'):
        q.sample([0],[0],source=q.source[::2])
