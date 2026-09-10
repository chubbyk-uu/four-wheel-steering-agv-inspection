import numpy as np
from generate_streaming_road import clip_partition


def test_partition_preserves_area_and_interpolated_height():
    vertices=np.array([[0.,0.,0.],[2.,0.,.02],[2.,2.,.04],[0.,2.,.02]])
    faces=np.array([[0,1,2],[0,2,3]])
    area=0
    for lo,hi in ((0,1),(1,2)):
        v,f=clip_partition(vertices,faces,lo,hi)
        assert np.all(v[:,0]>=lo) and np.all(v[:,0]<=hi)
        np.testing.assert_allclose(v[:,2],.01*(v[:,0]+v[:,1]),atol=1e-12)
        n=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
        assert np.all(n[:,2]>0)
        area+=n[:,2].sum()/2
    assert abs(area-4)<1e-12


def test_parallel_bake_is_byte_identical_and_checks_gutters(tmp_path):
    import pytest
    from streaming_tiles import bake_tiles
    def sample(xs,ys):
        x,y=np.meshgrid(xs,ys)
        rgb=np.repeat(((x+y)%1)[:,:,None],3,axis=2).astype(np.float32)
        normal=np.zeros_like(rgb);normal[:,:,2]=1
        return rgb,normal
    a,b=tmp_path/'serial',tmp_path/'parallel';a.mkdir();b.mkdir()
    first,worst=bake_tiles(sample,a,3,2,0,0,8,2,.001,workers=1)
    second,other=bake_tiles(sample,b,3,2,0,0,8,2,.001,workers=4)
    assert first==second and worst==other==0
    for entry in first:
        for kind in ('color','normal'):
            name=entry[kind]['file'];assert (a/name).read_bytes()==(b/name).read_bytes()
    data=bytearray((b/'color_1_0.raw').read_bytes());data[0]^=1;(b/'color_1_0.raw').write_bytes(data)
    with pytest.raises(ValueError,match='gutter'):
        bake_tiles(sample,b,3,2,0,0,8,2,.001,reuse=True,workers=4)
