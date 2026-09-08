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
