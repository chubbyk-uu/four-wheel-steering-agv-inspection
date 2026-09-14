import numpy as np
from compare_strip_pose_modes import measure


def test_clipped_marking_is_not_reported_as_a_shifted_complete_line():
    spacing=1.5/4096
    y=-1+(np.arange(5462)+.5)*spacing
    image=np.full((64,len(y)),40,np.uint8)
    for center in [-.15,.15]:image[:,abs(y-center)<.075]=125
    valid=np.ones_like(image,bool)
    full=measure(image,spacing,-1,valid=valid)
    np.testing.assert_allclose(full,[[-.15,.15]]*2,atol=spacing)
    valid[:,y<-.20]=False;image[~valid]=0
    clipped=measure(image,spacing,-1,valid=valid)
    assert np.isnan(clipped[:,0]).all()
    np.testing.assert_allclose(clipped[:,1],.15,atol=spacing)
