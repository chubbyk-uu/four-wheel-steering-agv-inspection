import numpy as np
from road_markings import masks,paint,metadata

def test_two_lane_cross_section_separates_joint_and_paint():
    ys=np.arange(-5,5,.001)+.0005
    yellow,white=masks([1.,5.,99.],ys,10)
    assert np.all(yellow[:,0]==yellow[:,1]) and np.all(yellow[:,1]==yellow[:,2])
    assert yellow[:,0].sum()==300 and white[:,0].sum()==300
    assert not np.any(yellow&white)
    assert not np.any((yellow|white)[abs(ys)<=.004])
    assert np.all((yellow|white)[abs(ys)>4.575]==False)
    rgb=np.full((len(ys),3,3),.2,np.float32);paint(rgb,[1,5,99],ys,10)
    assert np.all(rgb[yellow,0]>rgb[yellow,1]) and np.all(rgb[yellow,1]>rgb[yellow,2])
    assert np.all(rgb[white,0]==rgb[white,1])
    assert metadata(10)['center']['clear_gap_m']==.15
