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


def test_trial_markings_geometry_and_lane_clearance():
    from road_test_markings import polygons,masks as trial_masks
    items=polygons()
    for item in items:
        p=np.array(item['vertices']);edge=np.roll(p,-1,axis=0)-p
        assert np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))>0
        for a,b in zip(p,edge):
            assert np.min(b[0]*(p[:,1]-a[1])-b[1]*(p[:,0]-a[0]))>=-1e-9
        assert np.max(np.abs(p[:,1]))<4.5-.075
        assert np.min(np.abs(p[:,1]))>.225
    # Known shaft, tip, outside-head, reverse-arrow and box border locations.
    y,w=trial_masks([3.5,5.6,5.8,8.85],[-2.4,2.4,-4.0],items)
    assert w[0,0] and w[0,1] and not w[0,2] and w[1,2]
    assert y[2,3]
