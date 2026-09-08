import numpy as np
import pytest
from wheel_dead_reckoning import WheelOdometry


@pytest.mark.parametrize('steer,distance,expected',[(0,-.01,[-.01,0,0]),(-np.pi/2,-.01,[0,.01,0]),(np.pi/4,.01,[.01/np.sqrt(2),.01/np.sqrt(2),0])])
def test_drive_sign_and_steer_define_motion(steer,distance,expected):
    o=WheelOdometry(.1,.65,.55)
    o.update(1,np.zeros(4),np.full(4,steer));o.update(1.1,np.full(4,distance/.1),np.full(4,steer))
    np.testing.assert_allclose(o.pose,expected,atol=1e-12)


def test_rotating_chassis_and_inconsistent_wheel_are_detectable():
    o=WheelOdometry(.1,.65,.55)
    p=np.array([[.325,.275],[.325,-.275],[-.325,.275],[-.325,-.275]])
    vector=np.column_stack((-p[:,1],p[:,0]))*.01
    steer=np.arctan2(vector[:,1],vector[:,0]);drive=np.linalg.norm(vector,axis=1)/.1
    o.update(1,np.zeros(4),steer);o.update(1.1,drive,steer)
    np.testing.assert_allclose(o.pose,[0,0,.01],atol=1e-12)
    drive[0]+=.1;o.update(1.2,drive,steer)
    assert o.residual_speed>.01
    with pytest.raises(ValueError,match='gap'):o.update(2,drive,steer)
