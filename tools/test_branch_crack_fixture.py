"""Extraction regression fixture only; these synthetic lines are never road assets."""
import cv2
import numpy as np
from branch_crack_fixture import clean_source_skeleton


def test_extraction_removes_short_burr_but_retains_long_source_fork():
    mask=np.zeros((100,140),'uint8')
    cv2.line(mask,(10,65),(130,65),255,3)
    cv2.line(mask,(70,65),(95,15),255,3)
    cv2.line(mask,(40,65),(40,58),255,3)
    original=mask.copy()
    paths=clean_source_skeleton(mask>0)
    points=np.concatenate(paths)
    for tip in ((10,65),(130,65),(95,15)):
        assert np.min(np.linalg.norm(points-np.array(tip),axis=1))<3
    assert np.min(np.linalg.norm(points-np.array([40,58]),axis=1))>3
    assert np.array_equal(mask,original)


def test_source_scale_extraction_is_deterministic():
    mask=np.zeros((40,80),'uint8')
    cv2.line(mask,(5,20),(75,20),255,3)
    a=clean_source_skeleton(mask>0)
    b=clean_source_skeleton(mask>0)
    assert len(a)==len(b)>0
    assert all(np.array_equal(x,y) for x,y in zip(a,b))
