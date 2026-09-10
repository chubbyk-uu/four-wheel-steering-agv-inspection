import numpy as np
import pytest
from agv_linescan.obj_arrays import read_obj_python


@pytest.fixture(params=['native','python'])
def reader(request):
    if request.param=='python':return read_obj_python
    # CMake supplies the actual target directory, including custom build bases.
    from _obj_arrays import read_obj
    return lambda p,with_uv=False:read_obj(str(p),with_uv)



def test_checked_obj_arrays(reader,tmp_path):
    p=tmp_path/'mesh.obj';p.write_text('v +0.0 0 0\nv 1 0 0\nv 0 1 -.002\nvt 0 1\nvt 1 1\nvt 0 0\nvn 0 0 1\nf 1/1/1 2/2/1 3/3/1\n')
    v,f,uv=reader(p,True)
    np.testing.assert_array_equal(v,[[0,0,0],[1,0,0],[0,1,-.002]])
    np.testing.assert_array_equal(f,[[0,1,2]])
    np.testing.assert_array_equal(uv,[[0,1],[1,1],[0,0]])
    p.write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
    assert reader(p)[2] is None
    with pytest.raises(ValueError):reader(p,True)


@pytest.mark.parametrize('change',[('v 0 0 0','v nan 0 0'),('v 0 0 0','v 0 0'),('f 1/1/1 2/2/1 3/3/1','f 1/1/1 2/1/1 3/3/1'),('3/3/1','4/4/1'),('3/3/1','3/3/1 1/1/1')])
def test_invalid_obj_is_not_accepted(reader,tmp_path,change):
    source='v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 1\nvt 1 1\nvt 0 0\nf 1/1/1 2/2/1 3/3/1\n'
    p=tmp_path/'mesh.obj';p.write_text(source.replace(*change))
    with pytest.raises(ValueError):reader(p,True)
