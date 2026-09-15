"""A paused frame may become a valid tail only when capture later ends."""
import importlib.util
from pathlib import Path
import pytest


@pytest.mark.parametrize('rows,reason,valid',[
    (4096,'full',True),(1715,'capture_toggle',True),
    (1715,'full',False),(999,'capture_toggle',False),
    (1715,'unsupported_scan_motion',False),(4097,'full',False)])
def test_retained_frame_termination(monkeypatch,rows,reason,valid):
    tools=Path(__file__).resolve().parents[3]/'tools'
    monkeypatch.syspath_prepend(str(tools))
    spec=importlib.util.spec_from_file_location('rectangle_pause_probe',tools/'validate_rectangle_execution.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    meta=dict(rows=rows,end_reason=reason)
    if valid:module.check_pause_block(meta,4096,1000)
    else:
        with pytest.raises(AssertionError):module.check_pause_block(meta,4096,1000)
