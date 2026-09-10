"""The validator must reap owned children even after ROS launch has exited."""
import subprocess
import sys
import time
import psutil
from process_resources import ResourceMonitor
from validate_rectangle_execution import stop_tree


def test_parent_exit_preserves_child_ownership(tmp_path):
    parent=subprocess.Popen([sys.executable,'-c',"import subprocess,time;subprocess.Popen(['sleep','60']);time.sleep(.7)"])
    monitor=ResourceMonitor(parent.pid,tmp_path/'resources.jsonl',.1)
    try:
        parent.wait(timeout=10);time.sleep(.2);report=monitor.close()
        children=[p for p in monitor.owned.values() if p.pid!=parent.pid]
        assert children
        stop_tree(parent,known_children=list(monitor.owned.values()))
        assert all(not p.is_running() or p.status()==psutil.STATUS_ZOMBIE for p in children)
        assert not report['errors']
    finally:
        monitor.close();stop_tree(parent,known_children=list(monitor.owned.values()))
