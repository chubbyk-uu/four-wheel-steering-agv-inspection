import time
import pytest
import rclpy
from rclpy.node import Node
from agv_mission.callback_trace import CallbackTrace,TracedExecutor


def test_slow_callback_trace_is_bounded():
    trace=CallbackTrace()
    for i in range(500):trace.record(str(i),.259,.001)
    value=trace.snapshot()
    assert len(value['recent_slow'])==64 and len(value['max_wall_s'])<=129
    assert value['recent_slow'][-1]['thread_cpu_s']==.001


def test_real_executor_records_blocking_timer_and_preserves_exceptions():
    rclpy.init();executor=TracedExecutor();node=Node('callback_trace_test')
    try:
        def delayed():
            timer.cancel();time.sleep(.03)
        timer=node.create_timer(.001,delayed);executor.add_node(node)
        end=time.monotonic()+3
        while not any('delayed' in x['callback'] for x in executor.trace.slow) and time.monotonic()<end:executor.spin_once(timeout_sec=.1)
        assert any('delayed' in x['callback'] and x['wall_s']>=.03 for x in executor.trace.slow)
        def broken():raise RuntimeError('callback exception retained')
        node.create_timer(.001,broken)
        with pytest.raises(RuntimeError,match='callback exception retained'):
            end=time.monotonic()+3
            while time.monotonic()<end:executor.spin_once(timeout_sec=.1)
    finally:
        node.destroy_node();executor.shutdown();rclpy.try_shutdown()


def test_stall_watch_names_the_kernel_wait_point():
    import threading,time
    from agv_mission.callback_trace import PhaseTrace,StallWatch
    probe=PhaseTrace(.02);watch=StallWatch(probe,.02,period=.002)
    watch.start(threading.get_native_id())
    try:
        probe.begin();time.sleep(.15);probe.end()
    finally:watch.close()
    entry=probe.slow[-1]
    assert entry['wait_points'],'no wait point sampled'
    # The blocked thread must be identified by the kernel function it sleeps in,
    # not merely by how long it took.
    assert any(p['wchan'] and p['wchan']!='0' for p in entry['wait_points']),entry['wait_points']
    assert all(p['syscall'].isdigit() or p['syscall']=='running' for p in entry['wait_points'])
