from concurrent.futures import Future
from types import SimpleNamespace
from agv_mission.capture import CaptureGate


class Client:
    def __init__(self):self.requests=[];self.future=None
    def service_is_ready(self):return True
    def call_async(self,request):
        self.requests.append(request.data);self.future=Future();return self.future


class Node:
    def __init__(self):self.client=Client()
    def create_client(self,*args):return self.client
    def create_subscription(self,*args):pass
    def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000))


def acknowledge(gate,node):
    node.client.future.set_result(SimpleNamespace(success=True,message='archive'))
    gate.poll()


def test_explicit_initial_disable_and_pending_enable_close_order(tmp_path):
    n=Node();g=CaptureGate(n,tmp_path,True)
    try:
        assert g.active is None and not g.request(False)
        acknowledge(g,n);assert g.active is False and g.intervals==[]
        assert not g.request(True,2)
        assert not g.request(False)  # Must wait for outstanding enable response.
        assert n.client.requests==[False,True]
        acknowledge(g,n);assert g.active is True
        assert not g.request(False);acknowledge(g,n)
        assert g.active is False and g.intervals[0]['track_id']==2
        assert 'disabled_ack_time_s' in g.intervals[0]
    finally:g.close()


def test_failed_handshake_does_not_claim_active_capture(tmp_path):
    n=Node();g=CaptureGate(n,tmp_path,True)
    try:
        g.request(True,0)
        n.client.future.set_result(SimpleNamespace(success=False,message='missing terrain'));g.poll()
        assert g.error.startswith('CAMERA_HANDSHAKE_FAILED') and g.active is None
    finally:g.close()


def test_close_records_cancellation_reason_and_exception_latches_failure(tmp_path):
    n=Node();g=CaptureGate(n,tmp_path,True)
    try:
        g.request(True,0);acknowledge(g,n)
        g.request(False,reason='CANCELED');acknowledge(g,n)
        assert g.intervals[-1]['end_reason']=='CANCELED'
        g.request(True,0);n.client.future.set_exception(RuntimeError('transport failed'));g.poll()
        assert g.active is False and g.future is None and g.error.startswith('CAMERA_HANDSHAKE_FAILED')
    finally:g.close()


def test_camera_heartbeat_distinguishes_pause_from_disabled_and_stale_data(tmp_path,monkeypatch):
    import json
    from agv_mission import capture
    clock=[10.];monkeypatch.setattr(capture.time,'monotonic',lambda:clock[0])
    n=Node();g=CaptureGate(n,tmp_path,True)
    try:
        g.request(True,0);acknowledge(g,n)
        g.on_state(SimpleNamespace(data=json.dumps(dict(time_s=.99,enabled=False))))
        g.poll();assert not g.error # Old disabled state predates enable acknowledgement.
        clock[0]+=.1
        g.on_state(SimpleNamespace(data=json.dumps(dict(time_s=1.01,enabled=True,sampling_active=True))))
        g.poll();assert not g.error # No image needed while paused.
        clock[0]+=.1
        g.on_state(SimpleNamespace(data=json.dumps(dict(time_s=1.02,enabled=False))))
        g.poll();assert g.error=='CAMERA_DISABLED_UNEXPECTEDLY'
    finally:g.close()


def test_missing_or_replayed_camera_heartbeat_times_out(tmp_path,monkeypatch):
    import json
    from agv_mission import capture
    clock=[10.];monkeypatch.setattr(capture.time,'monotonic',lambda:clock[0])
    n=Node();g=CaptureGate(n,tmp_path,True)
    try:
        g.request(True,0);acknowledge(g,n)
        message=SimpleNamespace(data=json.dumps(dict(time_s=1.,enabled=True)))
        g.on_state(message);clock[0]+=.6;g.on_state(message);g.poll()
        assert g.error=='CAMERA_STATE_TIMEOUT'
    finally:g.close()


def test_state_before_first_ros_clock_is_not_a_camera_fault(tmp_path):
    import json
    n=Node();n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=0))
    g=CaptureGate(n,tmp_path,True)
    try:
        g.on_state(SimpleNamespace(data=json.dumps(dict(time_s=12.,enabled=False))))
        assert not g.error and g.heartbeat is None
    finally:g.close()


def test_scan_interruptions_fault_instead_of_silent_partial_success(tmp_path):
    import json
    n=Node();g=CaptureGate(n,tmp_path,True)
    try:
        for reason in ('unsupported_scan_motion','direction change','motion_UNKNOWN','nonfinite encoder'):
            g.error='';g.event(SimpleNamespace(data=json.dumps(dict(reason=reason))))
            assert g.error=='CAMERA_'+reason
        g.error='';g.event(SimpleNamespace(data=json.dumps(dict(reason='capture_toggle'))));assert not g.error
    finally:g.close()


def test_failed_close_is_not_retried_or_reported_as_archived(tmp_path):
    n=Node();g=CaptureGate(n,tmp_path,True)
    try:
        g.request(True,0);acknowledge(g,n);g.request(False)
        n.client.future.set_result(SimpleNamespace(success=False,message='storage failed'));g.poll()
        assert g.close_failed and g.active is True and 'disabled_ack_time_s' not in g.intervals[0]
        count=len(n.client.requests)
        for _ in range(5):assert not g.request(False)
        assert len(n.client.requests)==count
    finally:g.close()
