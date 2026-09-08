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
