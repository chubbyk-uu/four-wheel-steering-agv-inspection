"""Health status leaves the 500 Hz delivery thread without waiting on a consumer."""
from pathlib import Path
import threading
import time
from agv_localization.offload import TelemetryPublisher


def test_offer_does_not_wait_for_the_transport():
    gate=threading.Event();entered=threading.Event();sent=[]
    class Recorder:
        def publish(self,message):
            entered.set();gate.wait(5);sent.append(message)
    telemetry=TelemetryPublisher(Recorder(),lambda data:data)
    try:
        telemetry.offer('first');assert entered.wait(2)
        start=time.monotonic()
        for i in range(200):telemetry.offer(i)
        assert time.monotonic()-start<.01
    finally:
        gate.set();assert telemetry.close()
    assert sent[0]=='first'


def test_the_two_package_copies_do_not_drift():
    # No dependency links these packages, so the helper is duplicated on purpose;
    # a silent divergence would leave one control loop unprotected.
    here=Path(__file__).resolve().parents[1]/'agv_localization/offload.py'
    other=Path(__file__).resolve().parents[2]/'agv_mission/agv_mission/offload.py'
    assert other.is_file()
    assert here.read_text()==other.read_text()
