"""The control loop must hand telemetry over, never make the waiting call itself."""
import threading
import time
import pytest
from agv_mission.offload import TelemetryPublisher


class Recorder:
    def __init__(self,block=None,fail=0):
        self.sent=[];self.block=block;self.fail=fail;self.entered=threading.Event()

    def publish(self,message):
        self.entered.set()
        if self.block is not None:self.block.wait(5)
        if self.fail:self.fail-=1;raise RuntimeError('transport unavailable')
        self.sent.append(message)


@pytest.fixture
def make():
    created=[]
    def build(recorder,depth=64):
        value=TelemetryPublisher(recorder,lambda data:data,depth);created.append(value);return value
    yield build
    for value in created:value.close()


def test_offer_returns_while_the_transport_is_still_waiting(make):
    gate=threading.Event();recorder=Recorder(block=gate);telemetry=make(recorder)
    telemetry.offer('first');assert recorder.entered.wait(2)
    # This is the measured failure: 0.328 s inside one 50 Hz tick. Enqueueing
    # 200 further samples must cost far less than a single control period.
    start=time.monotonic()
    for i in range(200):telemetry.offer('sample%d'%i)
    elapsed=time.monotonic()-start
    gate.set()
    assert elapsed<.02,elapsed


def test_payloads_arrive_in_order(make):
    recorder=Recorder();telemetry=make(recorder)
    for i in range(50):telemetry.offer(i)
    assert telemetry.close()
    assert recorder.sent==list(range(50))


def test_a_backlog_drops_the_oldest_and_counts_it(make):
    gate=threading.Event();recorder=Recorder(block=gate);telemetry=make(recorder,depth=4)
    telemetry.offer('taken');assert recorder.entered.wait(2)
    for i in range(10):telemetry.offer(i)
    gate.set();assert telemetry.close()
    assert telemetry.dropped==6
    assert recorder.sent==['taken',6,7,8,9]


def test_a_failing_transport_neither_kills_the_thread_nor_hides_itself(make):
    recorder=Recorder(fail=2);telemetry=make(recorder)
    for value in ('a','b','c'):telemetry.offer(value)
    assert telemetry.close()
    assert telemetry.errors==2 and 'transport unavailable' in telemetry.last_error
    assert recorder.sent==['c']


def test_archive_append_does_not_wait_for_the_filesystem():
    import threading,time
    from agv_mission.offload import ArchiveWriter
    gate=threading.Event();entered=threading.Event();written=[]
    class Handle:
        def write(self,text):
            entered.set();gate.wait(5);written.append(text)
        def flush(self):pass
        def close(self):written.append('close')
    writer=ArchiveWriter(flush_interval_s=.01);handle=writer.track(Handle())
    try:
        writer.append(handle,'first');assert entered.wait(2)
        # The measured stall was 0.35 s of system-wide I/O. Enqueueing a further
        # 500 records must stay far inside one 20 ms control period.
        start=time.monotonic()
        for i in range(500):writer.append(handle,'row%d'%i)
        assert time.monotonic()-start<.02
    finally:gate.set()
    assert writer.close()
    assert written==['first']+['row%d'%i for i in range(500)]+['close']
    assert writer.peak>1 and not writer.errors


def test_archive_reports_a_failed_write_instead_of_losing_it_silently():
    from agv_mission.offload import ArchiveWriter
    class Handle:
        def write(self,text):raise OSError('no space left on device')
        def flush(self):pass
        def close(self):pass
    writer=ArchiveWriter(flush_interval_s=.01);handle=writer.track(Handle())
    writer.append(handle,'row')
    assert writer.close()
    assert writer.errors==1 and 'no space left' in writer.last_error


def test_archive_close_drains_every_pending_write():
    from agv_mission.offload import ArchiveWriter
    written=[]
    class Handle:
        def write(self,text):written.append(text)
        def flush(self):written.append('flush')
        def close(self):written.append('close')
    writer=ArchiveWriter(flush_interval_s=10.);handle=writer.track(Handle())
    for i in range(200):writer.append(handle,i)
    assert writer.close()
    # Draining, then flushing, then closing: no ordering leaves a record unwritten.
    assert written==list(range(200))+['flush','close']


def test_idle_separates_handed_over_from_written_and_flushed():
    import time
    from agv_mission.offload import ArchiveWriter
    written=[]
    class Handle:
        def write(self,text):written.append(text)
        def flush(self):written.append('flush')
        def close(self):written.append('close')
    # A ten second interval: without sync, nothing reaches the file on its own.
    writer=ArchiveWriter(flush_interval_s=10.);handle=writer.track(Handle())
    try:
        assert writer.idle,'an empty writer is idle'
        writer.append(handle,'row')
        deadline=time.monotonic()+2
        while writer.idle and time.monotonic()<deadline:time.sleep(.005)
        assert not writer.idle,'a queued write must not read as idle'
        # Written but unflushed still is not idle: the evidence is in a buffer.
        deadline=time.monotonic()+2
        while written[:1]!=['row'] and time.monotonic()<deadline:time.sleep(.005)
        assert written==['row'] and not writer.idle
        writer.sync()
        deadline=time.monotonic()+2
        while not writer.idle and time.monotonic()<deadline:time.sleep(.005)
        assert writer.idle and written==['row','flush']
    finally:assert writer.close()


def test_a_write_submitted_during_a_sync_is_not_left_behind_by_idle():
    import time
    from agv_mission.offload import ArchiveWriter
    written=[]
    class Handle:
        def write(self,text):written.append(text)
        def flush(self):pass
        def close(self):pass
    writer=ArchiveWriter(flush_interval_s=10.);handle=writer.track(Handle())
    try:
        for i in range(200):writer.append(handle,i)
        writer.sync()
        deadline=time.monotonic()+3
        while not writer.idle and time.monotonic()<deadline:time.sleep(.005)
        # Idle is only allowed to be true once every one of them is on the file.
        assert writer.idle and written==list(range(200))
    finally:assert writer.close()
