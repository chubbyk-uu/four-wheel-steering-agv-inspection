"""Work moved off a control thread, because both of its blocking calls measured 0.35 s.

Two distinct stalls were recorded in one 50 Hz control loop, each with a single
voluntary wait, no CPU, no page faults and no scheduler preemption: 0.328 s
inside a best-effort telemetry publish, and 0.349 s inside a log flush. Kernel
pressure accounting named the shared cause -- a system-wide I/O stall
(`io_full` 0.104 s) that blocks whichever call the loop happens to make.

Telemetry may be dropped when the consumer falls behind; the archive may not,
because it is the evidence record, so its backlog and failures are reported
instead.
"""
import threading
from collections import deque
from time import monotonic


class TelemetryPublisher:
    def __init__(self,publisher,build,depth=64):
        self.publisher=publisher;self.build=build
        self.queue=deque(maxlen=depth);self.dropped=0;self.errors=0;self.last_error=''
        self.signal=threading.Condition();self.closed=False
        self.thread=threading.Thread(target=self.run,name='telemetry',daemon=True);self.thread.start()

    def offer(self,payload):
        """Never blocks and never raises; the oldest sample is dropped when behind."""
        with self.signal:
            if len(self.queue)==self.queue.maxlen:self.dropped+=1
            self.queue.append(payload);self.signal.notify()

    def run(self):
        while True:
            with self.signal:
                while not self.queue and not self.closed:self.signal.wait()
                if not self.queue:return
                payload=self.queue.popleft()
            try:self.publisher.publish(self.build(payload))
            except Exception as exc:
                # A display or a shutdown race must not kill the owning process,
                # but a silent drop would hide a broken telemetry path.
                self.errors+=1;self.last_error=repr(exc)

    def close(self,timeout=2.):
        """Must precede node destruction: the writer thread holds the publisher."""
        with self.signal:self.closed=True;self.signal.notify()
        self.thread.join(timeout)
        return not self.thread.is_alive()


class ArchiveWriter:
    def __init__(self,flush_interval_s=1.):
        self.interval=flush_interval_s;self.pending=deque();self.handles=[]
        self.errors=0;self.last_error='';self.peak=0;self.flushed=monotonic()
        # `dirty` is what separates "handed over" from "on the file". A caller that
        # must not claim success before the evidence lands asks for `sync` and then
        # watches `idle`, instead of blocking on a thread it was moved off.
        self.dirty=False;self.syncing=False
        self.signal=threading.Condition();self.closed=False
        self.thread=threading.Thread(target=self.run,name='archive',daemon=True);self.thread.start()

    def track(self,handle):
        """Hand an open file over. The writer owns its flushing and its closing,
        so no caller can close it while a queued write is still in flight."""
        self.handles.append(handle);return handle

    def submit(self,action):
        """Hand over one write. Never blocks; the backlog is bounded only by the stall."""
        with self.signal:
            self.pending.append(action);self.peak=max(self.peak,len(self.pending))
            self.signal.notify()

    def append(self,handle,text):self.submit(lambda:handle.write(text))

    def sync(self):
        """Ask for an immediate drain and flush; cheap enough to call every tick."""
        with self.signal:self.syncing=True;self.signal.notify()

    @property
    def idle(self):
        """Everything handed over has been written and flushed to its file."""
        with self.signal:return not self.pending and not self.dirty

    def guard(self,action):
        try:action()
        except Exception as exc:self.errors+=1;self.last_error=repr(exc)

    def flush(self,force=False):
        if not force and monotonic()-self.flushed<self.interval:return
        self.flushed=monotonic()
        for handle in self.handles:self.guard(handle.flush)
        self.dirty=False

    def run(self):
        while True:
            with self.signal:
                if not self.pending and not self.closed and not self.syncing:self.signal.wait(self.interval)
                actions=list(self.pending);self.pending.clear();closing=self.closed
                syncing=self.syncing;self.syncing=False
                # Marked under the same lock that empties the queue, so no reader
                # can see an empty queue and a clean flag while a write is pending.
                if actions:self.dirty=True
            for action in actions:self.guard(action)
            self.flush(force=closing or syncing)
            if closing:
                with self.signal:
                    if not self.pending:return

    def close(self,timeout=10.):
        """Drains, then closes the tracked files: an unwritten record is a lost record."""
        with self.signal:self.closed=True;self.signal.notify()
        self.thread.join(timeout)
        if self.thread.is_alive():return False
        for handle in self.handles:self.guard(handle.close)
        return True
