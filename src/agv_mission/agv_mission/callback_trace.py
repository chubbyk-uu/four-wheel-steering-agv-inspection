"""Bounded callback timing, including DDS take/publish work, with no hot-path I/O."""
import threading
from collections import deque
from resource import RUSAGE_THREAD, getrusage
from time import monotonic, thread_time
from rclpy.executors import SingleThreadedExecutor


class CallbackTrace:
    def __init__(self):
        self.peaks={};self.slow=deque(maxlen=64)

    def record(self,name,wall,cpu):
        if name not in self.peaks and len(self.peaks)>=128:name='other'
        self.peaks[name]=max(wall,self.peaks.get(name,0.))
        if wall>=.02:self.slow.append(dict(callback=name,wall_s=wall,thread_cpu_s=cpu))

    def snapshot(self):
        return dict(max_wall_s=dict(self.peaks),recent_slow=list(self.slow))


class TimedHandler:
    def __init__(self,handler,trace,name):
        self.handler=handler;self.trace=trace;self.name=name

    def __getattr__(self,name):return getattr(self.handler,name)

    def __call__(self):
        wall=monotonic();cpu=thread_time()
        try:return self.handler()
        finally:self.trace.record(self.name,monotonic()-wall,thread_time()-cpu)


class TracedExecutor(SingleThreadedExecutor):
    def __init__(self,**kwargs):
        super().__init__(**kwargs);self.trace=CallbackTrace()

    def wait_for_ready_callbacks(self,*args,**kwargs):
        wall=monotonic();cpu=thread_time()
        try:handler,entity,node=super().wait_for_ready_callbacks(*args,**kwargs)
        finally:self.trace.record('executor:dispatch_wait',monotonic()-wall,thread_time()-cpu)
        name=getattr(entity,'topic_name',None) or getattr(entity,'srv_name',None)
        if not name:name=getattr(getattr(entity,'callback',None),'__qualname__',type(entity).__name__)
        name=(node.get_name() if node else 'executor')+':'+name
        return TimedHandler(handler,self.trace,name),entity,node


class PhaseTrace:
    """Per-tick phase timings, retained only for slow ticks so the loop stays I/O free."""
    def __init__(self,threshold=.05,maxlen=32):
        self.threshold=threshold;self.slow=deque(maxlen=maxlen);self.peaks={}
        self.marks=[];self.start=0.;self.cpu=0.;self.previous_end=None;self.stalls=0
        self.previous_usage=(0,0);self.previous_faults=0;self.gap=0.
        self.wait_points=[];self.active=False

    def begin(self):
        now=monotonic();self.wait_points=[];self.active=True
        # Gap since the previous tick returned: a blocked executor inflates every
        # arrival age measured in this tick, so the gap must be reported with them.
        self.gap=0. if self.previous_end is None else now-self.previous_end
        self.start=now;self.cpu=thread_time();self.marks=[];self.usage=getrusage(RUSAGE_THREAD)
        if self.gap>=self.threshold:
            # Reported here, not at the end: a tick that faults on its own gap must
            # carry the evidence in the same record, even if the process then exits.
            self.stalls+=1
            self.slow.append(dict(kind='gap',monotonic_s=round(now,6),gap_before_s=round(self.gap,6),
                waits=self.usage.ru_nvcsw-self.previous_usage[0],
                preemptions=self.usage.ru_nivcsw-self.previous_usage[1],
                major_faults=self.usage.ru_majflt-self.previous_faults))

    def mark(self,name):self.marks.append((name,monotonic()))

    def end(self,**context):
        now=monotonic();total=now-self.start;previous=self.start;phases={}
        for name,stamp in self.marks:
            phases[name]=round(stamp-previous,6);previous=stamp
        for name,value in phases.items():self.peaks[name]=max(value,self.peaks.get(name,0.))
        self.active=False;self.previous_end=now;usage=getrusage(RUSAGE_THREAD)
        self.previous_usage=(usage.ru_nvcsw,usage.ru_nivcsw);self.previous_faults=usage.ru_majflt
        if total>=self.threshold:
            self.stalls+=1
            # Voluntary switches mean the thread waited on something; involuntary
            # ones mean it was preempted. Wall time alone cannot tell them apart.
            self.slow.append(dict(kind='tick',monotonic_s=round(now,6),wall_s=round(total,6),gap_before_s=round(self.gap,6),
                                  thread_cpu_s=round(thread_time()-self.cpu,6),
                                  waits=usage.ru_nvcsw-self.usage.ru_nvcsw,
                                  preemptions=usage.ru_nivcsw-self.usage.ru_nivcsw,
                                  major_faults=usage.ru_majflt-self.usage.ru_majflt,
                                  phases=phases,wait_points=list(self.wait_points),**context))

    def snapshot(self):
        return dict(stalls=self.stalls,max_phase_s=dict(self.peaks),pending=list(self.slow))

    def drain(self):
        """Hand over pending stall reports exactly once, so none is only visible on FAULT."""
        if not self.slow:return None
        value=list(self.slow);self.slow.clear();return value


def _read(path):
    try:
        with open(path) as handle:return handle.read().strip()
    except OSError:return ''


class StallWatch:
    """Names the kernel wait point of a control tick that is still running.

    Wall time proves a tick blocked; only the wait channel says on what, which is
    what separates writeback throttling from waiting on a lock or a socket.
    """
    def __init__(self,probe,threshold,period=.005,samples=6):
        self.probe=probe;self.threshold=threshold;self.period=period;self.samples=samples
        self.stop=threading.Event();self.thread=None

    def start(self,tid):
        self.wchan='/proc/self/task/%d/wchan'%tid;self.call='/proc/self/task/%d/syscall'%tid
        self.thread=threading.Thread(target=self.run,name='stall_watch',daemon=True);self.thread.start()

    def sample(self,start):
        return dict(after_s=round(monotonic()-start,4),wchan=_read(self.wchan),
                    syscall=_read(self.call).split(' ')[0])

    def run(self):
        while not self.stop.wait(self.period):
            start=self.probe.start
            if not self.probe.active or monotonic()-start<self.threshold:continue
            trace=[]
            while (self.probe.active and self.probe.start==start and len(trace)<self.samples
                   and not self.stop.is_set()):
                trace.append(self.sample(start));self.stop.wait(self.period)
            if self.probe.start==start:self.probe.wait_points=trace

    def close(self,timeout=1.):
        self.stop.set()
        if self.thread is not None:self.thread.join(timeout)
