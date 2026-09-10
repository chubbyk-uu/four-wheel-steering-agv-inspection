"""Bounded callback timing, including DDS take/publish work, with no hot-path I/O."""
from collections import deque
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
