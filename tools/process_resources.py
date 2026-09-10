"""Sample an owned process tree and whole-device GPU usage outside ROS callbacks."""
import json
import subprocess
import threading
import time
from pathlib import Path
import psutil


class ResourceMonitor:
    def __init__(self,pid,output,interval=2.):
        self.pid=pid;self.output=Path(output);self.interval=interval;self.stop=threading.Event();self.samples=[];self.errors=[];self.owned={}
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def run(self):
        start=time.monotonic()
        with self.output.open('x') as stream:
            while not self.stop.is_set():
                try:
                    try:
                        parent=psutil.Process(self.pid);processes=[parent]+parent.children(recursive=True)
                    except psutil.NoSuchProcess:
                        break  # Normal end of the owned launch process.
                    self.owned.update({p.pid:p for p in processes})
                    rss=0;pss=0
                    for process in processes:
                        try:
                            m=process.memory_full_info();rss+=m.rss;pss+=m.pss
                        except psutil.NoSuchProcess:continue
                    gpu=subprocess.run(['nvidia-smi','--query-gpu=memory.used','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=3,check=True)
                    sample=dict(wall_elapsed_s=time.monotonic()-start,tree_rss_bytes=rss,tree_pss_bytes=pss,
                                device_used_bytes=max(int(v) for v in gpu.stdout.split())*2**20,process_count=len(processes))
                    self.samples.append(sample);stream.write(json.dumps(sample)+'\n');stream.flush()
                except Exception as exc:self.errors.append(type(exc).__name__+': '+str(exc))
                self.stop.wait(self.interval)

    def close(self):
        self.stop.set();self.thread.join(timeout=5)
        result=dict(interval_s=self.interval,samples=len(self.samples),errors=self.errors,
                    scope='Owned Linux process tree RSS (shared pages may repeat) and PSS; GPU usage is whole device, includes desktop; sampled peaks only')
        for key in ('tree_rss_bytes','tree_pss_bytes','device_used_bytes'):
            result['peak_'+key]=max((s[key] for s in self.samples),default=None)
        self.output.with_suffix('.summary.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
