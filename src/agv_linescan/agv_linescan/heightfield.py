"""Checked piecewise planar reference heightfield for layered shallow-road proxies."""
import json
import numpy as np

class Heightfield:
    def __init__(self,path):
        m=json.loads(path.read_text());self.x=np.asarray(m['x'],float);self.y=np.asarray(m['y'],float);self.z=np.asarray(m['z'],float)
        if (m.get('schema')!='agv.reference_heightfield.v1' or self.x.ndim!=1 or self.y.ndim!=1 or min(len(self.x),len(self.y))<2 or
            self.z.shape!=(len(self.y),len(self.x)) or not all(np.isfinite(a).all() for a in (self.x,self.y,self.z)) or
            np.any(np.diff(self.x)<=0) or np.any(np.diff(self.y)<=0) or np.max(abs(self.z))>.003000001):
            raise ValueError('invalid bounded reference heightfield')
    def sample(self,xy):
        xy=np.asarray(xy)
        if xy.ndim < 1 or xy.shape[-1] != 2 or not np.isfinite(xy).all():
            raise ValueError('invalid heightfield sample coordinates')
        x,y=xy[...,0],xy[...,1]
        if np.any(x<self.x[0]-1e-8) or np.any(x>self.x[-1]+1e-8) or np.any(y<self.y[0]-1e-8) or np.any(y>self.y[-1]+1e-8):raise ValueError('heightfield sample out of bounds')
        i=np.clip(np.searchsorted(self.x,x,side='right')-1,0,len(self.x)-2);j=np.clip(np.searchsorted(self.y,y,side='right')-1,0,len(self.y)-2)
        u=np.clip((x-self.x[i])/(self.x[i+1]-self.x[i]),0,1);v=np.clip((y-self.y[j])/(self.y[j+1]-self.y[j]),0,1)
        a,b,c,d=self.z[j,i],self.z[j,i+1],self.z[j+1,i+1],self.z[j+1,i]
        return np.where(v<=u,a*(1-u)+b*(u-v)+c*v,a*(1-v)+c*u+d*(v-u))
    def mesh(self,lo,hi):
        x=np.r_[lo[0],self.x[(self.x>lo[0]+1e-9)&(self.x<hi[0]-1e-9)],hi[0]]
        y=np.r_[lo[1],self.y[(self.y>lo[1]+1e-9)&(self.y<hi[1]-1e-9)],hi[1]]
        xx,yy=np.meshgrid(x,y);v=np.column_stack((xx.ravel(),yy.ravel(),np.zeros(xx.size)));v[:,2]=self.sample(v[:,:2])
        i,j=np.meshgrid(np.arange(len(y)-1),np.arange(len(x)-1),indexing='ij');a=(i*len(x)+j).ravel()
        f=np.concatenate((np.stack((a,a+1,a+len(x)+1),1),np.stack((a,a+len(x)+1,a+len(x)),1)))
        return v,f
