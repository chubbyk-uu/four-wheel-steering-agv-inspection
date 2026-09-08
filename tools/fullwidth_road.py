"""Shared full-width road features; AI crack fields supply both optics and mesh."""
import math
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial import Delaunay
import cv2
from branch_crack_fixture import branch_fields, STEP
from bake_concrete_road import sha


class Features:
    def __init__(self,length):
        self.length=length
        self.field=branch_fields()
        h,w=self.field['alpha'].shape
        self.span=np.array([w*STEP,h*STEP])
        # Placement only, no procedural crack paths; not every slab is cracked.
        self.instances=[dict(center=[float(x),float(y)],flip_x=bool(i%2),flip_y=bool(j%2))
            for i,x in enumerate(np.arange(2.5,length,10)) for j,y in enumerate((-2.5,2.5))]
        self.support=binary_dilation(self.field['alpha']>0,iterations=2)
        self.near=binary_dilation(self.field['alpha']>0,iterations=3)

    def arrays(self,instance,key):
        data=self.field[key]
        if instance['flip_x']:data=data[:,::-1]
        if instance['flip_y']:data=data[::-1]
        return data

    def apply(self,rgb,n,xs,ys):
        for instance in self.instances:
            origin=np.array(instance['center'])-self.span/2
            xi=np.flatnonzero((xs>=origin[0]-STEP)&(xs<=origin[0]+self.span[0]+STEP))
            yi=np.flatnonzero((ys>=origin[1]-STEP)&(ys<=origin[1]+self.span[1]+STEP))
            if not len(xi) or not len(yi):continue
            sl=np.s_[yi[0]:yi[-1]+1,xi[0]:xi[-1]+1]
            xx,yy=np.meshgrid(np.float32((xs[xi]-origin[0])/STEP-.5),np.float32((ys[yi]-origin[1])/STEP-.5))
            for key in ('pigment','normal_strength'):
                values=cv2.remap(self.arrays(instance,key),xx,yy,cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,borderValue=1)
                if key=='pigment':rgb[sl]*=values[:,:,None]
                else:n[sl][:,:,:2]*=values[:,:,None]
        n/=np.linalg.norm(n,axis=2)[:,:,None]
        return rgb,n

    def geometry(self,x0,x1,y0,y1):
        xx,yy=np.meshgrid(np.linspace(x0,x1,math.ceil((x1-x0)*10)+1),
                          np.linspace(y0,y1,math.ceil((y1-y0)*10)+1))
        base=np.column_stack((xx.ravel(),yy.ravel(),np.zeros(xx.size)))
        parts=[]
        for instance in self.instances:
            origin=np.array(instance['center'])-self.span/2
            if origin[0]+self.span[0]<x0 or origin[0]>x1 or origin[1]+self.span[1]<y0 or origin[1]>y1:continue
            # Current placements lie wholly within the 5m slab partitions.
            if not (x0<origin[0] and origin[0]+self.span[0]<x1 and y0<origin[1] and origin[1]+self.span[1]<y1):
                raise ValueError('crack crosses mesh partition; split its shared geometry before moving placement')
            support=self.support
            near=self.near
            if instance['flip_x']:support=support[:,::-1];near=near[:,::-1]
            if instance['flip_y']:support=support[::-1];near=near[::-1]
            iy,ix=np.nonzero(support)
            z=self.arrays(instance,'depth')[iy,ix]
            parts.append(np.column_stack((origin[0]+(ix+.5)*STEP,origin[1]+(iy+.5)*STEP,z)))
            bx=np.floor((base[:,0]-origin[0])/STEP).astype(int);by=np.floor((base[:,1]-origin[1])/STEP).astype(int)
            valid=(bx>=0)&(bx<near.shape[1])&(by>=0)&(by<near.shape[0])
            remove=np.zeros(len(base),bool);remove[valid]=near[by[valid],bx[valid]]
            base=base[~remove]
        parts.append(base)
        # 8mm board joints, 3mm depth, 1mm edge bevel; same heights on partition boundaries.
        for seam in np.arange(0,self.length+.01,5):
            cuts=seam+np.array([-.004,-.003,0,.003,.004])
            cuts=cuts[(cuts>=x0)&(cuts<=x1)]
            if len(cuts):
                x,y=np.meshgrid(cuts,np.linspace(y0,y1,math.ceil((y1-y0)*50)+1))
                parts.append(np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size))))
        for seam in (-5,0,5):
            cuts=seam+np.array([-.004,-.003,0,.003,.004])
            cuts=cuts[(cuts>=y0)&(cuts<=y1)]
            if len(cuts):
                y,x=np.meshgrid(cuts,np.linspace(x0,x1,math.ceil((x1-x0)*50)+1))
                parts.append(np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size))))
        v=np.vstack(parts);_,ids=np.unique(v[:,:2],axis=0,return_index=True);v=v[ids]
        dx=np.min(np.abs(v[:,0,None]-np.arange(0,self.length+.01,5)),axis=1)
        dy=np.min(np.abs(v[:,1,None]-np.array([-5,0,5])),axis=1)
        v[:,2]=np.minimum(v[:,2],-.003*np.clip((.004-np.minimum(dx,dy))/.001,0,1))
        f=Delaunay(v[:,:2]).simplices
        return v,f


def collision_mesh(out,stem,v,f):
    # Checked proxy policy is shared with the single-trial simplifier.
    from agv_linescan.collision_proxy import shallow_rectangle
    lo,hi,fraction=shallow_rectangle(v,f,0,.003)
    path=Path(out)/(stem+'_collision.obj')
    with path.open('w') as stream:
        np.savetxt(stream,[[lo[0],lo[1],0],[hi[0],lo[1],0],[hi[0],hi[1],0],[lo[0],hi[1],0]],fmt='v %.9f %.9f %.9f')
        stream.write('vn 0 0 1\nf 1//1 2//1 3//1\nf 1//1 3//1 4//1\n')
    return dict(method='shallow_horizontal_rectangle_v1',mesh=path.name,sha256=sha(path),
                plane_z_m=0,max_surface_deviation_m=.003,omitted_depression_area_fraction=fraction,triangles=2)
