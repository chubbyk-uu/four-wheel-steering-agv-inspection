"""Metric convex paint polygons shared by display assets and CUDA recipes."""
import numpy as np


def _cell_polygons(origin_x, width_m):
    result=[]
    def add(points,color):
        result.append(dict(vertices=[[x+origin_x,y] for x,y in points],color=color))
    lane_y=.24*width_m
    for x,y,d in [(4.5,-lane_y,1),(14.2,-lane_y,1),(5.8,lane_y,-1),(15.5,lane_y,-1)]:
        def arrow(points):
            p=[[x+d*a,y+d*b] for a,b in points]
            add(p,'white')
        arrow([[-1.2,-.16],[.25,-.16],[.25,.16],[-1.2,.16]])
        arrow([[.15,-.6],[1.2,0],[.15,.6]])
    # Box occupies one lane; diagonal strips are clipped at its border.
    xmin,xmax,ymin,ymax=8.8,12.2,-.405*width_m,-.06*width_m
    def clip(poly,axis,bound,sign):
        out=[]
        for p,q in zip(poly,poly[1:]+poly[:1]):
            a=sign*(p[axis]-bound)>=0;b=sign*(q[axis]-bound)>=0
            if a:out.append(p)
            if a!=b:
                t=(bound-p[axis])/(q[axis]-p[axis]);out.append([p[i]+t*(q[i]-p[i]) for i in range(2)])
        return out
    for slope in [-1,1]:
        for intercept in np.arange(-18,18,.85):
            half=.05*2**.5
            p=[[xmin-1,slope*(xmin-1)+intercept-half],[xmax+1,slope*(xmax+1)+intercept-half],
               [xmax+1,slope*(xmax+1)+intercept+half],[xmin-1,slope*(xmin-1)+intercept+half]]
            for axis,bound,sign in [(0,xmin,1),(0,xmax,-1),(1,ymin,1),(1,ymax,-1)]:
                if p:p=clip(p,axis,bound,sign)
            if len(p)>=3:add(p,'yellow')
    for a,b,c,d in [(xmin,xmax,ymin,ymin+.1),(xmin,xmax,ymax-.1,ymax),(xmin,xmin+.1,ymin,ymax),(xmax-.1,xmax,ymin,ymax)]:
        add([[a,c],[b,c],[b,d],[a,d]],'yellow')
    return result


def polygons(length_m=20.,width_m=10.):
    """Distribute fixed-size two-lane markings in repeatable 20 m cells."""
    if not np.isfinite(length_m) or not np.isfinite(width_m) or length_m<=0 or width_m<4:
        raise ValueError('marking layout requires a positive length and width >= 4 m')
    result=[]
    for origin in np.arange(0.,length_m,20.):
        for item in _cell_polygons(float(origin),width_m):
            p=np.asarray(item['vertices'])
            if np.min(p[:,0])>=0 and np.max(p[:,0])<=length_m:
                result.append(item)
    return result


def seam_crossings(items,length_m,block_length_m=1.5):
    """Return paint polygons that deliberately exercise captured-frame seams."""
    if block_length_m<=0:
        raise ValueError('block length must be positive')
    result=[]
    directions=(('forward',np.arange(block_length_m,length_m,block_length_m)),
                ('reverse',length_m-np.arange(block_length_m,length_m,block_length_m)))
    for direction,seams in directions:
        for index,item in enumerate(items):
            xs=np.asarray(item['vertices'])[:,0];hit=seams[(seams>xs.min())&(seams<xs.max())]
            result.extend(dict(direction=direction,polygon_index=index,seam_x_m=float(x)) for x in hit)
    return result


def masks(xs,ys,items):
    x=np.asarray(xs)[None,:];y=np.asarray(ys)[:,None]
    yellow=np.zeros((y.size,x.size),bool);white=yellow.copy()
    for item in items:
        p=item['vertices'];inside=np.ones_like(yellow)
        for a,b in zip(p,p[1:]+p[:1]):
            inside &= (b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])>=-1e-12
        (yellow if item['color']=='yellow' else white)[:] |= inside
    return yellow,white


def paint(rgb,xs,ys,items):
    yellow,white=masks(xs,ys,items)
    rgb[yellow]=[.80,.56,.015]+.06*rgb[yellow]
    rgb[white]=.72+.12*rgb[white]
    return yellow|white
