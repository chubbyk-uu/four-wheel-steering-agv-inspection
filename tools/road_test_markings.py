"""Metric convex paint polygons shared by display assets and CUDA recipes."""
import numpy as np


def polygons():
    result=[]
    def add(points,color):
        result.append(dict(vertices=points,color=color))
    for x,y,d in [(4.5,-2.4,1),(14.2,-2.4,1),(5.8,2.4,-1),(15.5,2.4,-1)]:
        def arrow(points):
            p=[[x+d*a,y+d*b] for a,b in points]
            add(p,'white')
        arrow([[-1.2,-.16],[.25,-.16],[.25,.16],[-1.2,.16]])
        arrow([[.15,-.6],[1.2,0],[.15,.6]])
    # Box occupies one lane; diagonal strips are clipped at its border.
    xmin,xmax,ymin,ymax=8.8,12.2,-4.05,-.6
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
