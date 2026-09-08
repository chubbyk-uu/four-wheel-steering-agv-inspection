"""Two opposing lanes; marking and slab-joint coordinates remain independent."""
import numpy as np
LINE_WIDTH=.15
YELLOW_CENTERS=(-.15,.15)
EDGE_INSET=.5

def masks(xs,ys,width):
    x,y=np.meshgrid(np.asarray(xs),np.asarray(ys))
    yellow=np.abs(np.abs(y)-.15)<=LINE_WIDTH/2
    white=np.abs(np.abs(y)-(width/2-EDGE_INSET))<=LINE_WIDTH/2
    return yellow,white

def paint(rgb,xs,ys,width):
    yellow,white=masks(xs,ys,width)
    # Linear RGB pigment approximations, retaining a small amount of substrate texture.
    rgb[yellow]=np.array([.80,.56,.015],np.float32)+.06*rgb[yellow]
    rgb[white]=.72+.12*rgb[white]
    return yellow|white

def metadata(width):
    return dict(layout='two_opposing_lanes',center=dict(color='yellow',type='double_solid',
                line_width_m=LINE_WIDTH,centers_y_m=list(YELLOW_CENTERS),clear_gap_m=.15),
                edges=dict(color='white',type='solid',line_width_m=LINE_WIDTH,
                centers_y_m=[-width/2+EDGE_INSET,width/2-EDGE_INSET],center_inset_m=EDGE_INSET),
                scope='project road example informed by public engineering documents; not regulatory certification',
                longitudinal_joint_relation='center joint at y=0 lies inside yellow-line gap; transverse crossings allowed')
