"""Camera-only fixed calibration trials, expressed to users in body axes."""
from copy import deepcopy
import math
import numpy as np


def cases(base):
    # Nominal optical X=body Y, optical Y=body X, optical Z=-body Z.
    # T_hat=T_true*Delta: convert body-axis vectors into nominal optical axes.
    body_to_optical=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
    baseline=deepcopy(base)
    baseline['calibration']['camera_translation_m']=[0.,0.,0.]
    baseline['calibration']['camera_rotvec_rad']=[0.,0.,0.]
    yield 'baseline',baseline,dict(kind='baseline',frame='body',value=0.)
    for kind,values,key,scale in (
            ('translation_mm',(1,3,5),'camera_translation_m',.001),
            ('rotation_deg',(.05,.1,.2),'camera_rotvec_rad',math.pi/180)):
        for axis in range(3):
            for value in values:
                for sign in (-1,1):
                    vector=np.zeros(3);vector[axis]=sign*value*scale
                    config=deepcopy(baseline)
                    config['calibration'][key]=(body_to_optical@vector).tolist()
                    name=f'{kind}_{"xyz"[axis]}_{"plus" if sign>0 else "minus"}_{value:g}'
                    yield name,config,dict(kind=kind,frame='body',axis='xyz'[axis],value=sign*value)
