#!/usr/bin/env python3
"""Cross-check SDF/Ogre mesh import against independent OBJ/OptiX traversal."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/agv_linescan'))
from agv_linescan.shared_scene import validate,digest


def main():
    p=argparse.ArgumentParser();p.add_argument('--manifest',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--optix-build',type=Path,default=Path('/tmp/agv_shared_optix_build'))
    a=p.parse_args();manifest=a.manifest.resolve();m=validate(manifest);out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,GALLIUM_DRIVER='d3d12',MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA')
    def run(name,command,environment=None):
        with (out/(name+'.log')).open('w') as f:subprocess.run(command,env=environment,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=120)
    run('ogre',['ros2','run','agv_linescan','validate_shared_ogre',str(manifest.parent/m['world']),str(out/'ogre.json')],env)
    run('optix',['bash',str(root/'tools/with_optix_runtime.sh'),str(a.optix_build/'bench'),str(a.optix_build/'scan.ptx'),str(out/'optix'),'1','256',str(manifest),str(out/'ogre.json')])
    gz=json.loads((out/'ogre.json').read_text())['queries'];rt=json.loads((out/'optix/query_results.json').read_text())
    assert len(gz)==len(rt) and len(gz)>100
    errors=[];blocked=0;unblocked=0;objects=set()
    for i,(g,r) in enumerate(zip(gz,rt)):
        assert (g['distance']>=0)==(r['distance']>=0),(i,'visibility mismatch',g,r)
        assert g['object']==r['object'],(i,'object mismatch',g,r)
        if g['distance']>=0:
            errors.append(abs(g['distance']-r['distance']));objects.add(g['object'])
        if g['kind']=='led_visibility':
            blocked+=g['distance']>=0;unblocked+=g['distance']<0
    assert max(errors)<.00005,('50 micrometer geometric budget exceeded',max(errors))
    assert blocked and unblocked and 'terrain' in objects
    assert any(n.startswith('screen') for n in objects) and any(n.startswith('canopy') for n in objects)
    run('physics',['gz','sim','-s','-r','--iterations','20',str(manifest.parent/m['world'])],dict(env,GZ_PARTITION='agv_shared_'+str(os.getpid())))
    physics_log=(out/'physics.log').read_text()
    assert '[Err]' not in physics_log and 'Segmentation fault' not in physics_log,physics_log[-2000:]
    # Ensure neither process changed the shared asset bundle.
    validate(manifest)
    report=dict(passed=True,scope='static shared geometry; Ogre2 CPU RayQuery vs OptiX; no photometric/dynamic/physics-contact acceptance',
        manifest=str(manifest),manifest_sha256=digest(manifest),queries=len(gz),
        triangles=sum(x['triangles'] for x in m['assets']),max_hit_distance_error_m=max(errors),
        physics_import_steps=20,physics_contact_dynamics_tested=False,
        hit_objects=sorted(objects),blocked_led_paths=blocked,clear_led_paths=unblocked,
        geometry_tolerance_m=.00005,full_acceptance_passed=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

if __name__=='__main__':main()
