#!/usr/bin/env python3
"""Export only selected robot runtime data; never Station keys or credentials."""
import argparse
import copy
import hashlib
import io
import json
from pathlib import Path
import tarfile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--scene',type=Path,required=True)
    parser.add_argument('--poses',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    scene=json.loads(args.scene.read_text())
    source_path=(args.scene.parent/scene['simplification']['source_scene']).resolve()
    source=json.loads(source_path.read_text())
    files={};payloads={}
    def data(name,value):payloads[name]=json.dumps(value,indent=2).encode()
    def rewrite(entries, origin):
        result=[]
        for entry in entries:
            entry=copy.deepcopy(entry)
            mesh=(origin/entry['mesh']).resolve()
            # One shared mapping for the display and collision references.
            key=str(mesh)
            if key not in files:
                prefix=f'fixed/part_{len(files)}'
                files[key]=(prefix,mesh)
                report=json.loads(mesh.with_name('assembly.json').read_text())
                cad=(mesh.parent/report['source']).resolve()
                if 'cad' in payloads and payloads['cad']!=str(cad):
                    raise ValueError('More than one CAD source; export explicitly')
                payloads['cad']=str(cad)
                report['source']='../../cad/full_robot_box_complete.step'
                data(prefix+'/assembly.json',report)
            entry['mesh']=files[key][0]+'/assembly.obj'
            result.append(entry)
        return result
    source['surface_meshes']=rewrite(source['surface_meshes'],source_path.parent)
    data('source-scene.local.json',source)
    for key in ('surface_meshes','display_surface_meshes'):
        if key in scene:scene[key]=rewrite(scene[key],args.scene.parent)
    scene['simplification']['source_scene']='source-scene.local.json'
    scene['simplification']['comparison']['source_scene_sha256']=hashlib.sha256(payloads['source-scene.local.json']).hexdigest()
    data('scene.local.json',scene)
    data('calibration.local.json',json.loads(args.config.read_text()))
    data('poses.local.json',json.loads(args.poses.read_text()))
    cad=Path(payloads.pop('cad'))
    preview=root/'assets/full_robot_box_complete_collision'
    if (preview/'assembly.obj').exists():
        report=json.loads((preview/'assembly.json').read_text());report['source']='../cad/full_robot_box_complete.step'
        data('preview/assembly.json',report)
    with tarfile.open(args.output,'x:gz') as archive:
        for name,blob in payloads.items():
            info=tarfile.TarInfo(name);info.size=len(blob);info.mode=0o600
            archive.addfile(info,io.BytesIO(blob))
        for prefix,mesh in files.values():archive.add(mesh,arcname=prefix+'/assembly.obj',recursive=False)
        archive.add(cad,arcname='cad/full_robot_box_complete.step',recursive=False)
        if (preview/'assembly.obj').exists():archive.add(preview/'assembly.obj',arcname='preview/assembly.obj',recursive=False)
    print('Runtime bundle:',args.output)
    print('Contains robot calibration, poses and CAD only; excludes Station data, credentials, SDK and venv.')


if __name__=='__main__':main()
