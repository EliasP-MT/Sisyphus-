#!/usr/bin/env python3
"""Extract fixed CAD groups and fit their assembly frame to shared URDF base meshes."""
import argparse
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from inspect_cad_assembly import load, name
from import_surface_cad import convert


def vertices(shape):
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from OCP.TopLoc import TopLoc_Location
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    BRepMesh_IncrementalMesh(shape, .1, False, .1, True).Perform()
    e = TopExp_Explorer(shape, TopAbs_FACE)
    points = []
    while e.More():
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(TopoDS.Face_s(e.Current()), location)
        if mesh:
            for i in range(1, mesh.NbNodes()+1):
                point = mesh.Node(i).Transformed(location.Transformation())
                points.append([point.Coord(j)/1000 for j in (1, 2, 3)])
        e.Next()
    return np.unique(np.round(points, 8), axis=0)


def urdf_points(urdf, link_name):
    root = ET.parse(urdf).getroot()
    parents = {j.find('child').get('link'): j for j in root.findall('joint')}
    pose = np.eye(4)
    link = link_name
    while link != 'base_link':
        j = parents[link]
        if j.get('type') != 'fixed':
            raise ValueError('Alignment reference must be a fixed base part')
        origin = j.find('origin')
        t = np.eye(4)
        if origin is not None:
            t[:3, :3] = Rotation.from_euler('xyz', list(map(float, origin.get('rpy', '0 0 0').split()))).as_matrix()
            t[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
        pose = t @ pose
        link = j.find('parent').get('link')
    visual = root.find(f"link[@name='{link_name}']/visual")
    mesh = visual.find('geometry/mesh')
    raw = (urdf.parent/mesh.get('filename')).read_bytes()
    count = int.from_bytes(raw[80:84], 'little')
    dtype = np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
    if len(raw) != 84+50*count:
        raise ValueError('Expected binary STL')
    points = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['v'].reshape(-1,3).astype(float)
    points *= np.array(list(map(float, mesh.get('scale','1 1 1').split())))
    origin = visual.find('origin')
    if origin is not None:
        r = Rotation.from_euler('xyz', list(map(float, origin.get('rpy','0 0 0').split()))).as_matrix()
        points = points @ r.T + np.array(list(map(float, origin.get('xyz','0 0 0').split())))
    return np.unique(np.round(points @ pose[:3,:3].T+pose[:3,3],8),axis=0)


def fit(cad, model):
    # Each part remains separately matched; base symmetry cannot swap servo/base.
    trees = [cKDTree(p) for p in model]
    best = None
    for permutation in itertools.permutations(range(3)):
        for signs in itertools.product((-1,1),repeat=3):
            r = np.eye(3)[list(permutation)]*np.array(signs)[:,None]
            if np.linalg.det(r)<0: continue
            t = np.mean(np.vstack(model),axis=0)-np.mean(np.vstack(cad)@r.T,axis=0)
            for _ in range(30):
                moved = [p@r.T+t for p in cad]
                matches = [m[tree.query(p)[1]] for tree,m,p in zip(trees,model,moved)]
                x,y = np.vstack(cad),np.vstack(matches)
                xc,yc=x.mean(axis=0),y.mean(axis=0)
                u,_,vt=np.linalg.svd((x-xc).T@(y-yc))
                correction=np.eye(3);correction[-1,-1]=np.linalg.det(vt.T@u.T)
                r=vt.T@correction@u.T;t=yc-xc@r.T
            distances=np.concatenate([tree.query(p@r.T+t)[0] for tree,p in zip(trees,cad)])
            score=float(np.sqrt(np.mean(distances**2)))
            if best is None or score<best[0]:best=(score,float(distances.max()),r,t)
    return best


def main():
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    urdf=root/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'
    doc,tool=load(args.source)
    roots=TDF_LabelSequence();tool.GetFreeShapes(roots)
    if roots.Length()!=1:raise ValueError('Expected one assembly root')
    def target(label):
        ref=TDF_Label()
        return ref if tool.GetReferredShape_s(label,ref) else label
    top=target(roots.Value(1));children=TDF_LabelSequence();tool.GetComponents_s(top,children)
    groups={name(target(children.Value(i))):children.Value(i) for i in range(1,children.Length()+1)}
    fixed=['DISPENSER Letter box','etage2_box_elias','REF LeKiwi']
    expected=set(fixed+['REF ElRobot (NormaCore arm)'])
    if set(groups)!=expected:raise ValueError('Unexpected assembly groups: '+str(list(groups)))
    arm=target(groups['REF ElRobot (NormaCore arm)']);parts=TDF_LabelSequence();tool.GetComponents_s(arm,parts)
    references={name(target(parts.Value(i))):parts.Value(i) for i in range(1,parts.Length()+1)}
    names=['base_link','ST3215_1_v1_1']
    cad=[vertices(tool.GetShape_s(references[n])) for n in names]
    model=[urdf_points(urdf,n) for n in names]
    rms,maximum,r,t=fit(cad,model)
    print('Registration RMS/max metres',rms,maximum,'rotation',r.tolist(),'translation',t.tolist(),flush=True)
    if rms>.001 or maximum>.01:raise ValueError('Base geometry alignment is insufficient even for preview; refusing scene creation')
    meshes=[]
    for i,n in enumerate(fixed):
        out=args.output/f'fixed_{i}'
        convert(args.source,out,shape=tool.GetShape_s(groups[n]))
        meshes.append(dict(name=n,mesh=str((out/'assembly.obj').resolve()),xyz_m=t.tolist(),rpy_rad=Rotation.from_matrix(r).as_euler('xyz').tolist()))
    scene=json.loads((root/'robot_scene.example.json').read_text());scene.pop('cad')
    scene.update(surface_meshes=meshes,hardware_verified=False,
                 registration=dict(method='shared named base geometry ICP',rms_m=rms,max_m=maximum,
                                   within_execution_tolerance=maximum<=.003,
                                   references=names,excluded_CAD_arm='REF ElRobot (NormaCore arm)'))
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/'rover-scene.local.json').open('w') as stream:json.dump(scene,stream,indent=2)
    print('Created unverified scene:',args.output/'rover-scene.local.json',flush=True)


if __name__=='__main__':main()
