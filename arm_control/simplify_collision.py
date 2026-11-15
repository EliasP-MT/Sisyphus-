#!/usr/bin/env python3
"""Conservative collision proxies: link boxes and merged surface-cell boxes."""
import argparse
import itertools
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull


def mesh_vertices(path):
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], 'little')
    dtype = np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')])
    if len(raw) != 84+50*count:
        raise ValueError('Expected binary STL for collision proxy')
    return np.frombuffer(raw,dtype=dtype,count=count,offset=84)['v'].reshape(-1,3).astype(float)


def arm_boxes(urdf, output):
    root = ET.parse(urdf).getroot()
    for element in root.iter('mesh'):
        element.set('filename', str((urdf.parent/element.get('filename')).resolve()))
    preserve={id(c) for link in root.findall('link') if link.get('name')=='Joint_01_1'
              for c in link.findall('collision')}
    for collision in root.iter('collision'):
        if id(collision) in preserve:
            continue  # Tight base-mount clearance; a link AABB would fill its recess.
        geometry = collision.find('geometry')
        mesh = geometry.find('mesh')
        if mesh is None:
            continue
        points = mesh_vertices(Path(mesh.get('filename'))) * np.array(list(map(float,mesh.get('scale','1 1 1').split())))
        lo,hi=points.min(axis=0),points.max(axis=0)
        origin=collision.find('origin')
        if origin is None:origin=ET.SubElement(collision,'origin')
        r=Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split())))
        xyz=np.array(list(map(float,origin.get('xyz','0 0 0').split())))+r.apply((lo+hi)/2)
        origin.set('xyz',' '.join(map(str,xyz)))
        geometry.remove(mesh)
        ET.SubElement(geometry,'box',size=' '.join(map(str,np.maximum(hi-lo,1e-5))))
    ET.ElementTree(root).write(output)


def arm_hulls(urdf, output):
    root=ET.parse(urdf).getroot()
    for element in root.iter('mesh'):
        element.set('filename',str((urdf.parent/element.get('filename')).resolve()))
    for index,collision in enumerate(root.iter('collision')):
        mesh=collision.find('geometry/mesh')
        if mesh is None:continue
        points=np.unique(mesh_vertices(Path(mesh.get('filename'))),axis=0)
        hull=ConvexHull(points)
        used=hull.vertices;lookup={old:i+1 for i,old in enumerate(used)}
        path=output.parent/f'hull-{index}.obj'
        with path.open('w') as stream:
            for point in points[used]:stream.write('v '+' '.join(map(str,point))+'\n')
            for triangle,equation in zip(hull.simplices,hull.equations):
                a,b,c=triangle
                if np.dot(np.cross(points[b]-points[a],points[c]-points[a]),equation[:3])<0:b,c=c,b
                stream.write(f'f {lookup[a]} {lookup[b]} {lookup[c]}\n')
        mesh.set('filename',str(path))
    ET.ElementTree(root).write(output)


def surface_cells(path, cell):
    vertices=[];occupied=set()
    for line in path.read_text().splitlines():
        fields=line.split()
        if fields and fields[0]=='v':vertices.append(list(map(float,fields[1:4])))
        elif fields and fields[0]=='f':
            triangle=np.array([vertices[int(v.split('/')[0])-1] for v in fields[1:4]])
            lo=np.floor(triangle.min(axis=0)/cell).astype(int)
            hi=np.floor(triangle.max(axis=0)/cell).astype(int)
            candidates=np.array(list(itertools.product(*(range(lo[i],hi[i]+1) for i in range(3)))))
            if len(candidates)>1:
                # Triangle/box separating-axis test: covers triangle interiors
                # without filling empty corners of each triangle's AABB.
                edges=np.roll(triangle,-1,axis=0)-triangle
                axes=[*np.eye(3),np.cross(edges[0],edges[1])]
                axes.extend(np.cross(edge,axis) for edge in edges for axis in np.eye(3))
                centers=(candidates+.5)*cell
                keep=np.ones(len(candidates),dtype=bool)
                for axis in axes:
                    if np.dot(axis,axis)<1e-24:continue
                    projection=triangle@axis;radius=cell/2*np.abs(axis).sum()
                    middle=centers@axis
                    keep&=(projection.min()<=middle+radius+1e-12)&(projection.max()>=middle-radius-1e-12)
                candidates=candidates[keep]
            occupied.update(map(tuple,candidates))
    return occupied


def merge_cells(cells):
    cells=set(cells);boxes=[]
    while cells:
        x,y,z=min(cells);xx=x+1
        while (xx,y,z) in cells:xx+=1
        yy=y+1
        while all((i,yy,z) in cells for i in range(x,xx)):yy+=1
        zz=z+1
        while all((i,j,zz) in cells for i in range(x,xx) for j in range(y,yy)):zz+=1
        for i in range(x,xx):
            for j in range(y,yy):
                for k in range(z,zz):cells.remove((i,j,k))
        boxes.append((np.array([x,y,z]),np.array([xx,yy,zz])))
    return boxes


def simplify(source, output, cell=.005, arm_proxy=False, retain_chassis=False):
    if not .002<=cell<=.03:raise ValueError('Cell size must be 2–30 mm')
    scene=json.loads(source.read_text());entries=scene.pop('surface_meshes')
    scene['display_surface_meshes']=entries
    if arm_proxy:
        scene['arm_collision_proxy']='link_boxes'
    else:
        scene['arm_collision_proxy']='convex_hulls'
    scene['proxy_boxes']=[]
    counts={}
    for entry in entries:
        if retain_chassis and entry['name']=='REF LeKiwi':
            scene.setdefault('surface_meshes',[]).append(entry)
            continue
        mesh=(source.parent/entry['mesh']).resolve()
        cells=surface_cells(mesh,cell)
        merged=merge_cells(cells)
        counts[entry['name']]=dict(occupied_cells=len(cells),boxes=len(merged))
        rotation=Rotation.from_euler('xyz',entry['rpy_rad'])
        for lo,hi in merged:
            scene['proxy_boxes'].append(dict(name=entry['name'],size_m=((hi-lo)*cell).tolist(),
                xyz_m=(rotation.apply((lo+hi)*cell/2)+entry['xyz_m']).tolist(),rpy_rad=entry['rpy_rad']))
    scene['simplification']=dict(method='triangle-box SAT cells merged into boxes',cell_m=cell,counts=counts,
                                 conservative=True,source_scene=str(source.resolve()))
    scene['hardware_verified']=False
    with output.open('x') as stream:json.dump(scene,stream,indent=2)
    print(json.dumps(scene['simplification'],indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cell',type=float,default=.005)
    parser.add_argument('--arm-boxes',action='store_true',help='Conservative but may cause false collisions near the mount')
    parser.add_argument('--retain-chassis',action='store_true',help='Keep detailed mounting clearances')
    args=parser.parse_args();simplify(args.source,args.output,args.cell,args.arm_boxes,args.retain_chassis)
