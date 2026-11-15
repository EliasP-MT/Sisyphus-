#!/usr/bin/env python3
"""Import every STEP face for assembly inspection, including open shells.

Output is a static assembly preview, NOT an articulated collision scene.
"""
import argparse
import json
from pathlib import Path

from cad import sha256


def convert(source, output, shape=None):
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    if shape is None:
        reader = STEPControl_Reader()
        if reader.ReadFile(str(source)) != IFSelect_RetDone:
            raise ValueError('STEP import failed')
        reader.SetSystemLengthUnit(1.0)
        if reader.TransferRoots() < 1:
            raise ValueError('No assembly geometry')
        shape = reader.OneShape()
    print('Assembly transferred; triangulating every face', flush=True)
    BRepMesh_IncrementalMesh(shape, .2, False, .1, True).Perform()
    output.mkdir(parents=True, exist_ok=True)
    destination = output/'assembly.obj'
    temporary = output/'assembly.obj.tmp'
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    offset = triangles = count = degenerate = 0
    low, high = [float('inf')]*3, [-float('inf')]*3
    with temporary.open('w') as stream:
        stream.write('# Full STEP assembly, metres, original assembly frame\n')
        while faces.More():
            face = TopoDS.Face_s(faces.Current())
            location = TopLoc_Location()
            mesh = BRep_Tool.Triangulation_s(face, location)
            if mesh is None:
                BRepMesh_IncrementalMesh(face, .02, False, .05, False).Perform()
                mesh = BRep_Tool.Triangulation_s(face, location)
            if mesh is None:
                properties = GProp_GProps()
                BRepGProp.SurfaceProperties_s(face, properties)
                if abs(properties.Mass()) < 1e-10:  # mm²: no physical surface.
                    degenerate += 1
                    count += 1
                    faces.Next()
                    continue
                raise ValueError(f'Unmeshed face {count}, area {properties.Mass()} mm²; refusing incomplete export')
            for index in range(1, mesh.NbNodes()+1):
                point = mesh.Node(index).Transformed(location.Transformation())
                xyz = [point.Coord(i)/1000 for i in (1, 2, 3)]
                low = [min(a, b) for a, b in zip(low, xyz)]
                high = [max(a, b) for a, b in zip(high, xyz)]
                stream.write('v ' + ' '.join(f'{x:.10g}' for x in xyz) + '\n')
            for index in range(1, mesh.NbTriangles()+1):
                a, b, c = mesh.Triangle(index).Get()
                if face.Orientation() == TopAbs_REVERSED:
                    b, c = c, b
                stream.write(f'f {offset+a} {offset+b} {offset+c}\n')
            offset += mesh.NbNodes()
            triangles += mesh.NbTriangles()
            count += 1
            faces.Next()
    temporary.replace(destination)
    report = dict(schema='surface-assembly-preview-v1', source=str(source.resolve()),
                  source_sha256=sha256(source), mesh_sha256=sha256(destination),
                  faces=count, triangles=triangles, vertices=offset,
                  zero_area_faces=degenerate,
                  bounds_m=[low, high], units='metres', hardware_verified=False,
                  limitation='Static assembly; articulation and arm-base registration required')
    with (output/'assembly.json').open('w') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    convert(args.source, args.output)
