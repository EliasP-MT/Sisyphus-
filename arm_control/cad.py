#!/usr/bin/env python3
"""Convert STEP solids to separate, metre-scale triangle meshes using OpenCascade."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_solids(source):
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_SHELL
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError('STEP illisible par OpenCascade')
    reader.SetSystemLengthUnit(1.0)
    if reader.TransferRoots() < 1:
        raise ValueError('STEP sans géométrie transférable')
    shape = reader.OneShape()
    # Some CAD exporters represent most assembly parts as open/triangle shells.
    # Silently extracting only solids would omit those collision obstacles.
    shells = TopExp_Explorer(shape, TopAbs_SHELL, TopAbs_SOLID)
    if shells.More():
        raise ValueError('STEP contains surface shells outside closed solids; solid-only conversion would omit assembly geometry. Surface-aware import required.')
    solids = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        solids.append(TopoDS.Solid_s(explorer.Current()))
        explorer.Next()
    if not solids:
        raise ValueError('Le STEP ne contient aucun solide fermé')
    return solids


def convert_step(source, output, deflection_mm=0.2):
    from OCP.Bnd import Bnd_Box
    from OCP.BRep import BRep_Tool
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GProp import GProp_GProps
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    source, output = Path(source), Path(output)
    if not 0 < deflection_mm <= 0.5:
        raise ValueError('Déflexion de maillage requise dans ]0, 0.5] mm')
    # OCCT transfer units are explicitly millimetres, regardless of STEP source units.
    solids = read_solids(source)
    output.mkdir(parents=True, exist_ok=True)
    manifest = dict(source_sha256=sha256(source), source_name=source.name,
                    source_path=os.path.relpath(source.resolve(), output.resolve()),
                    mesh_units='metres', occt_transfer_units='millimetres',
                    mesh_deflection_m=deflection_mm/1000, solids=[])
    for i, solid in enumerate(solids):
        if not BRepCheck_Analyzer(solid).IsValid():
            raise ValueError(f'Solide {i} invalide selon OpenCascade')
        BRepMesh_IncrementalMesh(solid, deflection_mm, False, 0.1, False).Perform()
        vertices, triangles = [], []
        faces = TopExp_Explorer(solid, TopAbs_FACE)
        while faces.More():
            face = TopoDS.Face_s(faces.Current())
            location = TopLoc_Location()
            mesh = BRep_Tool.Triangulation_s(face, location)
            if mesh is None:
                raise ValueError(f'Solide {i} : face sans triangulation')
            start = len(vertices)
            for n in range(1, mesh.NbNodes()+1):
                p = mesh.Node(n).Transformed(location.Transformation())
                vertices.append(tuple(p.Coord(k)/1000 for k in (1, 2, 3)))
            for n in range(1, mesh.NbTriangles()+1):
                a, b, c = mesh.Triangle(n).Get()
                if face.Orientation() == TopAbs_REVERSED:
                    b, c = c, b
                triangles.append((start+a, start+b, start+c))
            faces.Next()
        name = f'solid_{i:03d}.obj'
        mesh_path = output/name
        with mesh_path.open('w') as f:
            f.write('# STEP solid; coordinates in metres; original CAD assembly frame\n')
            for v in vertices:
                f.write('v '+' '.join(f'{x:.10g}' for x in v)+'\n')
            for triangle in triangles:
                f.write('f '+' '.join(map(str, triangle))+'\n')
        bounds = Bnd_Box()
        BRepBndLib.Add_s(solid, bounds)
        properties = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid, properties)
        centre = properties.CentreOfMass()
        manifest['solids'].append(dict(id=i, mesh=name, mesh_sha256=sha256(mesh_path),
            bounds_m=[x/1000 for x in bounds.Get()], volume_m3=properties.Mass()/1e9,
            centre_m=[centre.Coord(k)/1000 for k in (1, 2, 3)],
            vertices=len(vertices), triangles=len(triangles)))
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    result = convert_step(a.source, a.output)
    print(json.dumps(result, indent=2))
