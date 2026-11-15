from pathlib import Path
import tempfile
import unittest

from import_surface_cad import convert


class SurfaceImportTests(unittest.TestCase):
    def test_open_face_is_included_in_metre_mesh(self):
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
        from OCP.gp import gp_Pln, gp_Pnt, gp_Dir
        from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
        import json
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root/'surface.step'
            face = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)),
                                         0, 100, 0, 200).Face()
            writer = STEPControl_Writer()
            writer.Transfer(face, STEPControl_AsIs)
            writer.Write(str(source))
            convert(source, root/'out')
            report = json.loads((root/'out/assembly.json').read_text())
            self.assertEqual(report['faces'], 1)
            self.assertGreaterEqual(report['triangles'], 2)
            self.assertAlmostEqual(report['bounds_m'][1][0], .1)
            self.assertAlmostEqual(report['bounds_m'][1][1], .2)
            self.assertFalse(report['hardware_verified'])
            self.assertIn('f ', (root/'out/assembly.obj').read_text())
            # Surface-only import can be used by the collision engine, but
            # an unregistered scene must never authorize physical execution.
            from planner import Arm, load_config
            from collision import CollisionWorld
            project = Path(__file__).resolve().parent
            urdf = project/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'
            arm = Arm(urdf, load_config(project/'elrobot.example.json'))
            scene = json.loads((project/'robot_scene.example.json').read_text())
            scene.pop('cad')
            scene['surface_meshes'] = [dict(name='test surface', mesh=str(root/'out/assembly.obj'),
                                          xyz_m=[1,0,0], rpy_rad=[0,0,0])]
            path = root/'scene.json'
            path.write_text(json.dumps(scene))
            with CollisionWorld(urdf, arm, path) as world:
                self.assertEqual(len(world.cad_bodies), 1)
                self.assertIsNone(world.cad_bodies[0]['classifier'])
                world.collision([0]*7)
                with self.assertRaisesRegex(ValueError, 'registration'):
                    world.require_hardware_ready()
