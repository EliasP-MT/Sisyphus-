from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplify_collision import surface_cells, merge_cells


class ProxyTests(unittest.TestCase):
    def test_box_contacts_are_refined_against_original_arm(self):
        from planner import Arm
        from test_arm import URDF,calibration,collision_scene
        arm=Arm(URDF,calibration())
        rng=np.random.default_rng(7)
        with collision_scene(arm) as detailed, collision_scene(arm,{'arm_collision_proxy':'link_boxes'}) as proxy:
            self.assertIsNotNone(proxy.refinement_robot)
            for q in [np.zeros(7),*rng.uniform(-.15,.15,(8,7))]:
                self.assertEqual(detailed.collision(q) is None,proxy.collision(q) is None)

    def test_merge_preserves_exact_cell_coverage(self):
        cells={(x,y,z) for x in range(5) for y in range(4) for z in range(2)}
        cells.add((8,8,8))
        restored=set()
        for lo,hi in merge_cells(cells):
            for x in range(lo[0],hi[0]):
                for y in range(lo[1],hi[1]):
                    for z in range(lo[2],hi[2]):
                        self.assertNotIn((x,y,z),restored)
                        restored.add((x,y,z))
        self.assertEqual(cells,restored)

    def test_large_triangle_interior_is_covered(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'wall.obj'
            path.write_text('v 0 0 0\nv 0.1 0 0\nv 0 0.1 0\nf 1 2 3\n')
            cells=surface_cells(path,.01)
            for x in np.linspace(0,.1,20):
                for y in np.linspace(0,.1-x,20):
                    key=tuple(np.floor(np.array([x,y,0])/.01).astype(int))
                    self.assertIn(key,cells)
            self.assertNotIn((5,5,5),cells)  # No filled volume above the wall.
            self.assertNotIn((9,9,0),cells)  # Empty triangle-AABB corner stays free.
