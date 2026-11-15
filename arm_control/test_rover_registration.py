import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from build_rover_scene import fit


class RegistrationTests(unittest.TestCase):
    def test_recovers_shared_named_part_transform(self):
        rng = np.random.default_rng(12)
        cad = [rng.uniform(-.03, .03, (100, 3)), rng.uniform(.04, .09, (100, 3))]
        expected = Rotation.from_euler('z', 7, degrees=True).as_matrix()
        translation = np.array([.006, -.018, .001])
        model = [points @ expected.T + translation for points in cad]
        rms, maximum, rotation, offset = fit(cad, model)
        self.assertLess(rms, 1e-8)
        self.assertLess(maximum, 1e-8)
        np.testing.assert_allclose(rotation, expected, atol=1e-8)
        np.testing.assert_allclose(offset, translation, atol=1e-8)
