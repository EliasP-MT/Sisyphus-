import unittest

import numpy as np

from mailbox_slam.config import DEFAULTS
from mailbox_slam.geometry import rigid
from mailbox_slam.graph import PoseGraph


class VectorizedResidualTests(unittest.TestCase):
    def test_matches_the_loop_definition_and_converges(self):
        rng = np.random.default_rng(2)
        graph = PoseGraph(dict(DEFAULTS))
        yaw = 0.0
        for i in range(9):
            graph.add_frame(float(i), np.array([0.01 * i, 0.0, yaw]))
            yaw += 0.6
        truth = {}
        for k in range(4):
            truth[f"m{k}"] = rigid(np.array([0.4 + 0.05 * k, -0.3 + 0.1 * k, 0.3, 0.1 * k, 0.2, 3.0]))
            graph.landmarks[f"m{k}"] = dict(pose=truth[f"m{k}"].copy())
        for f in range(9):
            for k in range(4):
                if rng.random() < 0.6:
                    graph.observations.append((f, f"m{k}", truth[f"m{k}"]))
        graph.optimize()
        residual, reference, x0 = graph._residual_pair
        for _ in range(3):
            x = x0 + rng.normal(0, 0.05, x0.shape)
            np.testing.assert_allclose(residual(x), reference(x), atol=1e-9)
        self.assertTrue(graph.last_optimization["converged"])


if __name__ == "__main__":
    unittest.main()
