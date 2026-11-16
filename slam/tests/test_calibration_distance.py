"""Validate distance geometry independently of camera hardware."""

import unittest
from unittest.mock import patch

import cv2
import numpy as np

from check_calibration import estimate_board_distance


class DistanceTests(unittest.TestCase):
    def setUp(self):
        self.config = dict(
            fx=800.0,
            fy=810.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
            distortion=[0.05, -0.02, 0.001, -0.001, 0],
            inner_corners=[9, 6],
            square_size_m=0.0222,
        )
        self.image = np.zeros((480, 640, 3), np.uint8)

    def test_metric_center_depth_and_plane_distance(self):
        matrix = np.array([[800.0, 0, 320], [0, 810.0, 240], [0, 0, 1.0]])
        world = np.zeros((54, 3))
        world[:, :2] = (np.mgrid[:9, :6].T.reshape(-1, 2) - [4, 2.5]) * 0.0222
        for rvec, tvec in [([0.0, 0, 0], [0.0, 0, 0.6]), ([0.25, -0.35, 0.1], [0.12, -0.06, 0.85])]:
            rvec, tvec = np.array(rvec), np.array(tvec)
            corners, _ = cv2.projectPoints(
                world, rvec, tvec, matrix, np.array(self.config["distortion"])
            )
            corners = corners.astype(np.float32)
            with (
                patch.object(cv2, "findChessboardCorners", return_value=(True, corners)),
                patch.object(cv2, "cornerSubPix", return_value=corners),
            ):
                result = estimate_board_distance(self.image, self.config)
            self.assertAlmostEqual(result.center_range_m, np.linalg.norm(tvec), places=5)
            self.assertAlmostEqual(result.depth_m, tvec[2], places=5)
            rotation, _ = cv2.Rodrigues(rvec)
            self.assertAlmostEqual(result.perpendicular_m, abs(rotation[:, 2] @ tvec), places=5)
            self.assertLess(result.reprojection_rms_px, 0.001)

    def test_absent_board(self):
        self.assertIsNone(estimate_board_distance(self.image, self.config))

    def test_wrong_resolution_and_missing_square_size(self):
        with self.assertRaises(ValueError):
            estimate_board_distance(self.image[:100], self.config)
        config = dict(self.config)
        del config["square_size_m"]
        with self.assertRaises(ValueError):
            estimate_board_distance(self.image, config)

    def test_full_detection_on_rendered_chessboard(self):
        image = np.full((480, 640), 255, np.uint8)
        # 10x7 squares produce 9x6 inner corners, 30 pixels per square.
        for y in range(7):
            for x in range(10):
                if (x + y) % 2 == 0:
                    image[135 + y * 30 : 135 + (y + 1) * 30, 170 + x * 30 : 170 + (x + 1) * 30] = 0
        config = dict(self.config, fx=800.0, fy=800.0, distortion=[0.0] * 5)
        result = estimate_board_distance(image, config)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.depth_m, 800 * 0.0222 / 30, places=3)


if __name__ == "__main__":
    unittest.main()
