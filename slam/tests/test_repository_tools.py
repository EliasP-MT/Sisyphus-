"""Deployment contents and durable map output regression checks."""

import json
import tarfile
import tempfile
import unittest
from pathlib import Path

import numpy as np

from mailbox_slam.mapping import write_json
from scripts.build_bundle import build_bundle


class RepositoryTests(unittest.TestCase):
    def test_bundle_excludes_local_state_models_and_calibration_by_default(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime.tar.gz"
            build_bundle(path)
            with tarfile.open(path) as archive:
                names = archive.getnames()
        self.assertIn("initialize_mailboxes.py", names)
        self.assertIn("mailbox_slam/discovery.py", names)
        self.assertIn("recipient_names.py", names)
        self.assertIn("config/recipients.json", names)
        self.assertNotIn("config/arm-camera-intrinsics.json", names)
        self.assertFalse(
            any("__pycache__" in n or "station_data" in n or "station/bin" in n for n in names)
        )
        self.assertNotIn("robot_constants.py", names)
        self.assertNotIn("teleop/arms.json", names)

    def test_bundle_can_explicitly_include_measured_intrinsics(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime.tar.gz"
            build_bundle(path, include_calibration=True)
            with tarfile.open(path) as archive:
                self.assertIn("config/arm-camera-intrinsics.json", archive.getnames())

    def test_bad_map_does_not_overwrite_previous_valid_export(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.json"
            write_json(path, {"pose": np.array([1.0, 2.0, 3.0])})
            before = path.read_bytes()
            with self.assertRaises(ValueError):
                write_json(path, {"pose": float("nan")})
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(json.loads(before)["pose"], [1.0, 2.0, 3.0])
            self.assertEqual(list(Path(directory).iterdir()), [path])
