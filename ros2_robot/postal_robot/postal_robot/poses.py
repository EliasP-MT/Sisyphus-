"""Pose library in the existing encoder-poses-v1 format, with atomic saves.

The file stays compatible with arm_control/calibration_ui.py: each pose has
"recorded_at" and raw "ticks" for motors 1..8 (8 = gripper). Extra fields
("calibration_sha256", "source") are additive. Replacing or deleting a pose
first copies the whole file to poses-backup-<timestamp>.local.json.
"""
import datetime
import json
import os
import re
import shutil
import tempfile
import threading
from pathlib import Path

from .state import Sample, StableCapture

SCHEMA = 'encoder-poses-v1'
ARM_IDS = tuple(range(1, 9))
NAME = re.compile(r'^[A-Za-z0-9_\-][A-Za-z0-9_\- .]{0,62}$')


class PoseError(ValueError):
    pass


def _stamp():
    return datetime.datetime.now(datetime.timezone.utc)


class PoseStore:
    def __init__(self, path, bus_serial):
        self.path, self.bus_serial = Path(path), bus_serial
        self.lock = threading.Lock()
        if self.path.exists():
            self._read()  # validate early

    def _read(self):
        if not self.path.exists():
            return {'schema': SCHEMA, 'bus': self.bus_serial, 'model_calibrated': False, 'poses': {}}
        data = json.loads(self.path.read_text())
        if data.get('schema') != SCHEMA:
            raise PoseError(f'{self.path}: unsupported pose schema {data.get("schema")}')
        if data.get('bus') != self.bus_serial:
            raise PoseError(f'{self.path}: poses recorded on bus {data.get("bus")}, not {self.bus_serial}')
        for name, pose in data.get('poses', {}).items():
            ticks = pose.get('ticks', {})
            if sorted(ticks, key=int) != [str(i) for i in ARM_IDS] or any(
                    type(v) is not int or not 0 <= v <= 4095 for v in ticks.values()):
                raise PoseError(f'Pose {name}: eight raw encoder values 0..4095 required')
        return data

    def poses(self):
        with self.lock:
            return self._read()['poses']

    def _write(self, data, backup):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if backup and self.path.exists():
            name = f"poses-backup-{_stamp().strftime('%Y%m%d-%H%M%S-%f')}.local.json"
            shutil.copy2(self.path, self.path.with_name(name))
        fd, tmp = tempfile.mkstemp(prefix='.poses-', dir=self.path.parent)
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(data, f, indent=2)
                f.write('\n')
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
            dirfd = os.open(self.path.parent, os.O_RDONLY)
            try:
                os.fsync(dirfd)
            finally:
                os.close(dirfd)
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

    def save(self, name, ticks, replace, calibration_sha256, source):
        if not NAME.match(name or ''):
            raise PoseError('Pose name: 1-63 letters, digits, space, dot, dash or underscore')
        if sorted(ticks) != list(ARM_IDS):
            raise PoseError('Ticks for motors 1..8 required')
        with self.lock:
            data = self._read()
            exists = name in data['poses']
            if exists and not replace:
                raise PoseError(f'Pose "{name}" exists; use replace to overwrite (a backup is kept)')
            if replace and not exists:
                raise PoseError(f'Pose "{name}" does not exist')
            pose = {'recorded_at': _stamp().isoformat(),
                    'ticks': {str(i): int(ticks[i]) for i in ARM_IDS},
                    'calibration_sha256': calibration_sha256, 'source': source}
            data['poses'][name] = pose
            self._write(data, backup=exists)
            return pose

    def delete(self, name):
        with self.lock:
            data = self._read()
            if name not in data['poses']:
                raise PoseError(f'Pose "{name}" does not exist')
            del data['poses'][name]
            self._write(data, backup=True)


def capture_from_snapshots(snapshots, tolerance_ticks=3, required=3, max_skew=.1):
    """Find a stable capture in a sequence of fresh per-cycle snapshots.

    Each snapshot maps motor ID -> (ticks, received_monotonic, sequence) and must
    already contain only fresh, error-free replies. Returns ticks or None.
    """
    capture = StableCapture(tolerance=tolerance_ticks, required=required)
    for snapshot in snapshots:
        if sorted(snapshot) != list(ARM_IDS):
            capture.history.clear()
            continue
        stamps = [s[1] for s in snapshot.values()]
        if max(stamps) - min(stamps) > max_skew:
            capture.history.clear()
            continue
        result = capture.feed({i: Sample(float(t), r, q) for i, (t, r, q) in snapshot.items()})
        if result is not None:
            return {i: int(round(v)) for i, v in result.items()}
    return None
