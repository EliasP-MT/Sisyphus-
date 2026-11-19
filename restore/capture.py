"""Capture the explicit, non-secret robot data allowlist (no venv or station state)."""
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'restore'
paths = set()
for directory in ('arm_control/assets/rover-fixed.local',
                  'arm_control/assets/full_robot_box_complete_collision', 'config', 'models'):
    paths.update(p for p in (ROOT / directory).rglob('*') if p.is_file())
paths.update((ROOT / 'arm_control').glob('*.local.json'))
paths.update((ROOT / 'arm_control').glob('*.local.json.bak-*'))
paths.update(ROOT / 'arm_control/curobo' / name for name in
             ('elrobot.yml', 'elrobot_arm.generated.urdf', 'self_exempt.json'))
entries = []
with tarfile.open(OUT / 'robot-data.tar.gz', 'w:gz') as archive:
    for path in sorted(paths):
        if path.is_symlink():
            raise ValueError(f'Symlink not allowed: {path}')
        relative = path.relative_to(ROOT).as_posix()
        data = path.read_bytes()
        entries.append({'path': relative, 'size': len(data),
                        'sha256': hashlib.sha256(data).hexdigest()})
        archive.add(path, arcname=relative, recursive=False)
manifest = {'version': 1, 'source_root': str(ROOT), 'files': entries,
            'archive_sha256': hashlib.sha256((OUT / 'robot-data.tar.gz').read_bytes()).hexdigest()}
(OUT / 'robot-data.json').write_text(json.dumps(manifest, indent=2) + '\n')
print(f'Captured {len(entries)} robot data files')
