"""Restore verified robot data and relocate paths, refusing unknown local changes."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile

HERE = Path(__file__).resolve().parent


def digest(data):
    return hashlib.sha256(data).hexdigest()


def restore(target):
    target = target.resolve()
    manifest = json.loads((HERE / 'robot-data.json').read_text())
    archive_path = HERE / 'robot-data.tar.gz'
    if digest(archive_path.read_bytes()) != manifest['archive_sha256']:
        raise ValueError('Archive checksum mismatch')
    entries = {item['path']: item for item in manifest['files']}
    original, rendered = {}, {}
    with tarfile.open(archive_path, 'r:gz') as archive:
        members = archive.getmembers()
        if len(members) != len(entries) or {m.name for m in members} != set(entries):
            raise ValueError('Archive entries do not match manifest')
        for member in members:
            name = PurePosixPath(member.name)
            if not member.isfile() or name.is_absolute() or '..' in name.parts:
                raise ValueError(f'Unsafe archive member: {name}')
            entry = entries[member.name]
            if member.size != entry['size']:
                raise ValueError(f'Size mismatch: {name}')
            data = archive.extractfile(member).read()
            if digest(data) != entry['sha256']:
                raise ValueError(f'Checksum mismatch: {name}')
            original[member.name] = data
            # JSON, YAML and URDF paths may refer to the previous checkout.
            if name.suffix in ('.json', '.yml', '.urdf') or '.json.bak-' in name.name:
                data = data.decode().replace(manifest['source_root'], str(target)).encode()
            rendered[member.name] = data
    model = 'arm_control/curobo/elrobot.yml'
    exemptions = 'arm_control/curobo/self_exempt.json'
    saved = json.loads(original[exemptions])
    if saved['robot_yml_sha256'] != digest(original[model]):
        raise ValueError('Original collision exemptions do not match the saved model')
    if rendered[model] != original[model]:
        # Only paths changed; preserve the exact sphere geometry and exemption pairs.
        saved['robot_yml_sha256'] = digest(rendered[model])
        rendered[exemptions] = (json.dumps(saved, indent=1) + '\n').encode()
    pending = []
    for name, data in rendered.items():
        destination = target / name
        if not destination.resolve().is_relative_to(target) or destination.is_symlink():
            raise ValueError(f'Unsafe destination: {destination}')
        if destination.exists():
            current = destination.read_bytes()
            if current == data:
                continue
            if current != original[name]:
                raise ValueError(f'Refusing to overwrite local changes: {destination}')
        pending.append((destination, data))
    # All integrity and conflict checks finish before any write.
    for destination, data in pending:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
    print(f'Verified {len(entries)} files; restored/relocated {len(pending)} files in {target}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', type=Path, default=HERE.parent)
    restore(parser.parse_args().target)
