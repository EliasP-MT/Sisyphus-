"""Download the exact recorded PaddleOCR weights, verifying every file."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--target', type=Path, default=HERE.parent / 'base_ocr/models.local')
args = parser.parse_args()
for model in json.loads((HERE / 'ocr-models.json').read_text()):
    for filename, expected in model['files'].items():
        path = args.target / model['name'] / filename
        if path.exists():
            if hashlib.sha256(path.read_bytes()).hexdigest() == expected:
                continue
            raise SystemExit(f'Refusing to replace changed model file: {path}')
        url = f"https://huggingface.co/{model['repository']}/resolve/{model['revision']}/{filename}"
        data = subprocess.check_output(['curl', '--fail', '--location', '--silent',
                                        '--show-error', '--retry', '3',
                                        '--max-time', '180', url])
        if hashlib.sha256(data).hexdigest() != expected:
            raise SystemExit(f'Unexpected checksum: {url}')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    print(f"Verified {model['name']} at {model['revision']}")
