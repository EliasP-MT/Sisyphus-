# Standalone base-camera OCR

This branch contains the letter recipient reader, independent of SLAM and arm OCR.
The files under `base_ocr/` are self-contained; main-owned files are unchanged.

```bash
cd base_ocr
python3 -m pip install -r requirements.txt
python3 ocr_base.py --target mac --device 0 --test
```

Open http://127.0.0.1:8768. Device 0 must be the attached USB camera; close
other camera owners first. On the RDK use `python3 ocr_base.py --test` (default
robot backend and `/dev/video0`). Without `--test`, read once and print a name.
`--image letter.jpg` accepts a saved image; `--benchmark 5` reports warm stage timings.

```python
from ocr_base import BaseOCR

with BaseOCR(target="mac", device=0) as reader:
    reader.prepare()
    name = reader.read_name()
```

PP-OCRv5 Mobile detection/recognition run on the entire unchanged BGR frame.
No extra model, crop or preprocessing is applied outside PaddleOCR. Address
selection uses text positions and street/postcode patterns; unsupported or
ambiguous layouts return an empty string. The returned first line can include
a title. This heuristic needs validation on real letters.

Models load once. The camera stays open and continuously retains its latest
frame. OCR results/boxes/timings are shown only at actual inference speed.
The one-second budget raises TimeoutError after processing when exceeded;
this is not a hard execution timeout or a proven RDK performance guarantee.
PaddlePaddle installation requires a wheel matching your OS, CPU and Python.

Model-free checks:

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m unittest discover -s tests -v
ruff check .
ruff format --check .
```
