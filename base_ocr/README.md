# Base-camera OCR

Call `read_recipient_name()` from `ocr_base.py`. It opens the robot base camera,
discards five frames, reads the sixth, releases the camera and runs PP-OCRv5 Mobile
on that full image. Nearby words on the topmost detected line are joined in
left-to-right order, then matched to `recipients.json`. The returned string uses
the spelling in that list. Empty, low-confidence or ambiguous results return
`""`; camera, configuration and model errors raise exceptions. Later lines are
ignored, even if they contain a known name.

```python
from ocr_base import read_recipient_name

name = read_recipient_name()
```

The robot defaults to `/dev/video0`; pass `device="/dev/videoN"` if its base
camera uses another device. The caller must select the physical base camera.
Use `names_path="path/to/names.json"` to select another JSON list of strings.

Install and test with a Mac USB camera:

```bash
cd base_ocr
python3 -m pip install -r requirements.txt
python3 ocr_base.py --target mac --device 0
```

Or call `read_recipient_name(target="mac", device=0)`. The script prints one
result and exits. There is no preview or background capture. Models are cached
between function calls; the first call may load/download model weights. Capture,
model initialization and CPU inference have no guaranteed subsecond latency.
There is no additional crop, image preprocessing or separate detection model.

Model-free checks:

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m unittest discover -s tests -v
ruff check .
ruff format --check .
```
