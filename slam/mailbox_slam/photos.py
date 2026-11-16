"""Per-box evidence photos: the detected label/slot contours drawn on the frame, cropped."""

import base64

import cv2
import numpy as np


def box_photo(image, result, calibration, geometry, height_px=160):
    """Base64 JPEG of the box front face with its label (green) and slot (red) contours."""
    from letterbox_locator import slot_outline_px

    label = np.float32(result["label_corners_px"]).reshape(-1, 2)
    outline = np.float32(slot_outline_px(result, calibration, geometry, image))[:4]
    drawn = image.copy()
    cv2.polylines(drawn, [np.int32(label)], True, (0, 200, 0), 2)
    cv2.polylines(drawn, [np.int32(outline)], True, (0, 0, 255), 2)
    pts = np.vstack([label, outline])
    x0, y0 = pts.min(axis=0)
    x1, y1 = pts.max(axis=0)
    margin = 0.35 * max(x1 - x0, y1 - y0)
    h, w = image.shape[:2]
    left, top = int(max(0, x0 - margin)), int(max(0, y0 - margin))
    right, bottom = int(min(w, x1 + margin)), int(min(h, y1 + margin))
    crop = drawn[top:bottom, left:right]
    if crop.size == 0:
        return None
    scale = height_px / crop.shape[0]
    crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    ok, jpeg = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return base64.b64encode(jpeg).decode() if ok else None
