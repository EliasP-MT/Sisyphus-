"""OCR delegated to the PC station (the RDK has no PaddleOCR).

Same interface as ocr_arm.ArmOCR as used by LetterboxLocator: read_text(crop) returns merged
text groups, last_detections keeps the latest ones, TimeoutError when the station is silent.
Transport-agnostic: `send(payload)` publishes a request, `deliver(reply)` receives the answer.
"""

import base64
import threading
import uuid

import cv2
import numpy as np


class RemoteOCR:
    def __init__(self, send, *, timeout_s=10.0, merge_gap=0.9, min_confidence=0.3):
        self.send, self.timeout_s = send, float(timeout_s)
        self.merge_gap, self.min_confidence = merge_gap, min_confidence
        self.last_detections = []
        self._pending = {}
        self._lock = threading.Lock()

    def read_text(self, crop):
        from ocr_arm import merge_text_regions

        ok, png = cv2.imencode(".png", np.asarray(crop, np.uint8))  # lossless: small text
        if not ok:
            raise ValueError("Cannot encode the label crop")
        request_id = uuid.uuid4().hex
        slot = dict(event=threading.Event(), reply=None)
        with self._lock:
            self._pending[request_id] = slot
        try:
            self.send(dict(id=request_id, image_b64=base64.b64encode(png.tobytes()).decode()))
            if not slot["event"].wait(self.timeout_s):
                raise TimeoutError("Station OCR did not answer (is the PC station running?)")
        finally:
            with self._lock:
                self._pending.pop(request_id, None)
        reply = slot["reply"]
        if not reply.get("success"):
            raise RuntimeError(reply.get("message", "Station OCR failed"))
        self.last_detections = merge_text_regions(
            dict(
                rec_texts=reply["rec_texts"],
                rec_scores=reply["rec_scores"],
                rec_polys=reply["rec_polys"],
            ),
            merge_gap=self.merge_gap,
            min_confidence=self.min_confidence,
        )
        return self.last_detections

    def deliver(self, reply):
        with self._lock:
            slot = self._pending.get(reply.get("id"))
        if slot is not None:
            slot["reply"] = reply
            slot["event"].set()
