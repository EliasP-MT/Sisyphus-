"""Locate a letterbox slot from its name label and read the recipient name.

Two separate APIs: locate_all(image) is geometry-only and finds every label;
read_label_name(image, corners) performs lazy OCR and recipient matching.
locate(image) retains the legacy largest-label combined behavior.

Pipeline (one image):
  1. find the label: a quadrilateral with a single thick black border and a
     light interior (the slot is dark inside, so it is rejected);
  2. solvePnP with the label's real size -> label pose in the camera frame;
  3. add the fixed label->slot offset (config/letterbox.json) -> slot centre;
  4. OCR the rectified label interior and match it against config/recipients.json.

Camera frame = OpenCV optical frame: x right, y down, z forward, metres.
Label frame: origin at the label centre, x right, y down, z into the box.
The slot normal returned is the direction to push the letter (into the box).
Converting to the arm base frame needs the camera mounting + arm FK (not here).
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from recipient_names import match_recipient

HERE = Path(__file__).resolve().parent


def load_json(path):
    return json.loads(Path(path).read_text())


def camera_matrix(calibration):
    k = np.array(
        [
            [calibration["fx"], 0, calibration["cx"]],
            [0, calibration["fy"], calibration["cy"]],
            [0, 0, 1.0],
        ]
    )
    return k, np.asarray(calibration["distortion"], dtype=float)


# --------------------------------------------------------------------------
# 1. Label detection
# --------------------------------------------------------------------------
def _order(points):
    """TL, TR, BR, BL in image coordinates."""
    s, d = points.sum(axis=1), points[:, 1] - points[:, 0]
    return np.array(
        [points[np.argmin(s)], points[np.argmin(d)], points[np.argmax(s)], points[np.argmax(d)]],
        dtype=np.float32,
    )


def _approx_quad(contour):
    hull = cv2.convexHull(contour)
    poly = cv2.approxPolyDP(hull, 0.03 * cv2.arcLength(hull, True), True)
    return poly.reshape(-1, 2).astype(np.float32) if len(poly) == 4 else None


def detect_labels(image, *, min_area=300, aspect_range=(0.6, 2.2)):
    """Return outer corner quads (TL, TR, BR, BL float32 px) of single-border labels, largest first."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    block = max(31, (min(gray.shape) // 8) | 1)
    dark = cv2.adaptiveThreshold(
        cv2.GaussianBlur(gray, (1, 1), 0),
        255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY_INV,
        block,
        12,
    )
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, hierarchy = cv2.findContours(dark, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    found = []
    for i, (_, _, child, parent) in enumerate(hierarchy[0]):
        if parent != -1 or child == -1:
            continue  # need an outer contour with a hole
        outer = _approx_quad(contours[i])
        if outer is None or not cv2.isContourConvex(outer.reshape(-1, 1, 2)):
            continue
        area = cv2.contourArea(outer)
        if area < min_area:
            continue
        # The inner edge of the border: large lettering touching the border splits the white
        # interior into several holes, so take the hull of all of them rather than the largest.
        holes, j = [], child
        while j != -1:
            holes.append(contours[j])
            j = hierarchy[0][j][0]
        inner = _approx_quad(np.vstack(holes))
        if inner is None or not 0.2 < cv2.contourArea(inner) / area < 0.92:
            continue
        outer = _order(outer)
        sides = np.linalg.norm(outer - np.roll(outer, -1, axis=0), axis=1)
        aspect = (sides[0] + sides[2]) / max(1e-6, sides[1] + sides[3])
        if not aspect_range[0] < aspect < aspect_range[1]:
            continue
        # Interior must be clearly lighter than the border.
        mask_in = np.zeros_like(gray)
        cv2.fillPoly(mask_in, [inner.astype(np.int32)], 255)
        mask_border = np.zeros_like(gray)
        cv2.fillPoly(mask_border, [outer.astype(np.int32)], 255)
        mask_border[mask_in > 0] = 0
        if cv2.mean(gray, mask_in)[0] < cv2.mean(gray, mask_border)[0] + 40:
            continue
        refined = cv2.cornerSubPix(
            gray,
            outer.reshape(-1, 1, 2).copy(),
            (4, 4),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01),
        )
        found.append((area, refined.reshape(4, 2)))
    return [quad for _, quad in sorted(found, key=lambda f: -f[0])]


# --------------------------------------------------------------------------
# 2-3. Pose and slot centre
# --------------------------------------------------------------------------
def label_pose_candidates(corners, calibration, geometry):
    """All positive-depth IPPE candidates, sorted by corner RMS reprojection error."""
    w, h = geometry["label_width_m"], geometry["label_height_m"]
    obj = np.array(
        [[-w / 2, -h / 2, 0], [w / 2, -h / 2, 0], [w / 2, h / 2, 0], [-w / 2, h / 2, 0]],
        dtype=np.float64,
    )
    points = np.asarray(corners, dtype=np.float64)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        raise ValueError("Expected four finite ordered label corners")
    k, dist = camera_matrix(calibration)
    count, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, points, k, dist, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if not count:
        return candidates
    for rvec, tvec in zip(rvecs, tvecs):
        r, _ = cv2.Rodrigues(rvec)
        t = tvec.ravel()
        if not np.isfinite(t).all() or np.any((obj @ r.T + t)[:, 2] <= 0) or r[:, 2] @ t <= 0:
            continue
        projected, _ = cv2.projectPoints(obj, rvec, tvec, k, dist)
        err = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - points) ** 2, axis=1))))
        slot = r @ np.asarray(geometry["slot_from_label_center_m"]) + t
        candidates.append(
            dict(
                slot_center_m=slot.tolist(),
                slot_normal=r[:, 2].tolist(),
                slot_x_axis=r[:, 0].tolist(),
                label_center_m=t.tolist(),
                label_rotation=r.tolist(),
                distance_m=float(np.linalg.norm(slot)),
                reprojection_px=err,
            )
        )
    return sorted(candidates, key=lambda candidate: candidate["reprojection_px"])


def slot_pose(corners, calibration, geometry):
    """Best single candidate; mapping callers should retain label_pose_candidates."""
    candidates = label_pose_candidates(corners, calibration, geometry)
    return candidates[0] if candidates else None


def _similarity(source, target):
    """Least-squares rotation + uniform scale + translation mapping source points to target points
    (Umeyama). 4 DOF: with a 6 cm label and a 23 cm slot a full perspective fit amplifies one pixel
    of corner noise 4x and the outline runs away; a similarity cannot."""
    mean_s, mean_t = source.mean(axis=0), target.mean(axis=0)
    a, b = source - mean_s, target - mean_t
    u, sigma, vt = np.linalg.svd(b.T @ a)
    d = np.sign(np.linalg.det(u @ vt))
    flip = np.diag([1.0, d])
    rotation = u @ flip @ vt
    scale = float((sigma * np.array([1.0, d])).sum() / (a ** 2).sum())
    return scale, rotation, mean_t - scale * rotation @ mean_s


def refine_outline_angle(image, outline, search_deg=10.0, band_px=10, min_pixels=60):
    """Re-orient a rectangle (4 corners TL,TR,BR,BL) with the edges seen in the image.

    A label is glued by hand and is a few degrees off its box, so the outline derived from its corners
    inherits that angle. The long edges of the slot are strong straight lines: the orientation of the
    gradient pixels along the outline's top and bottom edges (within search_deg of the outline's own
    angle) gives the box's real direction. The rectangle keeps its centre and size. Returns the
    outline unchanged when too few edge pixels support a better angle."""
    quad = np.asarray(outline, np.float64)[:4]
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    centre = quad.mean(axis=0)
    along = (quad[1] - quad[0] + quad[2] - quad[3]) / 2
    across = (quad[3] - quad[0] + quad[2] - quad[1]) / 2
    theta0 = np.arctan2(along[1], along[0])
    x0, y0 = np.maximum(np.floor(quad.min(axis=0)) - band_px - 2, 0).astype(int)
    x1, y1 = np.minimum(np.ceil(quad.max(axis=0)) + band_px + 2, [gray.shape[1], gray.shape[0]]).astype(int)
    if x1 - x0 < 20 or y1 - y0 < 10:
        return quad
    roi = cv2.GaussianBlur(gray[y0:y1, x0:x1], (3, 3), 0).astype(np.float32)
    gx, gy = cv2.Sobel(roi, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(roi, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = np.hypot(gx, gy)
    edge_angle = np.arctan2(gy, gx) + np.pi / 2  # direction of the edge, not of its normal
    ys, xs = np.mgrid[y0:y1, x0:x1]
    points = np.column_stack([xs.ravel(), ys.ravel()]).astype(np.float64)
    unit = along / np.linalg.norm(along)
    normal = np.array([-unit[1], unit[0]])
    offset = (points - centre) @ normal                      # signed distance to the centre line
    half = np.linalg.norm(across) / 2
    on_long_edges = np.abs(np.abs(offset) - half) <= band_px   # the top and the bottom edge
    lateral = np.abs((points - centre) @ unit) <= np.linalg.norm(along) / 2 * 0.9
    delta = np.angle(np.exp(2j * (edge_angle.ravel() - theta0))) / 2   # modulo 180 deg, around theta0
    keep = on_long_edges & lateral & (np.abs(delta) <= np.radians(search_deg)) & (magnitude.ravel() > 0.35 * magnitude.max())
    if keep.sum() < min_pixels:
        return quad
    weights = magnitude.ravel()[keep]
    theta = theta0 + 0.5 * np.arctan2((weights * np.sin(2 * delta[keep])).sum(), (weights * np.cos(2 * delta[keep])).sum())
    length, width = np.linalg.norm(along), np.linalg.norm(across)
    direction = np.array([np.cos(theta), np.sin(theta)])
    centre, length = _snap_side_edges(gx, gy, points, magnitude.ravel(), centre, direction, length, width)
    u = length * direction
    v = width * np.array([-direction[1], direction[0]])
    return np.array([centre - u / 2 - v / 2, centre + u / 2 - v / 2, centre + u / 2 + v / 2, centre - u / 2 + v / 2])


def _snap_side_edges(gx, gy, points, magnitude, centre, direction, length, width, reach=0.26):
    """Move the left and right sides of the rectangle onto the strongest nearby vertical edges.

    The width comes from the size of a 6 cm label, so it is only as good as that estimate; the sides of
    the slot opening are straight dark lines in the image. Each side is searched within +-reach of the
    length and only moved when its edge clearly stands out from the rest of the profile."""
    normal = np.array([-direction[1], direction[0]])
    s = (points - centre) @ direction
    t = (points - centre) @ normal
    band = np.abs(t) <= 0.35 * width
    along_gradient = np.abs(gx.ravel() * direction[0] + gy.ravel() * direction[1])
    bins = np.arange(-0.75 * length, 0.75 * length + 1, 2.0)
    index = np.digitize(s[band], bins)
    profile = np.bincount(index, weights=along_gradient[band], minlength=len(bins) + 1)[1:len(bins)].astype(float)
    centres = (bins[:-1] + bins[1:]) / 2
    profile = np.convolve(profile, np.ones(5) / 5, mode='same')
    baseline = np.median(profile[profile > 0]) if (profile > 0).any() else 0.0

    def peak(expected, outermost):
        window = np.abs(centres - expected) <= reach * length
        if not window.any() or baseline <= 0:
            return expected
        best = np.argmax(np.where(window, profile, -1))
        if profile[best] < 2.0 * baseline:
            return expected
        if outermost:
            # the opening has inner lines (shadow, a flap) that can be stronger than its outer edge:
            # on the free side take the outermost local maximum that is still a clear edge
            local_max = (profile >= np.roll(profile, 1)) & (profile >= np.roll(profile, -1))
            strong = window & local_max & (profile >= 0.4 * profile[best])
            if strong.any():
                return centres[np.flatnonzero(strong)[-1 if outermost > 0 else 0]]
        return centres[best]

    left, right = peak(-length / 2, 0), peak(length / 2, 1)
    if right - left < 0.6 * length or right - left > 1.4 * length:
        return centre, length
    return centre + direction * (left + right) / 2, right - left


def slot_outline_px(pose, calibration, geometry, image=None):
    """The slot rectangle in the image (4 corners + centre), for drawing.

    The slot lies in the label's plane at a measured offset, so its image follows from the 4 detected
    label corners alone, without the estimated 3D tilt of the label (poorly constrained for a small
    label seen head-on, it used to turn the rectangle into a trapezoid). The mapping is a similarity,
    not a perspective fit: it is stable and always gives a rectangle. It assumes the wall roughly
    faces the camera, which is how the robot is placed.
    """
    sw, sh = geometry["slot_width_m"], geometry["slot_height_m"]
    cx, cy = geometry["slot_from_label_center_m"][:2]
    w, h = geometry["label_width_m"], geometry["label_height_m"]
    corners = pose.get("label_corners_px")
    k, dist = camera_matrix(calibration)
    if corners is None:  # no detected corners (injected poses): fall back to the pose projection
        pts = np.array([[cx - sw / 2, cy - sh / 2, 0], [cx + sw / 2, cy - sh / 2, 0],
                        [cx + sw / 2, cy + sh / 2, 0], [cx - sw / 2, cy + sh / 2, 0], [cx, cy, 0]])
        px, _ = cv2.projectPoints(pts, cv2.Rodrigues(np.asarray(pose["label_rotation"]))[0],
                                  np.asarray(pose["label_center_m"]), k, dist)
        return px.reshape(-1, 2)
    plane = np.float64([[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]])
    ideal = cv2.undistortPoints(np.float32(corners).reshape(-1, 1, 2), k, dist, P=k).reshape(-1, 2)
    scale, rotation, shift = _similarity(plane, ideal.astype(np.float64))
    slot = np.float64([[cx - sw / 2, cy - sh / 2], [cx + sw / 2, cy - sh / 2],
                       [cx + sw / 2, cy + sh / 2], [cx - sw / 2, cy + sh / 2], [cx, cy]])
    pixels = scale * slot @ rotation.T + shift
    # back to the (distorted) image: pinhole rays through the ideal pixels, then the lens model
    rays = np.column_stack([(pixels[:, 0] - k[0, 2]) / k[0, 0], (pixels[:, 1] - k[1, 2]) / k[1, 1],
                            np.ones(len(pixels))])
    distorted, _ = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), k, dist)
    outline = distorted.reshape(-1, 2)
    if image is not None:  # follow the real edges of the slot, not the angle of the glued label
        refined = refine_outline_angle(image, outline)
        outline = np.vstack([refined, refined.mean(axis=0)])
    return outline


# --------------------------------------------------------------------------
# 4. Name reading and matching
# --------------------------------------------------------------------------
def rectify_label(image, corners, geometry, scale_px_per_m=6000, inset=0.12):
    """Fronto-parallel crop of the label interior (border trimmed) for OCR."""
    w = int(geometry["label_width_m"] * scale_px_per_m)
    h = int(geometry["label_height_m"] * scale_px_per_m)
    dst = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    warped = cv2.warpPerspective(
        image,
        cv2.getPerspectiveTransform(corners.astype(np.float32), dst),
        (w, h),
        flags=cv2.INTER_CUBIC,  # the label is only a few dozen pixels wide in the frame
    )
    mx, my = int(w * inset), int(h * inset)
    crop = warped[my : h - my, mx : w - mx]
    pad = 20  # white margin helps the text detector
    return cv2.copyMakeBorder(crop, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=(255, 255, 255))


class LetterboxLocator:
    def __init__(self, calibration, geometry=None, recipients=None, ocr=True):
        self.calibration = (
            load_json(calibration) if isinstance(calibration, (str, Path)) else calibration
        )
        self.geometry = load_json(HERE / "config/letterbox.json") if geometry is None else geometry
        self.recipients = (
            load_json(HERE / "config/recipients.json") if recipients is None else recipients
        )
        self.ocr = None
        self.ocr_enabled = bool(ocr)

    def _check_image(self, image):
        h, w = image.shape[:2]
        if (w, h) != (self.calibration["width"], self.calibration["height"]):
            raise ValueError(
                f"Image {w}x{h} does not match calibration "
                f"{self.calibration['width']}x{self.calibration['height']}"
            )

    def read_text(self, crop):
        if not self.ocr_enabled:
            raise ValueError("Name reading disabled; construct with ocr=True")
        if self.ocr is None:
            from ocr_arm import ArmOCR

            self.ocr = ArmOCR(device=None, max_seconds=30.0, min_confidence=0.3)
        try:
            groups = self.ocr.read_text(crop)
        except TimeoutError:
            groups = self.ocr.last_detections
        return " ".join(g["text"] for g in groups)

    def read_label_name(self, image, corners):
        """Read ONLY this geometrically selected label, then match the database.

        OCR initialization is lazy: geometry discovery never loads its model.
        Unknown/near-tied database matches return recipient=None.
        """
        self._check_image(image)
        start = time.perf_counter()
        text = self.read_text(rectify_label(image, np.asarray(corners, np.float32), self.geometry))
        name, score = match_recipient(text, self.recipients)
        return dict(
            ocr_text=text, recipient=name, match_score=score, ocr_s=time.perf_counter() - start
        )

    def locate_all(self, image, *, read_names=False):
        """Geometry for ALL labels; optionally read names as a separate stage.

        Slot positions are predictions from the measured fixed label/slot offset,
        not independently detected holes. pose_candidates retains planar ambiguity.
        """
        self._check_image(image)
        start = time.perf_counter()
        results = []
        for corners in detect_labels(image):
            candidates = label_pose_candidates(corners, self.calibration, self.geometry)
            if not candidates:
                continue
            result = dict(
                label_corners_px=corners.tolist(),
                label_area_px2=float(cv2.contourArea(corners)),
                **candidates[0],
                pose_candidates=candidates,
            )
            result["geometry_s"] = time.perf_counter() - start
            if read_names:
                result.update(self.read_label_name(image, corners))
            results.append(result)
        return results

    def locate(self, image):
        """Legacy API: largest label, with names if OCR was enabled."""
        results = self.locate_all(image, read_names=self.ocr_enabled)
        return results[0] if results else None

    def annotate(self, image, result):
        out = image.copy()
        if result is None:
            cv2.putText(out, "no label", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return out
        cv2.polylines(out, [np.int32(result["label_corners_px"])], True, (0, 200, 0), 2)
        outline = slot_outline_px(result, self.calibration, self.geometry, image)
        cv2.polylines(out, [np.int32(outline[:4])], True, (0, 0, 255), 2)
        cv2.circle(out, tuple(np.int32(outline[4])), 5, (0, 0, 255), -1)
        x, y, z = result["slot_center_m"]
        lines = [
            f"slot x={x * 100:+.1f} y={y * 100:+.1f} z={z * 100:.1f} cm  ({result['distance_m'] * 100:.1f} cm)",
            f"reproj {result['reprojection_px']:.2f}px  box area {result['label_area_px2']:.0f} px^2",
        ]
        if "ocr_text" in result:
            lines.append(
                f'ocr "{result["ocr_text"]}" -> {result["recipient"]} ({result["match_score"]:.2f})'
            )
        for i, text in enumerate(lines):
            cv2.putText(out, text, (10, 25 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
            cv2.putText(
                out, text, (10, 25 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1
            )
        return out


# --------------------------------------------------------------------------
# CLI: single image, or live webcam with a browser preview
# --------------------------------------------------------------------------
def _serve_mjpeg(locator, device, port, width, height):
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    cap = cv2.VideoCapture(int(device) if str(device).isdecimal() else device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {device}")
    latest = dict(jpeg=None, result=None)
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/result.json":
                with lock:
                    body = json.dumps(latest["result"]).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != "/stream":
                body = b'<html><body style="margin:0;background:#111"><img src="/stream" style="width:100%"></body></html>'
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with lock:
                        jpeg = latest["jpeg"]
                    if jpeg:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                        )
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Preview: http://127.0.0.1:{port}  (Ctrl+C to stop)", file=sys.stderr)
    try:
        while True:
            for _ in range(2):
                cap.grab()  # drop stale buffered frames
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Camera read failed")
            result = locator.locate(frame)
            ok, jpeg = cv2.imencode(".jpg", locator.annotate(frame, result))
            with lock:
                latest.update(jpeg=jpeg.tobytes(), result=result)
            if result:
                print(
                    json.dumps(
                        {
                            k: result[k]
                            for k in ("slot_center_m", "distance_m", "reprojection_px")
                            if k in result
                        }
                        | {k: result.get(k) for k in ("ocr_text", "recipient")}
                    )
                )
    finally:
        cap.release()
        server.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--calibration",
        required=True,
        help="Camera intrinsics JSON (e.g. config/webcam-intrinsics.json)",
    )
    parser.add_argument("--image", help="Process one saved image")
    parser.add_argument("--device", help="Live camera: /dev/videoN or index")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument(
        "--no-ocr", action="store_true", help="Geometry only (fast, no PaddleOCR needed)"
    )
    parser.add_argument("--save", help="With --image: write the annotated image here")
    args = parser.parse_args(argv)
    if bool(args.image) == bool(args.device):
        parser.error("give exactly one of --image or --device")
    locator = LetterboxLocator(args.calibration, ocr=not args.no_ocr)
    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            parser.error(f"cannot read {args.image}")
        result = locator.locate(image)
        print(json.dumps(result, indent=2))
        if args.save:
            cv2.imwrite(args.save, locator.annotate(image, result))
    else:
        cal = locator.calibration
        try:
            _serve_mjpeg(locator, args.device, args.port, cal["width"], cal["height"])
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
