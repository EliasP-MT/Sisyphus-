"""Hardware-free checks: synthetic letterbox face rendered at known poses."""

import unittest

import cv2
import numpy as np

from letterbox_locator import HERE, LetterboxLocator, detect_labels, load_json
from recipient_names import match_recipient

GEOMETRY = load_json(HERE / "config/letterbox.json")
CAL = dict(
    fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480, distortion=[0, 0, 0, 0, 0]
)
PX_PER_M = 4000


# The synthetic face follows config/letterbox.json, so a change of the box measurements does not
# break the tests: label at the left edge of the face, vertically centred; slot at the measured offset.
LABEL_W, LABEL_H = GEOMETRY["label_width_m"], GEOMETRY["label_height_m"]
FACE_W, FACE_H = GEOMETRY["box_width_m"], GEOMETRY["box_height_m"]
LABEL_CX, LABEL_CY = LABEL_W / 2 + 0.01, FACE_H / 2  # label 1 cm from the left edge (as measured before)
SLOT_CX = LABEL_CX + GEOMETRY["slot_from_label_center_m"][0]


def face_texture(name="Mr. Jan Smith"):
    """Front face (box_width x box_height), origin top-left, as seen from the robot."""
    w, h = int(FACE_W * PX_PER_M), int(FACE_H * PX_PER_M)
    tex = np.full((h, w, 3), 190, np.uint8)

    def m(v):
        return int(round(v * PX_PER_M))

    sw, sh = GEOMETRY["slot_width_m"], GEOMETRY["slot_height_m"]
    cv2.rectangle(
        tex,
        (m(SLOT_CX - sw / 2), m(LABEL_CY - sh / 2)),
        (min(w - 1, m(SLOT_CX + sw / 2)), m(LABEL_CY + sh / 2)),
        (25, 25, 25),
        -1,
    )  # slot (dark hole)
    cv2.rectangle(
        tex,
        (m(LABEL_CX - LABEL_W / 2), m(LABEL_CY - LABEL_H / 2)),
        (m(LABEL_CX + LABEL_W / 2), m(LABEL_CY + LABEL_H / 2)),
        (15, 15, 15),
        -1,
    )  # label border
    inset = 0.004
    cv2.rectangle(
        tex,
        (m(LABEL_CX - LABEL_W / 2 + inset), m(LABEL_CY - LABEL_H / 2 + inset)),
        (m(LABEL_CX + LABEL_W / 2 - inset), m(LABEL_CY + LABEL_H / 2 - inset)),
        (250, 250, 250),
        -1,
    )  # label interior
    cv2.putText(
        tex, name, (m(LABEL_CX - LABEL_W / 2 + 0.006), m(LABEL_CY + 0.002)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
    )
    return tex


def render(rvec, tvec, tex=None):
    """Project the face (z=0 plane, metres) into a 640x480 image with a pinhole camera."""
    tex = face_texture() if tex is None else tex
    k = np.array([[CAL["fx"], 0, CAL["cx"]], [0, CAL["fy"], CAL["cy"]], [0, 0, 1]])
    r, _ = cv2.Rodrigues(np.asarray(rvec, float))
    homography = (
        k @ np.column_stack([r[:, 0], r[:, 1], tvec]) @ np.diag([1 / PX_PER_M, 1 / PX_PER_M, 1])
    )
    background = np.full((480, 640, 3), 120, np.uint8)
    return cv2.warpPerspective(
        tex, homography, (640, 480), dst=background, borderMode=cv2.BORDER_TRANSPARENT
    )


def expected_slot(rvec, tvec):
    r, _ = cv2.Rodrigues(np.asarray(rvec, float))
    return r @ np.array([SLOT_CX, LABEL_CY, 0]) + np.asarray(tvec)  # slot centre in face coordinates


class LocatorTests(unittest.TestCase):
    def setUp(self):
        self.locator = LetterboxLocator(CAL, GEOMETRY)

    def test_slot_centre_at_several_poses(self):
        poses = [
            ([0, 0, 0], [-0.10, -0.04, 0.35]),  # straight on, 35 cm
            ([0.15, -0.35, 0.05], [-0.12, -0.03, 0.40]),  # tilted / yawed
            ([-0.2, 0.3, -0.08], [-0.06, -0.05, 0.30]),
        ]
        for rvec, tvec in poses:
            result = self.locator.locate_all(render(rvec, tvec))[0]
            self.assertIsNotNone(result, f"label not found at {rvec}, {tvec}")
            label = np.array([LABEL_CX, LABEL_CY, 0.0])
            r = cv2.Rodrigues(np.asarray(rvec, float))[0]
            label_error = np.linalg.norm(np.asarray(result["label_center_m"]) - (r @ label + np.asarray(tvec)))
            self.assertLess(label_error, 0.005, f"label error {label_error * 1000:.1f} mm at {rvec}, {tvec}")
            # The slot is 16 cm from the label centre: a few degrees of tilt error on a 5 cm label
            # (planar ambiguity, 1 px of contour bias) moves it by up to ~2.5 cm along the viewing axis.
            error = np.linalg.norm(np.asarray(result["slot_center_m"]) - expected_slot(rvec, tvec))
            self.assertLess(error, 0.03, f"slot error {error * 1000:.1f} mm at {rvec}, {tvec}")
            normal = cv2.Rodrigues(np.asarray(rvec, float))[0][:, 2]
            self.assertGreater(np.dot(result["slot_normal"], normal), 0.99)

    def test_slot_itself_is_not_a_label(self):
        labels = detect_labels(render([0, 0, 0], [-0.10, -0.04, 0.35]))
        self.assertEqual(len(labels), 1)

    def test_label_with_large_text_touching_the_border_is_still_found(self):
        # Lettering that touches the black border splits the white interior into several holes
        # (a real camera-2 frame lost two of three stacked labels this way).
        image = np.full((480, 640, 3), 140, np.uint8)
        cv2.rectangle(image, (200, 150), (260, 200), (15, 15, 15), -1)
        cv2.rectangle(image, (205, 155), (255, 195), (250, 250, 250), -1)
        cv2.putText(image, "JAN", (204, 173), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 2)
        cv2.putText(image, "SMITH", (204, 192), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 2)
        self.assertEqual(len(detect_labels(image)), 1)

    def test_slot_outline_is_a_rectangle_for_a_face_seen_head_on(self):
        from letterbox_locator import slot_outline_px

        # Head-on, the slot (coplanar with the label) must be drawn as a rectangle: its edges
        # parallel to the label's, whatever tilt the 3D pose estimate happens to carry.
        for tvec in ([-0.1, -0.04, 0.35], [-0.08, -0.03, 0.30], [-0.12, -0.05, 0.40]):
            image = render([0, 0, 0], tvec)
            result = self.locator.locate_all(image)[0]
            outline = slot_outline_px(result, CAL, GEOMETRY)[:4]
            top, bottom = outline[1] - outline[0], outline[2] - outline[3]
            left, right = outline[3] - outline[0], outline[2] - outline[1]
            self.assertLess(abs(top[1]) + abs(bottom[1]), 2.0, "top/bottom edges must be horizontal")
            self.assertLess(abs(left[0]) + abs(right[0]), 2.0, "left/right edges must be vertical")
            self.assertAlmostEqual(np.linalg.norm(top), np.linalg.norm(bottom), delta=1.5)
            # and it sits where the slot is drawn on the face
            slot_px = outline[:4].mean(axis=0)
            expected, _ = cv2.projectPoints(
                np.array([[SLOT_CX, LABEL_CY, 0.0]]), np.zeros(3), np.asarray(tvec, float),
                np.array([[CAL["fx"], 0, CAL["cx"]], [0, CAL["fy"], CAL["cy"]], [0, 0, 1.0]]), np.zeros(5))
            self.assertLess(np.linalg.norm(slot_px - expected.reshape(2)), 3.0)

    def test_no_label(self):
        self.assertEqual(self.locator.locate_all(np.full((480, 640, 3), 128, np.uint8)), [])

    def test_recipient_matching_tolerates_ocr_errors(self):
        names = load_json(HERE.parent / "base_ocr/recipients.json")
        self.assertEqual(match_recipient("Jan Smith", names)[0], "Jan Smith")
        self.assertEqual(match_recipient("Yuki Tanaka", names)[0], "Yuki Tanaka")
        self.assertEqual(match_recipient("lvan Petr0v", names)[0], "Ivan Petrov")
        self.assertIsNone(match_recipient("", names)[0])
        self.assertIsNone(match_recipient("Bonjour", names)[0])


if __name__ == "__main__":
    unittest.main()
