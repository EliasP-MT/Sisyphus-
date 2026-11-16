"""Box evidence photos: contours drawn, cropped around the face, decodable JPEG."""

import base64
import unittest

import cv2
import numpy as np

from mailbox_slam.photos import box_photo
from tests.test_letterbox_locator import CAL, GEOMETRY, LetterboxLocator, render


class PhotoTests(unittest.TestCase):
    def test_photo_is_a_cropped_jpeg_with_contours(self):
        locator = LetterboxLocator(CAL, GEOMETRY, ocr=False)
        image = render([0.0, 0.0, 0.0], [-0.13, -0.02, 0.55])  # far enough for the 23 cm slot to fit the image
        result = locator.locate(image)
        self.assertIsNotNone(result)
        text = box_photo(image, result, CAL, GEOMETRY)
        photo = cv2.imdecode(np.frombuffer(base64.b64decode(text), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(photo.shape[0], 160)
        self.assertLess(photo.shape[1] * photo.shape[0], image.shape[0] * image.shape[1])
        b, g, r = (photo[:, :, i].astype(int) for i in range(3))
        green = ((g > 120) & (r < 90) & (b < 90)).sum()  # JPEG/resize blur the exact colours
        red = ((r > 150) & (g < 90) & (b < 90)).sum()
        self.assertGreater(green, 20)
        self.assertGreater(red, 20)


if __name__ == "__main__":
    unittest.main()
