import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'YOPO'))
from ros_utils import decode_depth_image


class DepthImageTests(unittest.TestCase):
    def test_float_meters_with_padded_rows(self):
        rows = [np.array([1.25, 2.5], dtype='<f4').tobytes() + b'PAD!',
                np.array([3.0, 4.0], dtype='<f4').tobytes() + b'PAD!']
        msg = SimpleNamespace(width=2, height=2, step=12, is_bigendian=False,
                              encoding='32FC1', data=b''.join(rows))
        np.testing.assert_array_equal(decode_depth_image(msg), [[1.25, 2.5], [3.0, 4.0]])

    def test_uint16_big_endian_millimeters(self):
        msg = SimpleNamespace(width=2, height=1, step=4, is_bigendian=True,
                              encoding='16UC1', data=np.array([1000, 2500], dtype='>u2').tobytes())
        np.testing.assert_allclose(decode_depth_image(msg), [[1.0, 2.5]])

    def test_rejects_short_payload(self):
        msg = SimpleNamespace(width=2, height=2, step=8, is_bigendian=False,
                              encoding='32FC1', data=b'\0' * 8)
        with self.assertRaisesRegex(ValueError, 'payload'):
            decode_depth_image(msg)

    def test_rejects_unsupported_encoding(self):
        with self.assertRaisesRegex(ValueError, 'Unsupported'):
            decode_depth_image(SimpleNamespace(encoding='rgb8'))


if __name__ == '__main__':
    unittest.main()
