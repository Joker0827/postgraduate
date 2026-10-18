"""Depth image decoding shared by simulation and camera deployment."""

import numpy as np


def decode_depth_image(image):
    formats = {'32FC1': ('f4', 1.0), '16UC1': ('u2', 0.001)}
    if image.encoding not in formats:
        raise ValueError(f'Unsupported depth encoding: {image.encoding}')
    kind, scale = formats[image.encoding]
    dtype = np.dtype(('>' if image.is_bigendian else '<') + kind)
    row_bytes = image.width * dtype.itemsize
    if image.width <= 0 or image.height <= 0 or image.step < row_bytes:
        raise ValueError('Invalid depth image dimensions or row stride')
    if len(image.data) < image.step * image.height:
        raise ValueError('Depth image payload is shorter than height * step')
    depth = np.ndarray((image.height, image.width), dtype=dtype,
                       buffer=image.data, strides=(image.step, dtype.itemsize))
    return depth.astype(np.float32) * scale
