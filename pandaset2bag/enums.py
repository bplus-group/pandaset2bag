#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2023 b-plus technologies GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from enum import Enum


class CompressedImageFormat(str, Enum):
    """An enumeration class for image compression formats.

    This class defines an enumeration of image compression formats, which can
    be used to specify the format in which an image should be compressed.
    The two options are:
    - JPEG: The image is compressed using the JPEG format.
    - PNG: The image is compressed using the PNG format.

    Attributes
    ----------
    JPEG (str):
        value 'jpeg', representing JPEG compression format
    PNG (str):
        value 'png', representing PNG compression format
    """

    JPEG = 'jpeg'
    PNG = 'png'


class ImageConvertType(int, Enum):
    """An enumeration class for image conversion type.

    This class defines an enumeration of image conversion types, which can
    be used to specify how an image should be converted. The three options are:
    - RAW: The image will be converted in its raw format, without any
           compression or modification.
    - COMPRESSED: The image will be converted in its compressed format.
    - RAW_COMPRESSED: The image will be converted in its raw and compressed
                      format.

    Attributes
    ----------
    RAW (int):
        value 0, representing raw format of image
    COMPRESSED (int):
        value 1, representing compressed format of image
    RAW_COMPRESSED (int):
        value 2, representing both raw and compressed format of image
    """

    RAW = 0
    COMPRESSED = 1
    RAW_COMPRESSED = 2


class LidarIdentifier(int, Enum):
    """An enumeration class for LiDAR sensor models.

    This class defines an enumeration of LIDAR sensor models, which can be
    used to specify the type of LiDAR sensor being used. The two options are:
    - PANDAR_64: Pandar64 LiDAR sensor model.
    - PANDAR_GT: PandarGT LiDAR sensor model.

    Attributes
    ----------
    PANDAR_64 (int):
        value 0, representing Pandar64 LiDAR sensor model
    PANDAR_GT (int):
        value 1, representing PandarGT LiDAR sensor model
    """

    PANDAR_64 = 0
    PANDAR_GT = 1
