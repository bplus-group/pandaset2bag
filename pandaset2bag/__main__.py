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

import sys
from argparse import (
    ArgumentDefaultsHelpFormatter,
    ArgumentParser,
    HelpFormatter,
    Namespace,
)
from importlib.resources import path as resources_path
from typing import List, Optional, Tuple

from rosbags.rosbag2 import Writer


def formatter(prog: str) -> HelpFormatter:
    return ArgumentDefaultsHelpFormatter(prog, width=100)


def parse_arguments() -> Namespace:
    parser = ArgumentParser(formatter_class=formatter)

    parser.add_argument(
        '-d',
        '--dataset-dir',
        type=str,
        required=True,
        help='the directory path of the dataset',
    )
    parser.add_argument(
        '-i',
        '--sequence-id',
        type=str,
        required=True,
        help='the ID of the sequence to be converted',
    )
    parser.add_argument(
        '-o',
        '--output',
        default='',
        type=str,
        help="the save path of the rosbag file; if '', i.e "
        'the rosbag file will be saved in the current '
        'working directory with the name '
        "'pandasetbag_{sequence_id}'",
    )
    parser.add_argument(
        '-s',
        '--max-image-size',
        nargs='+',
        default=[],
        type=int,
        help='maximum image size to convert to',
    )
    parser.add_argument(
        '-t',
        '--type',
        default='raw',
        type=str,
        choices=['raw', 'compressed', 'raw_compressed'],
        help='type to be used to convert an image',
    )
    parser.add_argument(
        '-f',
        '--format',
        default='jpeg',
        type=str,
        choices=['jpeg', 'png'],
        help='image format used for compression',
    )
    parser.add_argument(
        '-q',
        '--jpeg-quality',
        default=75,
        type=int,
        help='image compression quality for JPEG format; '
        'the image quality, on a scale from 0 (worst) '
        'to 95 (best), or the string keep. Values '
        'above 95 should be avoided; 100 disables '
        'portions of the JPEG compression algorithm, '
        'and results in large files with hardly any '
        'gain in image quality; the value keep is only '
        'valid for JPEG files and will retain the '
        'original image quality level, subsampling, '
        'and qtables',
    )
    parser.add_argument(
        '-l',
        '--png-compress-level',
        default=6,
        type=int,
        choices=range(10),
        help='image compression level for PNG format; '
        'ZLIB compression level, a number between 0 '
        'and 9: 1 gives best speed, 9 gives best '
        'compression, 0 gives no compression at all. '
        'When optimize option is True compress_level '
        'has no effect (it is set to 9 regardless of a '
        'value passed); the value is only valid for '
        'PNG files and will be otherwise ignored',
    )
    parser.add_argument(
        '-O',
        '--png-optimize',
        action='store_true',
        help='image optimization status for PNG format; '
        'if True, instructs the PNG writer to make the '
        'output file as small as possible; this '
        'includes extra processing in order to find '
        'optimal encoder settings; the value is only '
        'valid for PNG files and will be otherwise '
        'ignored',
    )
    parser.add_argument(
        '-m',
        '--mode',
        default='none',
        type=str,
        choices=['none', 'file', 'message'],
        help='compression mode for rosbag file.',
    )
    parser.add_argument(
        '-p',
        '--profiles',
        default='',
        type=str,
        help='QoS profiles filepath (YAML) to be offered for published topics.',
    )
    parser.add_argument(
        '-c',
        '--cuboids',
        action='store_true',
        help='save cuboids `DataFrame` status; '
        'if True, save the converted cuboids '
        "`DataFrame`'s as `.pkl.gz` files, "
        'representing the properties of the cuboids '
        'in normalized ego coordinates',
    )
    parser.add_argument('-v', '--version', action='version', version=get_version())

    args: Namespace = parser.parse_args()
    return args


def get_version() -> str:
    with resources_path(__package__, 'VERSION') as fp:
        return fp.open().read().strip()


def main() -> int:
    def valid_imgsz(imgsz: List[int]) -> Optional[Tuple[int, int]]:
        if not imgsz:
            return None

        return tuple(imgsz) * 2 if len(imgsz) < 2 else tuple(imgsz)[:2]  # type: ignore[return-value]  # noqa: E501

    try:
        args = parse_arguments()

        from .enums import CompressedImageFormat, ImageConvertType
        from .pandaset2bag_converter import PandaSet2BagConverter

        converter = PandaSet2BagConverter(args.dataset_dir)

        image_convert_type = ImageConvertType[args.type.upper()]
        image_format = CompressedImageFormat[args.format.upper()]
        compression_mode = Writer.CompressionMode[args.mode.upper()]

        converter.max_image_size = valid_imgsz(args.max_image_size)
        converter.image_convert_type = image_convert_type
        converter.image_format = image_format
        converter.jpeg_quality = args.jpeg_quality
        converter.png_compress_level = args.png_compress_level
        converter.png_optimize = args.png_optimize
        converter.compression_mode = compression_mode
        converter.offered_qos_profiles = args.profiles
        converter.save_cuboids_df = args.cuboids

        converter.convert(args.sequence_id.zfill(3), args.output)
    except KeyboardInterrupt:
        sys.stderr.write('\n')
        return 1
    except Exception as e:  # noqa: BLE001
        sys.stderr.write('Error:{} {}{}\n'.format('\033[91m', '\033[0m', e))
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
