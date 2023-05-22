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

from __future__ import annotations

import typing

if typing.TYPE_CHECKING:
    from typing import Dict, List, Tuple, Union

    from pandas import DataFrame
    from pandas._typing import CompressionOptions, JSONSerializable

    from .enums import LidarIdentifier

import json
from importlib.resources import path as resources_path
from pathlib import Path

from rosbags.typesys.types import sensor_msgs__msg__PointField as PointField


def register_updated_visualization_msgs__msg__Marker() -> None:  # noqa: N802
    """Update rosbags types with new definition of Marker.msg."""

    def guess_msgtype(path: Path) -> str:
        """Guess message type name from path."""
        name = path.relative_to(path.parents[1]).with_suffix('')
        if 'msg' not in name.parts:
            name = name.parent / 'msg' / name.name
        return str(name)

    from rosbags.typesys import get_types_from_msg, register_types
    from rosbags.typesys.types import FIELDDEFS

    package = f'{__package__}.visualization_msgs'
    resources = {'Marker.msg', 'MeshFile.msg', 'UVCoordinate.msg'}

    msg_types = {}
    for resource in resources:
        with resources_path(package, resource) as fp:
            msgdef = fp.read_text(encoding='utf-8')
            msg_types.update(get_types_from_msg(msgdef, guess_msgtype(fp)))

    del FIELDDEFS['visualization_msgs/msg/Marker']
    register_types(msg_types)


def get_frame_id_from_topic(topic: str, root: str = '/panda', suffix: str = '') -> str:
    """Get the frame ID associated with a ROS topic.

    The frame ID is the last part of the topic string,
    after the root namespace.

    Args:
    ----
    topic (str):
        The name of the ROS topic to extract the frame ID from.
    root (str, optional):
        The prefix string that appears before the frame ID in
        the topic name. Defaults to '/panda'.
    suffix (str, optional):
        The suffix string that appears after the frame ID in
        the topic name. Defaults to ''.

    Returns:
    -------
    str:
        The frame ID string extracted from the topic name.

    Raises:
    ------
    ValueError:
        If the `root` or `suffix` strings are not found in the topic name.
    """
    topic = topic[len(root) :] if root and topic.startswith(root) else topic
    return topic[: -len(suffix)] if suffix and topic.endswith(suffix) else topic


def format_lidar_name_from_id(lidar_id: LidarIdentifier) -> str:
    """Format Lidar name from its identifier.

    Given a LidarIdentifier object, this function returns a string that
    represents the formatted name of the Lidar. The returned name is
    lowercased and has underscores removed, with the last two characters
    converted to uppercase.

    Args:
    ----
    lidar_id (LidarIdentifier):
        The identifier of the Lidar.

    Returns:
    -------
    str:
        The formatted name of the Lidar.
    """
    lidar_name = lidar_id.name.lower().replace('_', '')
    lidar_name = lidar_name[:-2] + lidar_name[-2:].upper()

    return lidar_name


def split_unix_timestamp(timestamp: float) -> Tuple[int, int]:
    """Split UNIX timestamp into seconds and nanoseconds.

    Args:
    ----
    timestamp (float):
        A UNIX timestamp in float format.

    Returns:
    -------
    Tuple[int, int]:
        A tuple of integers representing the seconds
        and nanoseconds of the timestamp respectively.
    """
    sec: int = int(timestamp)
    nsec: int = int((timestamp * 1e9) % 1e9)

    return (sec, nsec)


def get_default_lidar_point_fields() -> Tuple[List[PointField], int]:
    """Get default fields for LIDAR point clouds.

    Returns a list of default `PointField` objects representing the standard
    fields used in a LiDAR point cloud.

    The returned fields are:
        - x: x-coordinate of the point in meters
        - y: y-coordinate of the point in meters
        - z: z-coordinate of the point in meters
        - intensity: measure of the reflected laser pulse strength scaled to
          the range `[0, 1]`
        - class_id: integer code representing the type of object the point
          belongs to (e.g. Bus, Car, Ground)

    Returns
    -------
    Tuple[List[PointField], int]:
    - A list of `PointField` objects representing the standard fields used
        in a LiDAR point cloud.
    - An integer representing the number of bytes between consecutive
        points in the point cloud.
    """
    channels = ('x', 'y', 'z', 'intensity', 'class_id')

    fields = [
        PointField(name=name, offset=(i * 4), datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(channels)
    ]

    point_step = len(fields) * 4

    return fields, point_step


def to_json_with_schema(
    schema: Dict[str, JSONSerializable],
    df_json_record: str,
) -> str:
    """Convert a JSON-encoded record to a JSON string including schema.

    Args:
    ----
    schema (Dict[str, JSONSerializable]):
        A dictionary containing the JSON schema that describes the format
        of the JSON data.
    df_json_record (str):
        A JSON-encoded `DataFrame` using `orient='record'` that conforms
        to the specified schema.

    Returns:
    -------
    str:
        A string representation of the JSON-encoded record, including
        the JSON schema.
    """
    table_schema = json.dumps(schema)
    return f'{{"schema": {table_schema}, "data": [{df_json_record}]}}'


def save_cuboid_data_frame(
    df: DataFrame,
    path: Union[Path, str],
    filename: str,
    compression: CompressionOptions = 'gzip',
) -> None:
    """Save the provided pandas DataFrame containing cuboid data to disk.

    Args:
    ----
    df (DataFrame):
        The pandas DataFrame containing the cuboid data to save.
    path (Union[Path, str]):
        The path to the directory where the cuboid data should be saved.
    filename (str):
        The name of the file to which the cuboid data should be saved.
    compression (CompressionOptions, optional):
        The compression method to use when saving the data. Valid options
        are 'infer', 'bz2', 'gzip', 'xz', 'zip' and 'zstd'. Defaults to
        'gzip'.

    Returns:
    -------
    None
    """
    pth = Path(path) / 'annotations/cuboids'

    if not pth.exists():
        pth.mkdir(parents=True)
    df.to_pickle(pth / filename, compression)
