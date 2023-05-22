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

import os
import typing
from io import BytesIO

if typing.TYPE_CHECKING:
    from pathlib import Path

    from pandaset.sequence import Sequence
    from rosbags.interfaces import Connection

import numpy as np
import pandas as pd
from pandas.io.json import build_table_schema
from pandaset import DataSet
from rich import print
from rich.console import Console
from rich.progress import track

if os.getenv('UPDATED_VISUALIZATION_MSG_MARKER', 'false').lower() == 'true':
    from .utils import register_updated_visualization_msgs__msg__Marker

    register_updated_visualization_msgs__msg__Marker()

    from rosbags.typesys.types import (  # type: ignore[attr-defined]  # noqa: I001, RUF100, E501
        visualization_msgs__msg__MeshFile as MeshFile,
    )

    print('[gold1]█████[/gold1] Using [bold]UPDATED[/bold] visualization_msgs')
else:
    print('[gold1]█████[/gold1] Using [bold]DEFAULT[/bold] visualization_msgs')
from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr
from rosbags.typesys.types import builtin_interfaces__msg__Duration as Duration
from rosbags.typesys.types import builtin_interfaces__msg__Time as Time
from rosbags.typesys.types import geometry_msgs__msg__Point as Point
from rosbags.typesys.types import geometry_msgs__msg__Pose as Pose
from rosbags.typesys.types import geometry_msgs__msg__Quaternion as Quaternion
from rosbags.typesys.types import geometry_msgs__msg__Transform as Transform
from rosbags.typesys.types import (
    geometry_msgs__msg__TransformStamped as TransformStamped,
)
from rosbags.typesys.types import geometry_msgs__msg__Vector3 as Vector3
from rosbags.typesys.types import sensor_msgs__msg__CameraInfo as CameraInfo
from rosbags.typesys.types import sensor_msgs__msg__CompressedImage as CompressedImage
from rosbags.typesys.types import sensor_msgs__msg__Image as Image
from rosbags.typesys.types import sensor_msgs__msg__NavSatFix as NavSatFix
from rosbags.typesys.types import sensor_msgs__msg__NavSatStatus as NavSatStatus
from rosbags.typesys.types import sensor_msgs__msg__PointCloud2 as PointCloud2
from rosbags.typesys.types import sensor_msgs__msg__RegionOfInterest as RegionOfInterest
from rosbags.typesys.types import std_msgs__msg__ColorRGBA as ColorRGBA
from rosbags.typesys.types import std_msgs__msg__Header as Header
from rosbags.typesys.types import (
    tf2_msgs__msg__TFMessage as TFMessage,
)
from rosbags.typesys.types import visualization_msgs__msg__Marker as Marker
from rosbags.typesys.types import visualization_msgs__msg__MarkerArray as MarkerArray
from scipy.spatial.transform import Rotation as R  # noqa: N817

from .enums import CompressedImageFormat, ImageConvertType, LidarIdentifier
from .geometry import (
    cuboid_data_frame_to_normative_ego,
    ego_vehicle_to_normative_ego,
    get_flatten_calibration_matrices,
    lidar_data_frame_to_ego,
    lidar_frame_to_normative,
    mat_encoded_as_pose,
    pose_to_normative,
    transform_origin_to_target,
)
from .label_colors import LABEL_COLORMAP
from .utils import (
    format_lidar_name_from_id,
    get_default_lidar_point_fields,
    get_frame_id_from_topic,
    save_cuboid_data_frame,
    split_unix_timestamp,
    to_json_with_schema,
)


class PandaSet2BagConverter:  # noqa: D101
    EGO_NAMESPACE = '/panda/ego_vehicle'

    def __init__(self, dataset_dir: Path | str):
        """Initialize a PandaSet2BagConverter instance.

        Args:
        ----
        dataset_dir (Path, str):
            The directory path of the dataset.

        """
        super().__init__()

        self._dataset: DataSet = DataSet(dataset_dir)
        self._sequence: Sequence
        self._rosbag_writer: Writer

        self._max_imgsz: tuple[int, int] | None = None
        self._image_convert_type: ImageConvertType = ImageConvertType.RAW
        self._image_format: CompressedImageFormat = CompressedImageFormat.JPEG
        self._jpeg_quality: int = 75
        self._png_compress_level: int = 6
        self._png_optimize: bool = False
        self._compression_mode = Writer.CompressionMode.NONE
        self._save_cuboids_df: bool = False

    @property
    def max_image_size(self) -> tuple[int, int] | None:
        """Maximum image size to convert to.

        Returns
        -------
        Optional[Tuple[int, int]]:
            A tuple of integers representing the maximum width and height
            of the images. Default is None, i.e no scaling.
        """
        return self._max_imgsz

    @max_image_size.setter
    def max_image_size(self, value: tuple[int, int] | None) -> None:
        if value:
            self._max_imgsz = value

    @property
    def image_convert_type(self) -> ImageConvertType:
        """Type to be used to convert an image.

        Returns
        -------
        ImageConvertType:
            The type to be used to convert an image.
            Default is RAW, i.e. raw image format.
        """
        return self._image_convert_type

    @image_convert_type.setter
    def image_convert_type(self, value: ImageConvertType) -> None:
        self._image_convert_type = value

    @property
    def image_format(self) -> CompressedImageFormat:
        """Image format used for compression.

        Returns
        -------
        CompressedImageFormat:
            Image format to be used for compression. Default is JPEG.
        """
        return self._image_format

    @image_format.setter
    def image_format(self, value: CompressedImageFormat) -> None:
        self._image_format = value

    @property
    def jpeg_quality(self) -> int:
        """Image compression quality for JPEG format.

        The image quality, on a scale from 0 (worst) to 95 (best), or
        the string keep. Values above 95 should be avoided; 100 disables
        portions of the JPEG compression algorithm, and results in large files
        with hardly any gain in image quality. The value keep is only valid
        for JPEG files and will retain the original image quality level,
        subsampling, and qtables.

        Returns
        -------
        int: JPEG quality [0, 100]. Default is 75.
        """
        return self._jpeg_quality

    @jpeg_quality.setter
    def jpeg_quality(self, value: int) -> None:
        self._jpeg_quality = max(min(value, 0), 100)

    @property
    def png_compress_level(self) -> int:
        """Image compression level for PNG format.

        ZLIB compression level, a number between 0 and 9: 1 gives best
        speed, 9 gives best compression, 0 gives no compression at all.
        When optimize option is True compress_level has no effect (it
        is set to 9 regardless of a value passed). The value is only
        valid for PNG files and will be otherwise ignored.

        Returns
        -------
        int: PNG compress level [0, 9]. Default is 7.
        """
        return self._png_compress_level

    @png_compress_level.setter
    def png_compress_level(self, value: int) -> None:
        self._png_compress_level = max(min(value, 0), 9)

    @property
    def png_optimize(self) -> bool:
        """Image optimization status for PNG format.

        If True, instructs the PNG writer to make the
        output file as small as possible. This includes extra
        processing in order to find optimal encoder settings. The
        value is only valid for PNG files and will be otherwise
        ignored.

        Returns
        -------
        bool: Image optimization status. Default is False.
        """
        return self._png_optimize

    @png_optimize.setter
    def png_optimize(self, value: bool) -> None:
        self._png_optimize = value

    @property
    def compression_mode(self) -> Writer.CompressionMode:
        """Compression mode for rosbag file.

        Returns
        -------
        Writer.CompressionMode:
            The compression mode to be applied to the rosbag file.
            Default is NONE, i.e no compression.
        """
        return self._compression_mode

    @compression_mode.setter
    def compression_mode(self, value: Writer.CompressionMode) -> None:
        self._compression_mode = value

    @property
    def save_cuboids_df(self) -> bool:
        """Save cuboids `DataFrame` status.

        If True, save the converted cuboids `DataFrame`'s as `.pkl.gz` files,
        representing the properties of the cuboids in normalized ego
        coordinates.

        Returns
        -------
        bool: Cuboids `DataFrame` save status. Default is False.
        """
        return self._save_cuboids_df

    @save_cuboids_df.setter
    def save_cuboids_df(self, value: bool) -> None:
        self._save_cuboids_df = value

    def _convert_camera_image_to_topic(self, camera_id: str) -> None:
        """Convert the images of a specified camera to a ROS topic.

        Applies downsizing to the images if a maximum image size is provided.

        Args:
        ----
        camera_id (str):
            The ID of the camera for which the images are to be converted.

        Returns:
        -------
        None
        """
        topic = f'{self.EGO_NAMESPACE}/{camera_id}/image'
        conn = self._rosbag_writer.add_connection(topic, Image.__msgtype__)

        camera = self._sequence.camera[camera_id]

        for img, timestamp in zip(camera, camera.timestamps):
            if self.max_image_size is not None and all(self.max_image_size):
                img.thumbnail(self.max_image_size)

            sec, nsec = split_unix_timestamp(timestamp)

            message = Image(
                Header(
                    stamp=Time(sec=sec, nanosec=nsec),
                    frame_id=get_frame_id_from_topic(conn.topic, suffix='/image')[1:],
                ),
                height=img.height,
                width=img.width,
                encoding='rgb8',
                is_bigendian=False,
                step=img.width * len(img.getbands()),
                data=np.frombuffer(img.tobytes(), dtype=np.uint8),
            )

            self._rosbag_writer.write(
                conn,
                timestamp * 1e9,
                serialize_cdr(message, message.__msgtype__),
            )

    def _convert_camera_image_to_compressed_topic(self, camera_id: str) -> None:
        """Convert the images of a specified camera to a ROS topic.

        Applies downsizing to the images if a maximum image size is provided.

        Args:
        ----
        camera_id (str):
            The ID of the camera for which the images are to be converted.

        Returns:
        -------
        None
        """
        topic = f'{self.EGO_NAMESPACE}/{camera_id}/image/compressed'
        conn = self._rosbag_writer.add_connection(topic, CompressedImage.__msgtype__)

        camera = self._sequence.camera[camera_id]

        for img, timestamp in zip(camera, camera.timestamps):
            if self.max_image_size is not None and all(self.max_image_size):
                img.thumbnail(self.max_image_size)

            buffer = BytesIO()
            img.save(
                buffer,
                format=self.image_format.value,
                quality=self.jpeg_quality,
                compress_level=self.png_compress_level,
                optimize=self.png_optimize,
            )

            sec, nsec = split_unix_timestamp(timestamp)

            message = CompressedImage(
                Header(
                    stamp=Time(sec=sec, nanosec=nsec),
                    frame_id=get_frame_id_from_topic(
                        conn.topic,
                        suffix='/image/compressed',
                    )[1:],
                ),
                format=self.image_format.value,
                data=np.frombuffer(buffer.getvalue(), dtype=np.uint8),
            )

            self._rosbag_writer.write(
                conn,
                timestamp * 1e9,
                serialize_cdr(message, message.__msgtype__),
            )

    def _convert_camera_info_to_topic(self, camera_id: str) -> None:
        """Convert the camera meta data of a specified camera to a ROS topic.

        Args:
        ----
        camera_id (str):
            The ID of the camera for which the images are to be converted.

        Returns:
        -------
        None
        """
        topic = f'{self.EGO_NAMESPACE}/{camera_id}/camera_info'
        conn = self._rosbag_writer.add_connection(topic, CameraInfo.__msgtype__)

        camera = self._sequence.camera[camera_id]

        # image previously inplace downsized
        imgsz = camera[0].size

        d, k, r, p = get_flatten_calibration_matrices(camera.intrinsics, imgsz)

        for img, timestamp in zip(camera, camera.timestamps):
            sec, nsec = split_unix_timestamp(timestamp)

            message = CameraInfo(
                Header(
                    stamp=Time(sec=sec, nanosec=nsec),
                    frame_id=get_frame_id_from_topic(conn.topic, suffix='/camera_info')[
                        1:
                    ],
                ),
                height=img.height,
                width=img.width,
                distortion_model='plumb_bob',
                d=d,
                k=k,
                r=r,
                p=p,
                binning_x=0,
                binning_y=0,
                roi=RegionOfInterest(
                    x_offset=0,
                    y_offset=0,
                    height=0,
                    width=0,
                    do_rectify=False,
                ),
            )

            self._rosbag_writer.write(
                conn,
                timestamp * 1e9,
                serialize_cdr(message, message.__msgtype__),
            )

    def _convert_cameras(self) -> None:
        """Convert the camera images and camera info for the sequence.

        Returns
        -------
        None
        """
        for camera_id in track(
            self._sequence.camera.keys(),
            description='Converting cameras...',
        ):
            if self.image_convert_type in (
                ImageConvertType.RAW,
                ImageConvertType.RAW_COMPRESSED,
            ):
                self._convert_camera_image_to_topic(camera_id)

            if self.image_convert_type in (
                ImageConvertType.COMPRESSED,
                ImageConvertType.RAW_COMPRESSED,
            ):
                self._convert_camera_image_to_compressed_topic(camera_id)

            self._convert_camera_info_to_topic(camera_id)

    def _convert_gps_to_topic(self) -> None:
        """Convert the GPS data for the sequence to a ROS topic.

        Returns
        -------
        None
        """
        topic = f'{self.EGO_NAMESPACE}/gnss'
        conn = self._rosbag_writer.add_connection(topic, NavSatFix.__msgtype__)

        for gps, timestamp in track(
            zip(self._sequence.gps, self._sequence.timestamps),
            description='Converting GPS...',
            total=len(self._sequence.gps.data),
        ):
            sec, nsec = split_unix_timestamp(timestamp)

            message = NavSatFix(
                Header(
                    stamp=Time(sec=sec, nanosec=nsec),
                    frame_id=get_frame_id_from_topic(conn.topic)[1:],
                ),
                NavSatStatus(status=0, service=0),
                latitude=gps['lat'],
                longitude=gps['long'],
                altitude=gps['height'],
                position_covariance=np.zeros(9, dtype=np.float64),
                position_covariance_type=0,
            )

            self._rosbag_writer.write(
                conn,
                timestamp * 1e9,
                serialize_cdr(message, message.__msgtype__),
            )

    def _convert_lidar_to_topic(self, lidar_id: LidarIdentifier) -> None:
        """Convert LiDAR data to a ROS topic for a given sensor_id.

        Args:
        ----
        lidar_id (LidarIdentifier):
            LiDAR sensor to be converted to topic.

        Returns:
        -------
        None
        """
        lidar_name = format_lidar_name_from_id(lidar_id)

        topic = f'{self.EGO_NAMESPACE}/{lidar_name}'
        conn = self._rosbag_writer.add_connection(topic, PointCloud2.__msgtype__)

        self._sequence.lidar.set_sensor(lidar_id.value)
        lidar = self._sequence.lidar

        for idx, (data_frame, pose, timestamp) in track(
            enumerate(zip(lidar.data, lidar.poses, lidar.timestamps)),
            description=f'Converting {lidar_name} lidar...',
            total=len(lidar.data),
        ):
            sec, nsec = split_unix_timestamp(timestamp)
            frame_width = len(data_frame.index)
            fields, point_step = get_default_lidar_point_fields()

            df: pd.DataFrame
            if self._sequence.semseg is not None:
                df = pd.concat(
                    (data_frame, self._sequence.semseg[idx]),
                    axis=1,
                ).dropna()
            else:
                df = data_frame.assign(**{'class': -1})

            ego_frame = lidar_data_frame_to_ego(df, pose)
            norm_frame = lidar_frame_to_normative(ego_frame)

            message = PointCloud2(
                Header(
                    stamp=Time(sec=sec, nanosec=nsec),
                    frame_id=get_frame_id_from_topic(conn.topic)[1:],
                ),
                height=1,
                width=frame_width,
                fields=fields,
                is_bigendian=False,
                point_step=point_step,
                row_step=frame_width * point_step,
                data=np.frombuffer(norm_frame.tobytes(), dtype=np.uint8),
                is_dense=True,
            )

            self._rosbag_writer.write(
                conn,
                timestamp * 1e9,
                serialize_cdr(message, message.__msgtype__),
            )

    def _convert_lidars(self) -> None:
        """Convert LiDAR's for the sequence.

        Returns
        -------
        None
        """
        for lidar_id in LidarIdentifier:
            self._convert_lidar_to_topic(lidar_id)

    def _convert_cuboids_to_topic(self) -> None:
        topic = '/panda/markers'
        conn = self._rosbag_writer.add_connection(topic, MarkerArray.__msgtype__)

        for idx, (lidar_pose, cuboid, timestamp) in track(
            enumerate(
                zip(
                    self._sequence.lidar.poses,
                    self._sequence.cuboids,
                    self._sequence.timestamps,
                ),
            ),
            description='Converting cuboids...',
            total=len(self._sequence.timestamps.data),
        ):
            df = cuboid_data_frame_to_normative_ego(cuboid, lidar_pose)

            table_schema = None
            if not table_schema:
                table_schema = build_table_schema(df)

            if self._save_cuboids_df:
                path = self._rosbag_writer.path
                save_cuboid_data_frame(df, path, f'{str(idx).zfill(2)}.pkl.gz')

            markers: list[Marker] = []
            marker_ids: list[str] = []

            sec, nsec = split_unix_timestamp(timestamp)

            for row in df.itertuples():
                if row.uuid not in marker_ids:
                    marker_ids.append(row.uuid)

                marker_dict = {
                    'header': Header(
                        stamp=Time(sec=sec, nanosec=nsec),
                        frame_id='ego_vehicle',
                    ),
                    'ns': '',
                    'id': marker_ids.index(row.uuid),
                    'type': Marker.CUBE,
                    'action': Marker.MODIFY,
                    'pose': Pose(
                        Point(*row[6:9]),
                        Quaternion(*R.from_euler('zyx', [row.yaw, 0, 0]).as_quat()),
                    ),
                    'scale': Vector3(*row[9:12]),
                    'color': ColorRGBA(*LABEL_COLORMAP[row.label], a=0.3),
                    'lifetime': Duration(sec=0, nanosec=0),
                    'frame_locked': False,
                    'points': [],
                    'colors': [],
                    'text': to_json_with_schema(
                        table_schema,
                        cuboid.iloc[[row.Index]].to_json(orient='records'),
                    ),
                    'mesh_resource': '',
                    'mesh_use_embedded_materials': False,
                }

                if (
                    os.getenv('UPDATED_VISUALIZATION_MSG_MARKER', 'false').lower()
                    == 'true'
                ):
                    marker_dict.update(
                        {
                            'texture_resource': '',
                            'texture': CompressedImage(
                                Header(stamp=Time(sec=0, nanosec=0), frame_id=''),
                                format='',
                                data=np.empty(0, dtype=np.uint8),
                            ),
                            'uv_coordinates': [],
                            'mesh_file': MeshFile(
                                filename='',
                                data=np.empty(0, dtype=np.uint8),
                            ),
                        },
                    )

                markers.append(Marker(**marker_dict))  # type: ignore[arg-type]

            message = MarkerArray(markers=markers)
            self._rosbag_writer.write(
                conn,
                timestamp * 1e9,
                serialize_cdr(message, message.__msgtype__),
            )

    def _generate_stamped_transform_ego_vehicle(self, connection: Connection) -> None:
        lidar = self._sequence.lidar

        for pose, timestamp in track(
            zip(lidar.poses, lidar.timestamps),
            description='Generating /tf for ego_vehicle...',
            total=len(self._sequence.lidar.poses),
        ):
            sec, nsec = split_unix_timestamp(timestamp)
            pose_norm_ego = ego_vehicle_to_normative_ego(pose)

            transforms = TransformStamped(
                header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id='world'),
                child_frame_id=get_frame_id_from_topic(self.EGO_NAMESPACE)[1:],
                transform=Transform(
                    translation=Vector3(
                        x=pose_norm_ego['position']['x'],
                        y=pose_norm_ego['position']['y'],
                        z=pose_norm_ego['position']['z'],
                    ),
                    rotation=Quaternion(
                        x=pose_norm_ego['heading']['x'],
                        y=pose_norm_ego['heading']['y'],
                        z=pose_norm_ego['heading']['z'],
                        w=pose_norm_ego['heading']['w'],
                    ),
                ),
            )

            message = TFMessage(transforms=[transforms])

            self._rosbag_writer.write(
                connection,
                timestamp * 1e9,
                serialize_cdr(message, message.__msgtype__),
            )

    def _generate_stamped_transform_lidar(
        self,
        lidar_id: LidarIdentifier,
        connection: Connection,
    ) -> None:
        lidar_name = format_lidar_name_from_id(lidar_id)

        for timestamp in track(
            self._sequence.lidar.timestamps,
            description=f'Generating /tf for {lidar_name}...',
        ):
            sec, nsec = split_unix_timestamp(timestamp)
            frame_id = get_frame_id_from_topic(self.EGO_NAMESPACE)[1:]

            transforms = TransformStamped(
                header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=frame_id),
                child_frame_id=f'{frame_id}/{lidar_name}',
                transform=Transform(
                    translation=Vector3(
                        x=0,
                        y=0,
                        z=0,
                    ),
                    rotation=Quaternion(
                        x=0,
                        y=0,
                        z=0,
                        w=1,
                    ),
                ),
            )

            message = TFMessage(transforms=[transforms])

            self._rosbag_writer.write(
                connection,
                timestamp * 1e9,
                serialize_cdr(message, message.__msgtype__),
            )

    def _generate_stamped_transform_camera(
        self,
        camera_id: str,
        connection: Connection,
    ) -> None:
        lidar = self._sequence.lidar
        camera = self._sequence.camera[camera_id]

        for lidar_pose, camera_pose, timestamp in track(
            zip(lidar.poses, camera.poses, camera.timestamps),
            description=f'Generating /tf for {camera_id}...',
            total=len(lidar.poses),
        ):
            sec, nsec = split_unix_timestamp(timestamp)
            frame_id = get_frame_id_from_topic(self.EGO_NAMESPACE)[1:]

            T_camera_lidar = transform_origin_to_target(  # noqa: N806
                camera_pose,
                lidar_pose,
            )
            pose = mat_encoded_as_pose(T_camera_lidar)
            pose_norm = pose_to_normative(pose)

            transforms = TransformStamped(
                header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=frame_id),
                child_frame_id=f'{frame_id}/{camera_id}',
                transform=Transform(
                    translation=Vector3(
                        x=pose_norm['position']['x'],
                        y=pose_norm['position']['y'],
                        z=pose_norm['position']['z'],
                    ),
                    rotation=Quaternion(
                        x=pose_norm['heading']['x'],
                        y=pose_norm['heading']['y'],
                        z=pose_norm['heading']['z'],
                        w=pose_norm['heading']['w'],
                    ),
                ),
            )

            message = TFMessage(transforms=[transforms])

            self._rosbag_writer.write(
                connection,
                timestamp * 1e9,
                serialize_cdr(message, message.__msgtype__),
            )

    def _convert_tf_sensors(self) -> None:
        tf_connection: Connection = self._rosbag_writer.add_connection(
            '/tf',
            TFMessage.__msgtype__,
        )

        self._generate_stamped_transform_ego_vehicle(tf_connection)

        for camera_id in self._sequence.camera:
            self._generate_stamped_transform_camera(camera_id, tf_connection)

        for lidar_id in LidarIdentifier:
            self._generate_stamped_transform_lidar(lidar_id, tf_connection)

    def convert(self, sequence_id: str, path: Path | str = '') -> None:
        """Convert a sequence from the PandaSet to a rosbag file.

        Args:
        ----
        sequence_id (str):
            The ID of the sequence to be converted.
        path (Union[Path, str], optional):
            The save path of the rosbag file. Default is '', i.e the
            rosbag file will be saved in the current working directory
            with the name 'pandaset_{sequence_id}'.

        Returns:
        -------
        None
        """
        self._rosbag_writer = Writer(path or f'pandasetbag_{sequence_id}')
        self._rosbag_writer.set_compression(
            self.compression_mode,
            Writer.CompressionFormat.ZSTD,
        )

        self._sequence = self._dataset[sequence_id]

        with Console().status(f"Loading sequence '{sequence_id}'..."):
            self._sequence.load()

        with self._rosbag_writer:
            self._convert_cameras()
            self._convert_gps_to_topic()
            self._convert_lidars()
            self._convert_cuboids_to_topic()
            self._convert_tf_sensors()

        self._dataset.unload(sequence_id)
