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
from typing import Dict, NewType

if typing.TYPE_CHECKING:
    from typing import List, Optional, Tuple

    import numpy.typing as npt
    from pandas import DataFrame
    from pandaset.sensors import Intrinsics

import numpy as np
from pandaset.geometry import lidar_points_to_ego
from scipy.spatial.transform import Rotation as R  # noqa: N817

Pose = NewType('Pose', Dict[str, Dict[str, float]])
"""Pose dictionary. The dictionary keys return the following types:

- `position`: `dict`
    - `x`: `float`
        - Position (x-axis) in meter
    - `y`: `float`
        - Position (y-axis) in meter
    - `z`: `float`
        - Position (z-axis) in meter
- `heading`: `dict`
    - `w`: `float`
        - Real part of _Quaternion_
    - `x`: `float`
        - First imaginary part of _Quaternion_
    - `y`: `float`
        - Second imaginary part of _Quaternion_
    - `z`: `float`
        - Third imaginary part of _Quaternion_
"""

T2Norm: npt.NDArray[np.float64] = np.identity(4)
"""Homogenous transformation matrix from PandaSet to normative coordinates.

    PandaSet ego coordinates of e.g. a LiDAR are:
    - x pointing to the right
    - y pointing to the front
    - z pointing up

    Normative coordinates are:
    - x pointing to the front
    - y pointings to the left
    - z pointing up

    x → -y'\n
        [ 1, 0, 0, 0 ]^T → [ 0, -1, 0, 0 ]^T

    y →  x'\n
        [ 0, 1, 0, 0 ]^T → [ 1,  0, 0, 0 ]^T

    z →  z'\n
        [ 0, 0, 1, 0 ]^T → [ 0,  0, 1, 0 ]^T
"""

T2Norm[0, :3] = [0, 1, 0]
T2Norm[1, :3] = [-1, 0, 0]
T2Norm[2, :3] = [0, 0, 1]


def get_flatten_calibration_matrices(
    intrinsics: Intrinsics,
    imgsz: Optional[Tuple[int, int]] = None,
) -> Tuple[
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
]:
    """Get the flatten calibration matrices for an given intrinsics.

    Calibration matrices will correspond to the normative coordinate system.

    Args:
    ----
    intrinsics (Intrinsics):
        Intrinsic parameters of a camera (fx, fy, cx, cy).
    imgsz (Tuple[int], optional):
        An optional tuple of integers representing the scaled size of the
        image. If provided, the intrinsic matrix 'K' is scaled to fit the
        image size. Default is None, i.e no scaling applied to original
        image captured by the camera.

    Returns:
    -------
    Tuple[ndarray[np.float64], ...]:
        A tuple of flattened numpy arrays representing
        the distortion, intrinsic, rotation, and projection matrices
        respectively.
    """
    d = np.zeros((5,), dtype=np.float64)  # distortion

    #     [fx   0  cx]
    # K = [ 0  fy  cy]
    #     [ 0   0   1]
    K = np.identity(3, dtype=np.float64)  # intrinsic
    K[0, [0, 2]] = [intrinsics.fx, intrinsics.cx]
    K[1, [1, 2]] = [intrinsics.fy, intrinsics.cy]

    R = np.identity(3, dtype=np.float64)  # rotation

    # all cameras in the dataset have the same image size
    org_imgsz = (1080, 1920)
    org_imgsz = org_imgsz[::-1]

    # scale intrinsic matrix to fit image size
    # https://dsp.stackexchange.com/a/6057
    if imgsz is not None and all(imgsz):
        K[0] /= abs(org_imgsz[0] / imgsz[0])
        K[1] /= abs(org_imgsz[1] / imgsz[1])

    #     [fx   0  cx Tx]
    # P = [ 0  fy  cy Ty]
    #     [ 0   0   1  0]
    P = np.column_stack((K, np.zeros(3)))  # projection

    return d, K.flatten(), R.flatten(), P.flatten()


def lidar_data_frame_to_ego(
    lidar_df: DataFrame,
    lidar_pose: Pose,
) -> npt.NDArray[np.float32]:
    """Convert LiDAR points to ego coordinates.

    Args:
    ----
    lidar_df (DataFrame):
        A concatenated DataFrame representing a set of Lidar points in the
        world coordinate frame. The DataFrame should contain at least 5
        columns, with the first 3 columns representing the x, y, and z
        coordinates of each point, the fourth column representing the
        intensity of the point and the fifth column represents the
        class ID associated with pandas contains each point.

    - index: `int`
        - Ordered point cloud. When joining the raw point cloud with data
            from ``SemanticSegmentation``, it is important to keep the index
            order.
    - `x`: `float`
        - Position of point in world-coordinate system (x-axis) in meter
    - `y`: `float`
        - Position of point in world-coordinate system (y-axis) in meter
    - `z`: `float`
        - Position of point in world-coordinate system (z-axis) in meter
    - `i`: `float`
        - Reflection intensity in a range `[0, 255]`
    - `t`: `float`
        - Recorded timestamp for specific point
    - `d`: `int`
        - Sensor ID. `0` -> mechnical 360° LiDAR, `1` -> forward-facing
            LiDAR
    - `class`: `int`
        - integer code representing the type of object the point
            belongs to (e.g. Bus, Car, Ground)

    lidar_pose (Pose):
        The pose of the Lidar sensor in the world coordinate frame.

    Returns:
    -------
    npt.NDArray[np.float32]:
        A 2D array of shape `(n, 5)`, where `n` is the number of Lidar
        points in the input `frame`. The first 3 columns of the array
        represent the x, y, and z coordinates of each point, transformed
        from the sensor frame to the ego vehicle frame. The fourth column
        represents the intensity of each point, scaled to the range
        `[0, 1]`. The fifth column represents the class ID associated with
        each point.
    """
    ego_points = lidar_points_to_ego(lidar_df.to_numpy()[:, :3], lidar_pose)
    intensities = lidar_df.to_numpy()[:, 3] / 255
    class_id = lidar_df.to_numpy()[:, -1]

    ego_frame: npt.NDArray[np.float32] = np.hstack(
        [
            ego_points,
            np.expand_dims(intensities, axis=1),
            np.expand_dims(class_id, axis=1),
        ],
    ).astype(np.float32)

    return ego_frame


def lidar_frame_to_normative(frame: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
    """Convert LiDAR frame to normative.

    Given a LiDAR frame represented as a numpy ndarray of shape (N, 6) with
    dtype np.float32, where N is the number of points and 5 is (x, y, z,
    intensity, class), this function returns a new numpy ndarray with the same
    shape and dtype that has been transformed to match normative coordinates.

    PandaSet ego coordinates of e.g. a LiDAR are:
    - x pointing to the right
    - y pointing to the front
    - z pointing up

    Normative coordinates are:
    - x pointing to the front
    - y pointings to the left
    - z pointing up

    Args:
    ----
    frame (npt.NDArray[np.float32]):
        LiDAR frame in the coordinate system of the PandaSet.

    Returns:
    -------
    npt.NDArray[np.float32]:
        LiDAR frame in the normative coordinate system.
    """
    norm_frame = frame.copy()

    norm_frame[:, :3] = frame[:, [1, 0, 2]]  # switch x and y
    norm_frame[:, 1] *= -1  # revert y axis

    return norm_frame


def ego_vehicle_to_normative_ego(pose: Pose) -> Pose:
    """Convert ego vehicle (is equivalent to LiDAR) pose to normative coordinates.

    Given a LiDAR Pose object, this function returns a modified Pose object,
    position transformed to match normative coordinates. Contrary to
    `pose_to_normative` the rotation/heading of the pose will be ignored.

    Args:
    ----
    pose (Pose):
        Pose in the coordinate system of the PandaSet.

    Returns:
    -------
    Pose:
        Pose in the normative coordinate system.
    """
    M_p = pose_encoded_as_mat(pose)
    M_p[:, 3] = T2Norm @ M_p[:, 3]  # ignore heading while transforming

    return mat_encoded_as_pose(M_p)


def pose_to_normative(pose: Pose) -> Pose:
    """Convert pose to normative coordinates.

    Given a Pose object, this function returns a Pose object,
    position / translation transformed to match normative coordinates.

    Args:
    ----
    pose (Pose):
        Pose in the coordinate system of the PandaSet.

    Returns:
    -------
    Pose:
        Pose in the normative coordinate system.
    """
    M_p = pose_encoded_as_mat(pose)

    return mat_encoded_as_pose(T2Norm @ M_p)


def heading_position_to_mat(
    heading: List[float],
    position: List[float],
) -> npt.NDArray[np.float64]:
    """Get the homogenous matrix from a given rotation and translation.

    Args:
    ----
    heading (List[float]):
        A list of 4 floats representing the quaternion [w, x, y, z]
        of the rotation.
    position (List[float]):
        A list of 3 floats representing the [x, y, z] translation.

    Returns:
    -------
    npt.NDArray[np.float64]:
        A 4x4 numpy ndarray representing heading, position as matrix.
    """
    quat = np.array([*heading[1:], heading[0]])  # reorder to x, y, z, w

    r = R.from_quat(quat)

    T = np.eye(4)
    T[:3, :3] = r.as_matrix()
    T[:3, 3] = np.array(position)

    return T


def transform_origin_to_target(origin: Pose, target: Pose) -> npt.NDArray[np.float64]:
    """Calculate the transformation matrix from origin to target.

    Args:
    ----
    origin (Pose):
        A pose dictionary of the origin (sensor).
    target (Pose):
        A pose dictionary of the target (sensor).

    Returns:
    -------
    npt.NDArray[np.float64]:
            A 4x4 numpy ndarray representing the homogenous
            transformation matrix from the origin to the target.
    """
    T_lidar2world = pose_encoded_as_mat(target)
    T_camera2world = pose_encoded_as_mat(origin)

    T_camera2lidar: npt.NDArray[np.float64]
    T_camera2lidar = np.linalg.inv(T_lidar2world) @ T_camera2world

    return T_camera2lidar  # noqa: RET504


def pose_encoded_as_mat(pose: Pose) -> npt.NDArray[np.float64]:
    """Get the homogenous matrix from a given Pose object.

    Args:
    ----
    pose (Pose):
        A pose dictionary.

    Returns:
    -------
    npt.NDArray[np.float64]:
        A 4x4 numpy ndarray representing the Pose object as matrix.
    """
    heading = list(pose['heading'].values())
    position = list(pose['position'].values())

    return heading_position_to_mat(heading, position)


def mat_encoded_as_pose(T: npt.NDArray[np.float64]) -> Pose:
    """Encode a homogeneous matrix into a Pose data structure.

    Args:
    ----
    T (npt.NDArray[np.float64]):
        A 4x4 numpy ndarray representing the homogeneous matrix.

    Returns:
    -------
    Pose:
        The matrix encoded as a Pose object.
    """
    position = T[0:3, 3]
    rotation = T[0:3, 0:3]
    quaternion = R.from_matrix(np.asarray(rotation)).as_quat()
    quaternion = np.array([quaternion[-1], *quaternion[:-1]])  # reorder to w, x, y, z

    return Pose(
        {
            'position': {'x': position[0], 'y': position[1], 'z': position[2]},
            'heading': {
                'w': quaternion[0],
                'x': quaternion[1],
                'y': quaternion[2],
                'z': quaternion[3],
            },
        },
    )


def cuboid_data_frame_to_normative_ego(
    cuboid_df: DataFrame,
    lidar_pose: Pose,
) -> DataFrame:
    """Transform cuboid data from world to normalized ego coordinates.

    Args:
    ----
    cuboid_df (DataFrame):
        A DataFrame with columns representing the properties of the
        cuboids in world-coordinates.
    lidar_pose (Pose):
        A pose dictionary of the LiDAR sensor in world-coordinates.

    Returns:
    -------
    DataFrame:
        A DataFrame with columns representing the properties of the
        cuboids in normalized ego coordinates.
    """
    lidar_pose_mat = pose_encoded_as_mat(lidar_pose)

    M_cuboid_poses = [
        heading_position_to_mat(
            np.roll(
                R.from_euler('zyx', [row[3], 0, 0]).as_quat(),
                1,
            ).tolist(),  # IMPROVE: drop np.roll for performance improvements  # noqa: E501
            row[6:9],
        )
        for row in cuboid_df.itertuples()
    ]

    # transform cuboid from world to ego frame
    M_cuboid_poses_ego = np.linalg.inv(lidar_pose_mat) @ M_cuboid_poses

    cuboid_df.iloc[:, 2] = [
        R.from_matrix(M_p[:3, :3]).as_euler('zyx')[0] for M_p in M_cuboid_poses_ego
    ]  # yaw

    cuboid_df.iloc[:, 5] = [M_p[1:2, 3] for M_p in M_cuboid_poses_ego]  # position_x
    cuboid_df.iloc[:, 6] = [-M_p[:1, 3] for M_p in M_cuboid_poses_ego]  # positon_y
    cuboid_df.iloc[:, 7] = [M_p[2:3, 3] for M_p in M_cuboid_poses_ego]  # position_z
    cuboid_df.iloc[:, 8:11] = cuboid_df.iloc[:, [9, 8, 10]]  # dimensions

    return cuboid_df
