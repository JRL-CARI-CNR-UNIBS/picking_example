# Copyright 2026 Manuel Beschi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pose math for the pick and place example (numpy only, no ROS)."""

import math
from typing import List, Sequence, Tuple

import numpy as np

# grasp_detection_msgs/Grasp: x = approach, y = closing direction.
# Robotiq 2F-85 on ur10e_tool0 / open_tip: z = approach, x = closing (the
# fingers move along x). Columns of R_GRASP_TCP: the TCP axes in the grasp frame.
R_GRASP_TCP = np.array([[0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    """Rotation matrix of a quaternion (x, y, z, w)."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r: np.ndarray) -> Tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) of a rotation matrix."""
    t = np.trace(r)
    if t > 0.0:
        s = 0.5 / math.sqrt(t + 1.0)
        w, x = 0.25 / s, (r[2, 1] - r[1, 2]) * s
        y, z = (r[0, 2] - r[2, 0]) * s, (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w, x = (r[2, 1] - r[1, 2]) / s, 0.25 * s
        y, z = (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w, x = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s
        y, z = 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w, x = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s
        y, z = (r[1, 2] + r[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    return tuple(float(v) for v in q)


def transform(position: Sequence[float], quaternion: Sequence[float]) -> np.ndarray:
    """4x4 homogeneous transform from a position and a quaternion (x, y, z, w)."""
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(quaternion)
    t[:3, 3] = position
    return t


def make_transform(rotation: np.ndarray, position: Sequence[float]) -> np.ndarray:
    """4x4 homogeneous transform from a rotation matrix and a position."""
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = position
    return t


def inverse(t: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 rigid transform."""
    inv = np.eye(4)
    inv[:3, :3] = t[:3, :3].T
    inv[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return inv


def rot_x(a: float) -> np.ndarray:
    """Rotation matrix about x."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_z(a: float) -> np.ndarray:
    """Rotation matrix about z."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def yaw_of(r: np.ndarray) -> float:
    """Rotation about z of a rotation matrix whose z axis is (close to) vertical."""
    return math.atan2(r[1, 0], r[0, 0])


def look_at(camera_position: Sequence[float], target: Sequence[float],
            roll: float = 0.0) -> np.ndarray:
    """
    Pose of a camera optical frame (z forward) at camera_position looking at target.

    x is horizontal and y points downwards when possible (the image is upright);
    roll rotates the frame about its z axis.
    """
    p = np.asarray(camera_position, dtype=float)
    z = np.asarray(target, dtype=float) - p
    z /= np.linalg.norm(z)
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(z @ ref)) > 0.99:     # looking straight up or down
        ref = np.array([1.0, 0.0, 0.0])
    x = np.cross(z, ref)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    r = np.column_stack([x, y, z]) @ rot_z(roll)
    return make_transform(r, p)


def view_positions(target: Sequence[float], distance: float,
                   views_deg: Sequence[float]) -> List[np.ndarray]:
    """
    Camera positions around target.

    views_deg is a flat list (tilt, azimuth, tilt, azimuth, ...) in degrees:
    tilt 0 = right above the target, azimuth measured about the world z axis.
    """
    t = np.asarray(target, dtype=float)
    out = []
    for i in range(0, len(views_deg) - 1, 2):
        tilt, az = math.radians(views_deg[i]), math.radians(views_deg[i + 1])
        d = np.array([math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az),
                      math.cos(tilt)])
        out.append(t + distance * d)
    return out


def grasp_to_tcp(position: Sequence[float], quaternion: Sequence[float], depth: float,
                 depth_offset: float = 0.0, flip: bool = False) -> np.ndarray:
    """
    TCP pose (open_tip) for a grasp_detection_msgs/Grasp in the same frame.

    The grasp position is the grasp center; the fingertips go depth (+ depth_offset)
    beyond it along the approach. flip turns the gripper by 180 deg about the
    approach (the parallel gripper is symmetric).
    """
    r_grasp = quat_to_matrix(quaternion)
    approach = r_grasp[:, 0]
    r_tcp = r_grasp @ R_GRASP_TCP
    if flip:
        r_tcp = r_tcp @ rot_z(math.pi)
    p = np.asarray(position, dtype=float) + (depth + depth_offset) * approach
    return make_transform(r_tcp, p)


def pose_distance(a: np.ndarray, b: np.ndarray, rotation_weight: float = 0.2) -> float:
    """Position distance plus rotation_weight * rotation angle between two transforms."""
    dp = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    c = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1.0) / 2.0
    return dp + rotation_weight * math.acos(max(-1.0, min(1.0, c)))
