# Copyright 2026 Manuel Beschi
# SPDX-License-Identifier: Apache-2.0
"""Unit tests of picking_example.geometry (no ROS)."""

import math

import numpy as np
from picking_example import geometry as g


def test_quaternion_roundtrip():
    r = g.rot_z(0.7) @ g.rot_x(-1.1)
    assert np.allclose(g.quat_to_matrix(g.matrix_to_quat(r)), r)


def test_look_at_is_upright():
    t = g.look_at((0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    z = t[:3, 2]
    assert np.allclose(z, np.array([1.0, 0.0, -1.0]) / math.sqrt(2.0))
    assert t[2, 1] < 0.0          # image y (down) points down in the world
    assert abs(t[2, 0]) < 1e-9    # image x horizontal


def test_look_at_straight_down():
    t = g.look_at((0.3, 0.2, 1.5), (0.3, 0.2, 1.0))
    assert np.allclose(t[:3, 2], [0.0, 0.0, -1.0])
    assert np.allclose(t[:3, :3] @ t[:3, :3].T, np.eye(3))


def test_grasp_to_tcp_axes():
    # Top-down grasp: approach -z, closing along y.
    r = np.column_stack([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    q = g.matrix_to_quat(r)
    t = g.grasp_to_tcp((0.1, 0.2, 1.1), q, depth=0.02)
    assert np.allclose(t[:3, 2], [0.0, 0.0, -1.0])     # TCP z = approach
    assert np.allclose(t[:3, 0], [0.0, 1.0, 0.0])      # TCP x = closing direction
    assert np.allclose(t[:3, 3], [0.1, 0.2, 1.08])     # fingertips depth beyond the center
    flipped = g.grasp_to_tcp((0.1, 0.2, 1.1), q, depth=0.02, flip=True)
    assert np.allclose(flipped[:3, 2], t[:3, 2])
    assert np.allclose(flipped[:3, 0], -t[:3, 0])


def test_view_positions():
    views = g.view_positions((0.0, 0.0, 1.0), 0.4, [0.0, 0.0, 90.0, 90.0])
    assert np.allclose(views[0], [0.0, 0.0, 1.4])
    assert np.allclose(views[1], [0.0, 0.4, 1.0])


def test_inverse():
    t = g.make_transform(g.rot_z(0.3), (1.0, 2.0, 3.0))
    assert np.allclose(g.inverse(t) @ t, np.eye(4))
