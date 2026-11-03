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

"""
Grasp in Gazebo: holds an object while the gripper is closed on it.

Grasping with the finger contacts alone is not reliable in Gazebo, so the
robot description has one DetachableJoint per object (see
urdf/sharework_picking.urdf.xacro). This node:

- detaches every object at startup (Gazebo attaches the joints when it loads them);
- when the gripper starts closing (knuckle position above close_threshold) and
  an object is between the fingers (ground truth pose from Gazebo, near the TCP),
  attaches that object;
- when the gripper is open again (below open_threshold), detaches it.
  The fingers bounce on the held object: the position alone, with a hysteresis,
  is used instead of the direction of motion.

The pick and place client only uses the gripper action, as on the real cell.
It talks to Gazebo with gz-transport (topics /picking/<object>/attach|detach|state
and /world/<world>/pose/info), not through ros_gz_bridge.
"""

import math
import threading
import time

from ament_index_python.packages import get_package_share_directory
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class SimGraspManager(Node):

    def __init__(self):
        super().__init__('sim_grasp_manager')
        default_objects = f"{get_package_share_directory('picking_example')}/config/objects.yaml"
        p = {k: self.declare_parameter(k, v).value for k, v in {
            'objects_file': default_objects,
            'world': 'sharework_cell',
            'gripper_joint': 'ur10e_robotiq_85_left_knuckle_joint',
            'tcp_frame': 'open_tip',          # between the fingertips
            'fixed_frame': 'world',
            'close_threshold': 0.10,          # [rad] knuckle above: closing, attach
            'open_threshold': 0.04,           # [rad] knuckle below: open, detach
            'attach_radius': 0.10,            # [m] horizontal TCP - object origin distance
            'attach_below': 0.03,             # [m] TCP at most this below the object origin
            'attach_above': 0.25,             # [m] TCP at most this above the object origin
            'startup_detach_period': 1.0,     # [s]
            'startup_detach_timeout': 120.0,  # [s] stop trying after this
        }.items()}
        self._p = p
        with open(p['objects_file']) as f:
            self._objects = [o['name'] for o in yaml.safe_load(f)['objects']]

        self._lock = threading.Lock()
        self._poses = {}                      # model name -> (x, y, z), world
        self._state = {n: 'unknown' for n in self._objects}   # from the plugins
        self._held = None
        self._closed_empty = False            # closed with nothing to hold: wait for opening
        self._start = time.monotonic()

        self._gz = GzNode()
        self._attach = {n: self._gz.advertise(f'/picking/{n}/attach', Empty) for n in self._objects}
        self._detach = {n: self._gz.advertise(f'/picking/{n}/detach', Empty) for n in self._objects}
        for n in self._objects:
            self._gz.subscribe(StringMsg, f'/picking/{n}/state',
                               lambda msg, name=n: self._on_state(name, msg))
        self._gz.subscribe(Pose_V, f"/world/{p['world']}/pose/info", self._on_poses)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._held_pub = self.create_publisher(String, '~/held_object', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self._startup_timer = self.create_timer(p['startup_detach_period'], self._startup_detach)
        self.get_logger().info(f'Managing the grasp of {self._objects}')

    # ----------------------------------------------------------- Gazebo side
    def _on_state(self, name, msg):
        with self._lock:
            self._state[name] = msg.data
        self.get_logger().info(f'{name}: {msg.data}')

    def _on_poses(self, msg):
        poses = {}
        for pose in msg.pose:
            if pose.name in self._state:      # top-level models: pose in the world
                poses[pose.name] = (pose.position.x, pose.position.y, pose.position.z)
        with self._lock:
            self._poses.update(poses)

    def _startup_detach(self):
        """Detach the objects that Gazebo attached when it loaded the plugins."""
        with self._lock:
            pending = [n for n, s in self._state.items() if s != 'detached' and n != self._held]
        if not pending:
            self.get_logger().info('All the objects are free')
            self._startup_timer.cancel()
            return
        if time.monotonic() - self._start > self._p['startup_detach_timeout']:
            self.get_logger().warn(f'No "detached" state from {pending}: giving up')
            self._startup_timer.cancel()
            return
        for n in pending:
            self._detach[n].publish(Empty())

    # ------------------------------------------------------------- ROS side
    def _tcp(self):
        try:
            t = self._tf.lookup_transform(self._p['fixed_frame'], self._p['tcp_frame'], Time(),
                                          Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'TCP pose: {e}', throttle_duration_sec=5.0)
            return None
        v = t.transform.translation
        return (v.x, v.y, v.z)

    def _object_between_fingers(self):
        tcp = self._tcp()
        if tcp is None:
            return None
        best, best_d = None, float('inf')
        with self._lock:
            poses = dict(self._poses)
        for name, (x, y, z) in poses.items():
            d = math.hypot(tcp[0] - x, tcp[1] - y)
            dz = tcp[2] - z
            if (d <= self._p['attach_radius'] and -self._p['attach_below'] <= dz
                    <= self._p['attach_above'] and d < best_d):
                best, best_d = name, d
        return best

    def _on_joints(self, msg):
        try:
            pos = msg.position[msg.name.index(self._p['gripper_joint'])]
        except ValueError:
            return
        if self._held is None:
            if pos < self._p['open_threshold']:
                self._closed_empty = False
                return
            if pos < self._p['close_threshold']:
                return
            if self._closed_empty:
                return
            name = self._object_between_fingers()
            if name is None:
                self.get_logger().info('Gripper closing with no object between the fingers')
                self._closed_empty = True
                return
            self._attach[name].publish(Empty())
            self._held = name
            self._held_pub.publish(String(data=name))
            self.get_logger().info(f'Holding {name}')
        elif pos < self._p['open_threshold']:
            self._detach[self._held].publish(Empty())
            self.get_logger().info(f'Released {self._held}')
            self._held = None
            self._held_pub.publish(String(data=''))


def main(args=None):
    rclpy.init(args=args)
    node = SimGraspManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
