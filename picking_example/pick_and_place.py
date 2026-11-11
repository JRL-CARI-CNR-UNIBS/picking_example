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
Pick and place of the objects on the Sharework table into a basket.

Each cycle:

1. observe: point the wrist camera at the table (observation_points) and call
   scene_updater_wrist/acquire at each point (YOLO, objects only, written to
   scene_db); optionally also the fixed camera updater;
2. update the MoveIt planning scene with the objects of scene_db (boxes) and the basket;
3. pick the free object closest to the TCP in pick_region:
   close views around it (capture_view) and acquire_grasps (VGN multi-view);
4. plan to the pre-grasp of the best grasps (all at once: parallel_multigoal_planner
   picks the cheapest), approach, close the gripper, lift;
5. place: plan to the basket (several yaws), open the gripper, retreat, remove the
   object from scene_db.

Motions: parallel_multigoal_planner (path) -> compute_joint_trajectory (TOTG) ->
joint_trajectory_controller. Cartesian goals are ur10e_tool0 poses in
ur10e_base_link (the frames of the ik_solver configuration).
"""

import math
import threading
import time
from typing import List, Optional

from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion
from geometry_msgs.msg import PoseStamped
from ik_solver_msgs.msg import Configuration
from ik_solver_msgs.srv import CollisionChecking, GetBound, GetIk
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject,
                             Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np
from parallel_multigoal_planner_msgs.action import ParallelMultigoalPlan
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from scene_msgs.msg import KeyValue
from scene_msgs.srv import Acquire, AcquireGrasps, CaptureView, QueryObjects, RemoveObject
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from time_parametrization_msgs.action import ApplyTimeParametrization

from picking_example import geometry as g


class TaskError(RuntimeError):
    """A step failed: the current object is skipped."""


def _pose_msg(t: np.ndarray) -> Pose:
    q = g.matrix_to_quat(t[:3, :3])
    return Pose(position=Point(x=float(t[0, 3]), y=float(t[1, 3]), z=float(t[2, 3])),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))


def _pose_to_matrix(p: Pose) -> np.ndarray:
    return g.transform((p.position.x, p.position.y, p.position.z),
                       (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place')
        p = {k: self.declare_parameter(k, v).value for k, v in {
            # ROS interfaces
            'ik_solver_ns': 'ur_ik_solver',
            'planner_action': '/parallel_multigoal_plan',
            'time_parametrization_action': '/compute_joint_trajectory',
            'trajectory_action': '/joint_trajectory_controller/follow_joint_trajectory',
            'gripper_action': '/robotiq_gripper_controller/gripper_cmd',
            'apply_planning_scene_service': '/apply_planning_scene',
            'collision_check_service': '/collision_check',   # only to explain IK failures
            'scene_db': '/scene_db',
            'wrist_updater': '/scene_updater_wrist',
            'fixed_updater': '',                    # e.g. /scene_updater_fixed; '' = not used
            # Frames
            'fixed_frame': 'world',
            'base_frame': 'ur10e_base_link',        # ik_solver base_frame
            'flange_frame': 'ur10e_tool0',          # ik_solver tool_frame
            'tcp_frame': 'open_tip',                # between the fingertips
            'camera_frame': 'wrist_color_optical_frame',
            # Robot
            'group_name': 'ur_robot',
            'home': [1.5707, -2.4434, 2.4434, -1.5707, 1.5707, 0.0],
            'gripper_joint': 'ur10e_robotiq_85_left_knuckle_joint',
            'gripper_open': 0.0,
            'gripper_closed': 0.7929,
            'gripper_timeout': 5.0,
            'touch_links': [''],
            'planning_time': 5.0,
            'velocity_scaling': 0.5,
            'acceleration_scaling': 0.5,
            'approach_velocity_scaling': 0.1,       # approach, lift, retreat
            # Perception
            # camera x, y, z, looked-at point x, y, z; ... (oblique: YOLO wants upright views)
            'observation_points': [-0.12, -0.27, 1.45, 0.30, -0.27, 1.04,
                                   -0.12, 0.45, 1.45, 0.30, 0.15, 1.04],
            'observation_rolls_deg': [0.0, -20.0, 20.0],                 # alternatives
            'grasp_view_rolls_deg': [0.0, 90.0, 180.0, 270.0],           # alternatives
            'grasp_view_distance': 0.40,            # [m] camera - object center
            'grasp_views_deg': [0.0, 0.0, 35.0, 90.0, 35.0, 270.0],      # tilt, azimuth, ...
            'acquire_timeout': 0.0,                 # 0 = updater default
            'clear_scene_on_start': True,
            # Picking
            'pick_region': [0.10, 0.50, -0.70, 0.42, 0.95, 1.40],  # xmin..zmax, fixed frame
            'support_z': 1.015,                     # rail top
            # Keep-out box on the work surface during observation and grasp views,
            # removed before the pre-grasp. xmin, xmax, ymin, ymax (fixed frame).
            'search_box_region': [0.10, 0.50, -0.70, 0.42],
            'search_box_height': 0.15,              # [m] above support_z; 0 = no box
            'max_grasp_candidates': 10,
            'min_grasp_score': 0.0,
            'max_grasp_width': 0.085,
            'min_approach_down': 0.5,               # approach . (-z) >= this (60 deg from vertical)
            'grasp_depth_offset': 0.0,              # [m] added to Grasp.depth
            'min_tip_height': 0.01,                 # [m] TCP above support_z
            'pregrasp_distance': 0.10,
            'lift_height': 0.15,
            'max_attempts_per_object': 2,
            'max_cycles': 20,
            # Basket (worlds/picking.sdf)
            'basket_center': [0.30, 0.62, 1.015],   # center of the bottom
            'basket_size': [0.30, 0.32, 0.13],      # outer x, y, wall height
            'basket_wall': 0.01,
            'place_clearance': 0.05,                # [m] object bottom above the basket rim
            'place_yaws_deg': [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
            'retreat_height': 0.10,
            'object_box_margin': 0.01,              # [m] MoveIt boxes of the objects
            'scene_settle_time': 0.5,               # [s] wait after a planning scene change
        }.items()}
        self._p = p
        if p['touch_links'] == ['']:
            p['touch_links'] = [
                'ur10e_tool0', 'ur10e_wrist_3_link', 'robotiq_coupler', 'wrist_link',
                'wrist_camera_support',
                'ur10e_robotiq_85_base_link',
                'ur10e_robotiq_85_left_knuckle_link', 'ur10e_robotiq_85_right_knuckle_link',
                'ur10e_robotiq_85_left_inner_knuckle_link',
                'ur10e_robotiq_85_right_inner_knuckle_link',
                'ur10e_robotiq_85_left_finger_link', 'ur10e_robotiq_85_right_finger_link',
                'ur10e_robotiq_85_left_finger_tip_link', 'ur10e_robotiq_85_right_finger_tip_link']

        cb = ReentrantCallbackGroup()
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._joints = {}
        self._joints_lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10,
                                 callback_group=cb)

        db = p['scene_db'].rstrip('/')
        wrist = p['wrist_updater'].rstrip('/')
        self._srv = {
            'bounds': self.create_client(GetBound, f"/{p['ik_solver_ns']}/get_bounds",
                                         callback_group=cb),
            'ik': self.create_client(GetIk, f"/{p['ik_solver_ns']}/get_ik", callback_group=cb),
            'collision': self.create_client(CollisionChecking, p['collision_check_service'],
                                            callback_group=cb),
            'scene': self.create_client(ApplyPlanningScene, p['apply_planning_scene_service'],
                                        callback_group=cb),
            'query': self.create_client(QueryObjects, f'{db}/query_objects', callback_group=cb),
            'remove': self.create_client(RemoveObject, f'{db}/remove_object', callback_group=cb),
            'clear': self.create_client(Trigger, f'{db}/clear', callback_group=cb),
            'acquire': self.create_client(Acquire, f'{wrist}/acquire', callback_group=cb),
            'capture': self.create_client(CaptureView, f'{wrist}/capture_view',
                                          callback_group=cb),
            'grasps': self.create_client(AcquireGrasps, f'{wrist}/acquire_grasps',
                                         callback_group=cb),
        }
        if p['fixed_updater']:
            self._srv['acquire_fixed'] = self.create_client(
                Acquire, f"{p['fixed_updater'].rstrip('/')}/acquire", callback_group=cb)
        self._act = {
            'plan': ActionClient(self, ParallelMultigoalPlan, p['planner_action'],
                                 callback_group=cb),
            'time': ActionClient(self, ApplyTimeParametrization,
                                 p['time_parametrization_action'], callback_group=cb),
            'traj': ActionClient(self, FollowJointTrajectory, p['trajectory_action'],
                                 callback_group=cb),
            'gripper': ActionClient(self, ParallelGripperCommand, p['gripper_action'],
                                    callback_group=cb),
        }
        self._joint_names: List[str] = []
        self._scene_ids = set()        # object boxes in the MoveIt scene
        self._search_box = False       # search_box in the MoveIt scene
        self._attempts = {}            # object id -> failed attempts
        self.log = self.get_logger()

    # =================================================================== helpers
    def _on_joints(self, msg: JointState):
        with self._joints_lock:
            self._joints.update(zip(msg.name, msg.position))

    def _wait(self, future, timeout: float, what: str):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise TaskError(f'{what}: timeout ({timeout:.0f} s)')
        if future.exception() is not None:
            raise TaskError(f'{what}: {future.exception()}')
        return future.result()

    def _call(self, name: str, request, timeout: float = 60.0, check: bool = True):
        client = self._srv[name]
        if not client.wait_for_service(timeout_sec=5.0):
            raise TaskError(f'Service {client.srv_name} not available')
        res = self._wait(client.call_async(request), timeout, client.srv_name)
        if check and hasattr(res, 'success') and not res.success:
            raise TaskError(f'{client.srv_name}: {res.message}')
        return res

    def _action(self, name: str, goal, timeout: float, feedback=None):
        client = self._act[name]
        if not client.wait_for_server(timeout_sec=5.0):
            raise TaskError(f'Action {client._action_name} not available')
        handle = self._wait(client.send_goal_async(goal, feedback_callback=feedback), 10.0,
                            f'{client._action_name} goal')
        if not handle.accepted:
            raise TaskError(f'{client._action_name}: goal rejected')
        result = self._wait(handle.get_result_async(), timeout, client._action_name)
        return result.status, result.result

    def _lookup(self, target: str, source: str) -> np.ndarray:
        try:
            t = self._tf.lookup_transform(target, source, Time(), Duration(seconds=2.0))
        except TransformException as e:
            raise TaskError(f'TF {target} -> {source}: {e}')
        tr, q = t.transform.translation, t.transform.rotation
        return g.transform((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))

    def _current_joints(self) -> List[float]:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with self._joints_lock:
                if all(n in self._joints for n in self._joint_names):
                    return [self._joints[n] for n in self._joint_names]
            time.sleep(0.05)
        raise TaskError('No joint state for the arm joints')

    # =================================================================== motion
    def _plan(self, goals: List[Constraints], what: str):
        goal = ParallelMultigoalPlan.Goal()
        req = goal.motion_plan_request
        req.group_name = self._p['group_name']
        req.allowed_planning_time = float(self._p['planning_time'])
        req.start_state.joint_state.name = list(self._joint_names)
        req.start_state.joint_state.position = self._current_joints()
        req.goal_constraints = goals
        status, result = self._action('plan', goal, self._p['planning_time'] + 60.0)
        code = result.motion_plan_response.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise TaskError(f'Planning {what} failed (error code {code}, {len(goals)} goals)')
        return result.motion_plan_response

    def _execute(self, plan, velocity_scaling: float, what: str):
        goal = ApplyTimeParametrization.Goal()
        goal.unparametrized_motion_plan_response = plan
        goal.max_velocity_scaling_factor = float(velocity_scaling)
        goal.max_acceleration_scaling_factor = float(self._p['acceleration_scaling'])
        _, result = self._action('time', goal, 30.0)
        timed = result.motion_plan_response
        if timed.error_code.val != MoveItErrorCodes.SUCCESS:
            raise TaskError(f'Time parametrization of {what} failed '
                            f'(error code {timed.error_code.val})')
        traj = timed.trajectory.joint_trajectory
        traj.header.stamp = Time().to_msg()     # start now
        duration = Duration.from_msg(traj.points[-1].time_from_start).nanoseconds * 1e-9
        fjt = FollowJointTrajectory.Goal()
        fjt.trajectory = traj
        _, res = self._action('traj', fjt, duration + 30.0)
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise TaskError(f'Execution of {what} failed: {res.error_code} {res.error_string}')

    def _cartesian_goal(self, t_world_flange: np.ndarray) -> Constraints:
        """Goal for ur10e_tool0 at a pose in the fixed frame (sent in the ik_solver base frame)."""
        t = self._t_base_world @ t_world_flange
        pose = _pose_msg(t)
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self._p['base_frame']
        pc.link_name = self._p['flange_frame']
        pc.constraint_region = BoundingVolume(
            primitive_poses=[Pose(position=pose.position, orientation=Quaternion(w=1.0))])
        oc = OrientationConstraint(header=pc.header, link_name=pc.link_name,
                                   orientation=pose.orientation)
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        return c

    def _joint_goal(self, positions) -> Constraints:
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=n, position=float(v), weight=1.0)
                               for n, v in zip(self._joint_names, positions)]
        return c

    def _move_home(self):
        home = self._p['home']
        if max(abs(a - b) for a, b in zip(self._current_joints(), home)) < 1e-2:
            return
        self._move([self._joint_goal(home)], 'home')

    def _explain_no_ik(self, goal: Constraints):
        """Log why a Cartesian goal has no valid IK solution (bounds or colliding links)."""
        if not goal.position_constraints:
            return
        req = GetIk.Request()
        req.target.pose = PoseStamped()
        req.target.pose.header.frame_id = self._p['base_frame']
        req.target.pose.pose.position = goal.position_constraints[0].constraint_region \
            .primitive_poses[0].position
        req.target.pose.pose.orientation = goal.orientation_constraints[0].orientation
        req.target.seeds = [Configuration(configuration=self._current_joints())]
        req.seed_joint_names = list(self._joint_names)
        try:
            sol = self._call('ik', req, 20.0, check=False)
            if not sol.solution.configurations:
                self.log.warn('  no IK solution at all: pose out of reach')
                return
            res = self._call('collision', CollisionChecking.Request(
                solutions=sol.solution.configurations, joint_names=sol.joint_names,
                detailed=True), 20.0)
        except TaskError as e:
            self.log.warn(f'  cannot explain: {e}')
            return
        pairs = {}
        for r in res.result:
            if r.out_of_bound:
                key = 'out of bounds: ' + ', '.join(r.out_of_bound_links)
            elif r.feasibility < 1.0:
                key = ', '.join(sorted({f'{a}-{b}' for a, b in zip(r.colliding_obj_first,
                                                                  r.colliding_obj_second)}))
            else:
                key = 'feasible'
            pairs[key] = pairs.get(key, 0) + 1
        self.log.warn(f'  IK solutions of the first goal: {pairs}')

    def _move(self, goals: List[Constraints], what: str, slow: bool = False):
        self.log.info(f'Moving: {what} ({len(goals)} alternative goals)')
        try:
            plan = self._plan(goals, what)
        except TaskError:
            self._explain_no_ik(goals[0])
            raise
        self._execute(plan, self._p['approach_velocity_scaling'] if slow
                      else self._p['velocity_scaling'], what)

    def _move_tcp(self, tcp_goals: List[np.ndarray], what: str, slow: bool = False):
        goals = [self._cartesian_goal(t @ self._t_tcp_flange) for t in tcp_goals]
        self._move(goals, what, slow)

    def _move_camera(self, camera_goals: List[np.ndarray], what: str):
        goals = [self._cartesian_goal(t @ self._t_camera_flange) for t in camera_goals]
        self._move(goals, what)

    def _translate_tcp(self, offset, what: str):
        """Small motion of the TCP keeping its orientation (approach, lift, retreat)."""
        t = self._lookup(self._p['fixed_frame'], self._p['tcp_frame'])
        t[:3, 3] += np.asarray(offset, dtype=float)
        self._move_tcp([t], what, slow=True)

    def _gripper(self, position: float, what: str) -> bool:
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [self._p['gripper_joint']]
        goal.command.position = [float(position)]
        try:
            status, res = self._action('gripper', goal, self._p['gripper_timeout'])
        except TaskError as e:
            self.log.warn(f'Gripper {what}: {e}')
            return False
        self.log.info(f'Gripper {what}: reached_goal={res.reached_goal} stalled={res.stalled}')
        return True

    # ============================================================ planning scene
    def _remove_msg(self, oid: str) -> CollisionObject:
        co = CollisionObject(id=oid, operation=CollisionObject.REMOVE)
        co.header.frame_id = self._p['fixed_frame']
        return co

    def _apply_scene(self, scene: PlanningScene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        req = ApplyPlanningScene.Request(scene=scene)
        if not self._call('scene', req, 10.0, check=False).success:
            raise TaskError('apply_planning_scene failed')
        # collision_checker_server reads the scene of move_group at 10 Hz.
        time.sleep(self._p['scene_settle_time'])

    def _box(self, oid: str, center, yaw: float, size) -> CollisionObject:
        co = CollisionObject()
        co.header.frame_id = self._p['fixed_frame']
        co.id = oid
        co.operation = CollisionObject.ADD
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                        dimensions=[float(s) for s in size])]
        co.primitive_poses = [_pose_msg(g.make_transform(g.rot_z(yaw), center))]
        return co

    def _object_box(self, obj, oid: Optional[str] = None,
                    bottom_gap: float = 0.005) -> CollisionObject:
        """Box of a scene_db object, extended down to the support (views see only the top)."""
        m = self._p['object_box_margin']
        top = obj.pose.position.z + 0.5 * obj.size.z
        bottom = min(self._p['support_z'] + bottom_gap, top - 0.01)
        q = obj.pose.orientation
        yaw = g.yaw_of(g.quat_to_matrix((q.x, q.y, q.z, q.w)))
        return self._box(oid or f'scene_db/{obj.id}',
                         (obj.pose.position.x, obj.pose.position.y, 0.5 * (top + bottom)), yaw,
                         (obj.size.x + 2 * m, obj.size.y + 2 * m, top - bottom))

    def _add_basket(self):
        cx, cy, cz = self._p['basket_center']
        sx, sy, h = self._p['basket_size']
        w = self._p['basket_wall']
        co = CollisionObject()
        co.header.frame_id = self._p['fixed_frame']
        co.id = 'basket'
        co.operation = CollisionObject.ADD
        parts = [((0, 0, w / 2), (sx, sy, w)),
                 ((sx / 2 - w / 2, 0, h / 2), (w, sy, h)),
                 ((-sx / 2 + w / 2, 0, h / 2), (w, sy, h)),
                 ((0, sy / 2 - w / 2, h / 2), (sx - 2 * w, w, h)),
                 ((0, -sy / 2 + w / 2, h / 2), (sx - 2 * w, w, h))]
        for (dx, dy, dz), size in parts:
            co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
            co.primitive_poses.append(_pose_msg(g.make_transform(
                np.eye(3), (cx + dx, cy + dy, cz + dz))))
        scene = PlanningScene()
        scene.world.collision_objects = [co]
        self._apply_scene(scene)

    def _set_search_box(self, present: bool):
        """Keep-out box over the objects, so that the camera moves do not hit them."""
        p = self._p
        h = float(p['search_box_height'])
        if h <= 0.0 or present == self._search_box:
            return
        scene = PlanningScene()
        if present:
            x0, x1, y0, y1 = p['search_box_region']
            scene.world.collision_objects = [self._box(
                'search_box', (0.5 * (x0 + x1), 0.5 * (y0 + y1), p['support_z'] + 0.5 * h),
                0.0, (x1 - x0, y1 - y0, h))]
        else:
            scene.world.collision_objects = [self._remove_msg('search_box')]
        self._apply_scene(scene)
        self._search_box = present

    def _sync_scene(self, objects, exclude=()):
        """MoveIt boxes = objects of scene_db (except the ids in exclude)."""
        boxes = [self._object_box(o) for o in objects if o.id not in exclude]
        ids = {b.id for b in boxes}
        scene = PlanningScene()
        for oid in self._scene_ids - ids:
            scene.world.collision_objects.append(self._remove_msg(oid))
        scene.world.collision_objects += boxes
        self._apply_scene(scene)
        self._scene_ids = ids

    def _attach(self, obj):
        aco = AttachedCollisionObject()
        aco.link_name = self._p['flange_frame']
        aco.object = self._object_box(obj, 'held_object', bottom_gap=0.02)
        aco.touch_links = list(self._p['touch_links'])
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [aco]
        self._apply_scene(scene)

    def _detach(self):
        aco = AttachedCollisionObject()
        aco.link_name = self._p['flange_frame']
        aco.object = self._remove_msg('held_object')
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [aco]
        # A detached object goes back to the world: remove it there too.
        scene.world.collision_objects = [self._remove_msg('held_object')]
        self._apply_scene(scene)

    # =============================================================== perception
    def _query(self, **filters):
        req = QueryObjects.Request(filters=[KeyValue(key=k, value=str(v))
                                            for k, v in filters.items()])
        return self._call('query', req, 10.0).objects

    def _observe(self):
        p = self._p
        timeout = float(p['acquire_timeout'])
        if 'acquire_fixed' in self._srv:
            res = self._call('acquire_fixed', Acquire.Request(timeout=timeout), 120.0)
            self.log.info(f'Fixed camera: {res.num_detections} detections, added {res.added_ids}')
        pts = p['observation_points']
        for k, i in enumerate(range(0, len(pts) - 5, 6)):
            cam, target = pts[i:i + 3], pts[i + 3:i + 6]
            goals = [g.look_at(cam, target, math.radians(r)) for r in p['observation_rolls_deg']]
            try:
                self._move_camera(goals, f'observation point {k + 1}')
            except TaskError as e:
                self.log.warn(f'{e}: point skipped')
                continue
            res = self._call('acquire', Acquire.Request(timeout=timeout), 120.0)
            self.log.info(f'Observation {k + 1}: {res.num_detections} detections, '
                          f'added {list(res.added_ids)}, moved {list(res.moved_ids)}')

    def _acquire_grasps(self, obj):
        p = self._p
        center = (obj.pose.position.x, obj.pose.position.y, obj.pose.position.z)
        views = 0
        for k, cam in enumerate(g.view_positions(center, p['grasp_view_distance'],
                                                 p['grasp_views_deg'])):
            goals = [g.look_at(cam, center, math.radians(r)) for r in p['grasp_view_rolls_deg']]
            try:
                self._move_camera(goals, f'grasp view {k + 1} of {obj.id}')
            except TaskError as e:
                self.log.warn(f'{e}: view skipped')
                continue
            res = self._call('capture', CaptureView.Request(clear=(views == 0)), 60.0)
            views = res.num_views
        if views == 0:
            raise TaskError(f'No view of {obj.id} for the grasp detection')
        res = self._call('grasps', AcquireGrasps.Request(object_id=obj.id), 180.0)
        self.log.info(f'{obj.id}: {res.num_grasps} grasps detected in {res.num_views} views, '
                      f'{res.num_grasps_assigned} assigned to the object')
        if res.num_grasps_assigned == 0:
            raise TaskError(f'No grasp for {obj.id}')

    # ==================================================================== tasks
    def _grasp_candidates(self, obj):
        """TCP poses (grasp, pre-grasp) of the usable grasps, best first."""
        p = self._p
        out = []
        for sg in obj.grasps:
            gr = sg.grasp
            if gr.score < p['min_grasp_score'] or gr.width > p['max_grasp_width']:
                continue
            pos = (gr.pose.position.x, gr.pose.position.y, gr.pose.position.z)
            q = (gr.pose.orientation.x, gr.pose.orientation.y, gr.pose.orientation.z,
                 gr.pose.orientation.w)
            approach = g.quat_to_matrix(q)[:, 0]
            if -approach[2] < p['min_approach_down']:
                continue          # from below or from the side: not for a ceiling robot
            for flip in (False, True):
                tcp = g.grasp_to_tcp(pos, q, gr.depth, p['grasp_depth_offset'], flip)
                low = p['support_z'] + p['min_tip_height'] - tcp[2, 3]
                if low > 0.0:     # fingertips in the table: back off along the approach
                    tcp[:3, 3] -= approach * (low / max(-approach[2], 1e-3))
                pre = tcp.copy()
                pre[:3, 3] -= p['pregrasp_distance'] * approach
                out.append((gr.score, tcp, pre))
            if len(out) >= 2 * p['max_grasp_candidates']:
                break
        return out

    @staticmethod
    def _same_object(obj, objects):
        """
        Ids of obj and of the objects whose center is inside its footprint.

        YOLO may report one object with two classes (a mug as cup and as bowl):
        scene_db keeps both. They are excluded and removed together.
        """
        r = max(0.5 * max(obj.size.x, obj.size.y), 0.03)
        return {o.id for o in objects
                if math.hypot(o.pose.position.x - obj.pose.position.x,
                              o.pose.position.y - obj.pose.position.y) <= r} | {obj.id}

    def _pick(self, obj):
        p = self._p
        self._acquire_grasps(obj)
        obj = self._query(id=obj.id)[0]
        candidates = self._grasp_candidates(obj)
        if not candidates:
            raise TaskError(f'No usable grasp for {obj.id} ({len(obj.grasps)} stored)')
        self.log.info(f'{obj.id}: {len(candidates)} grasp candidates, best score '
                      f'{candidates[0][0]:.2f}')

        self._gripper(p['gripper_open'], 'open')
        objects = self._query()
        same = self._same_object(obj, objects)
        if len(same) > 1:
            self.log.info(f'{obj.id}: same object as {sorted(same - {obj.id})}')
        self._set_search_box(False)                   # the gripper goes down to the object
        self._sync_scene(objects, exclude=same)       # the fingers may touch it
        self._move_tcp([c[2] for c in candidates], f'pre-grasp of {obj.id}')

        # The planner reached one of the pre-grasps: find it from the TCP pose.
        tcp_now = self._lookup(p['fixed_frame'], p['tcp_frame'])
        best = min(candidates, key=lambda c: g.pose_distance(c[2], tcp_now))
        approach = tcp_now[:3, 2]
        depth = float(np.dot(best[1][:3, 3] - tcp_now[:3, 3], approach))

        self._translate_tcp(depth * approach, f'approach to {obj.id}')
        tcp_grasp = self._lookup(p['fixed_frame'], p['tcp_frame'])
        self._gripper(p['gripper_closed'], 'close')          # stops on the object
        self._attach(obj)
        self._translate_tcp((0.0, 0.0, p['lift_height']), f'lift of {obj.id}')
        return obj, tcp_grasp, same

    def _place(self, obj, tcp_grasp: np.ndarray):
        p = self._p
        # Object frame: box center, bottom on the support, box yaw.
        q = obj.pose.orientation
        yaw = g.yaw_of(g.quat_to_matrix((q.x, q.y, q.z, q.w)))
        t_obj = g.make_transform(g.rot_z(yaw), (obj.pose.position.x, obj.pose.position.y,
                                                p['support_z']))
        t_obj_tcp = g.inverse(t_obj) @ tcp_grasp
        cx, cy, cz = p['basket_center']
        z = cz + p['basket_size'][2] + p['place_clearance']
        goals = []
        for a in p['place_yaws_deg']:
            t_place = g.make_transform(g.rot_z(yaw + math.radians(a)), (cx, cy, z))
            goals.append(t_place @ t_obj_tcp)
        self._move_tcp(goals, f'place of {obj.id}')
        self._gripper(p['gripper_open'], 'open')
        self._detach()
        self._translate_tcp((0.0, 0.0, p['retreat_height']), 'retreat')

    def _choose(self):
        p = self._p
        r = p['pick_region']
        tcp = self._lookup(p['fixed_frame'], p['tcp_frame'])[:3, 3]
        objs = self._query(region=','.join(str(v) for v in r), state='ok', free_only='true',
                           include_grasps='false',
                           near=f'{tcp[0]},{tcp[1]},{tcp[2]}')
        objs = [o for o in objs if self._attempts.get(o.id, 0) < p['max_attempts_per_object']]
        return objs[0] if objs else None

    def _recover(self, held: bool):
        """After a failure: release what is held, back to home."""
        if held:
            self._gripper(self._p['gripper_open'], 'open')
            try:
                self._detach()
            except TaskError:
                pass
        self._set_search_box(False)      # the TCP may be inside it
        try:
            self._translate_tcp((0.0, 0.0, self._p['retreat_height']), 'retreat')
        except TaskError as e:
            self.log.warn(f'Retreat: {e}')
        # Out of the search box before the next observation adds it again.
        try:
            self._move_home()
        except TaskError as e:
            self.log.warn(f'Home: {e}')

    def run(self):
        p = self._p
        res = self._call('bounds', GetBound.Request(), 30.0)
        self._joint_names = [b.joint_name for b in res.boundaries]
        self.log.info(f'Arm joints: {self._joint_names}')
        self._t_base_world = self._lookup(p['base_frame'], p['fixed_frame'])
        self._t_tcp_flange = self._lookup(p['tcp_frame'], p['flange_frame'])
        self._t_camera_flange = self._lookup(p['camera_frame'], p['flange_frame'])
        if p['clear_scene_on_start']:
            self._call('clear', Trigger.Request(), 10.0)
        self._add_basket()
        self._gripper(p['gripper_open'], 'open')
        self._move_home()

        placed = []
        for cycle in range(p['max_cycles']):
            self.log.info(f'===== Cycle {cycle + 1}: observing the table =====')
            self._set_search_box(True)
            self._observe()
            self._sync_scene(self._query())
            obj = self._choose()
            if obj is None:
                self.log.info('No object left to pick')
                break
            self.log.info(f'Picking {obj.id} ({obj.class_name}, {obj.color}) at '
                          f'({obj.pose.position.x:.3f}, {obj.pose.position.y:.3f}, '
                          f'{obj.pose.position.z:.3f})')
            held = False
            try:
                obj, tcp_grasp, same = self._pick(obj)
                held = True
                self._place(obj, tcp_grasp)
                held = False
                for oid in same:
                    self._call('remove', RemoveObject.Request(object_id=oid), 10.0, check=False)
                placed.append(obj.id)
                self.log.info(f'{obj.id} is in the basket')
            except TaskError as e:
                self._attempts[obj.id] = self._attempts.get(obj.id, 0) + 1
                self.log.error(f'{obj.id}: {e} (attempt {self._attempts[obj.id]})')
                self._recover(held)
                if held:      # dropped somewhere: the next observation finds it again
                    self._call('remove', RemoveObject.Request(object_id=obj.id), 10.0,
                               check=False)
        self._move_home()
        self.log.info(f'Done: placed {placed}, failed {[k for k in self._attempts]}')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
    except TaskError as e:
        node.get_logger().fatal(str(e))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
