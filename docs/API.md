# picking_example: API

Nodes, parameters, ROS and Gazebo interfaces, frames and conventions of the
example. How to run it: [TUTORIAL.md](TUTORIAL.md).

## Nodes

| Executable | Launched by | Role |
|---|---|---|
| `pick_and_place` | `pick_and_place.launch.py` | The task: observe, update the scene, grasps, pick, place |
| `sim_grasp_manager` | `sim.launch.py` | Holds the object between the fingers in Gazebo |

Nodes of other packages started by the launch files:

| Node | Package | Launch file |
|---|---|---|
| Gazebo, `move_group`, controllers, `robot_state_publisher`, camera bridges (wrist included) | `sharework_gazebo` | `sim.launch.py` |
| `rviz2` (`rviz/picking.rviz`) | `rviz2` | `sim.launch.py` |
| `yolo_rgbd_detector` | `yolo_rgbd_detector` | `services.launch.py` |
| `scene_db`, `scene_updater_wrist`, `scene_updater_fixed` | `scene_db` | `services.launch.py` |
| `ur_ik_solver` | `ik_solver` / `ur_ik_solver` | `services.launch.py` |
| `ik_collision_check` | `collision_check_server` | `services.launch.py` |
| `parallel_multigoal_planner` | `parallel_multigoal_planner` | `services.launch.py` |
| `apply_time_parametrization` | `time_parametrization` | `services.launch.py` |
| `vgn_grasp_server` | `vgn_grasp_server` (robostack) | its own launch file, `config/vgn.yaml` |

---

## pick_and_place

### Interfaces used

| Kind | Name (default) | Type | Use |
|---|---|---|---|
| sub | `/joint_states` | `sensor_msgs/JointState` | Current configuration (planning start) |
| TF | `fixed_frame` ← `flange_frame`, `tcp_frame`, `camera_frame` | | Goals are given as TCP or camera poses |
| srv | `/<ik_solver_ns>/get_bounds` | `ik_solver_msgs/GetBound` | Joint limits (diagnostics) |
| srv | `/<ik_solver_ns>/get_ik` | `ik_solver_msgs/GetIk` | IK (diagnostics) |
| srv | `collision_check_service` | `ik_solver_msgs/CollisionChecking` | Which links collide (diagnostics) |
| srv | `apply_planning_scene_service` | `moveit_msgs/ApplyPlanningScene` | Object boxes, basket, attached object |
| srv | `<scene_db>/query_objects` | `scene_msgs/QueryObjects` | Objects and their grasps |
| srv | `<scene_db>/remove_object` | `scene_msgs/RemoveObject` | Placed objects (and their duplicates) |
| srv | `<scene_db>/clear` | `std_srvs/Trigger` | At start (`clear_scene_on_start`) |
| srv | `<wrist_updater>/acquire` | `scene_msgs/Acquire` | YOLO on the current wrist view, update `scene_db` |
| srv | `<wrist_updater>/capture_view` | `scene_msgs/CaptureView` | Store a close view for VGN |
| srv | `<wrist_updater>/acquire_grasps` | `scene_msgs/AcquireGrasps` | VGN multi-view on one object |
| srv | `<fixed_updater>/acquire` | `scene_msgs/Acquire` | Optional fixed camera |
| action | `planner_action` | `parallel_multigoal_planner_msgs/ParallelMultigoalPlan` | Path to the best of several goals |
| action | `time_parametrization_action` | `time_parametrization_msgs/ApplyTimeParametrization` | TOTG |
| action | `trajectory_action` | `control_msgs/FollowJointTrajectory` | Execution |
| action | `gripper_action` | `control_msgs/ParallelGripperCommand` | Open / close |

### Parameters

Defaults in `picking_example/pick_and_place.py`; the launch file loads
`config/pick_and_place.yaml`. Positions in `fixed_frame`, meters, angles in degrees
where the name ends with `_deg`.

**Interfaces**

| Parameter | Default | |
|---|---|---|
| `ik_solver_ns` | `ur_ik_solver` | Namespace of the IK services (= planner IK) |
| `planner_action` | `/parallel_multigoal_plan` | |
| `time_parametrization_action` | `/apply_time_parametrization` | |
| `trajectory_action` | `/joint_trajectory_controller/follow_joint_trajectory` | |
| `gripper_action` | `/robotiq_gripper_controller/gripper_cmd` | |
| `apply_planning_scene_service` | `/apply_planning_scene` | |
| `collision_check_service` | `/collision_check` | Only to explain IK failures |
| `scene_db` | `/scene_db` | |
| `wrist_updater` | `/scene_updater_wrist` | |
| `fixed_updater` | `''` | `/scene_updater_fixed` to use a fixed camera too |

**Frames and robot**

| Parameter | Default | |
|---|---|---|
| `fixed_frame` | `world` | Frame of `scene_db` and of the poses below |
| `base_frame` | `ur10e_base_link` | = IK `base_frame` (Cartesian goals are expressed here) |
| `flange_frame` | `ur10e_tool0` | = IK `tool_frame` |
| `tcp_frame` | `open_tip` | Between the open fingertips |
| `camera_frame` | `wrist_color_optical_frame` | |
| `group_name` | `ur_robot` | |
| `home` | `[1.5707, -2.4434, 2.4434, -1.5707, 1.5707, 0]` | SRDF `home` |
| `gripper_joint`, `gripper_open`, `gripper_closed` | knuckle, `0.0`, `0.7929` | |
| `gripper_timeout` | `5.0` | [s] |
| `touch_links` | gripper and wrist links | Links allowed to touch the held object |
| `planning_time` | `5.0` | [s] per request |
| `velocity_scaling`, `acceleration_scaling` | `0.5`, `0.5` | Free motions |
| `approach_velocity_scaling` | `0.1` | Approach, lift, retreat |

**Perception**

| Parameter | Default | |
|---|---|---|
| `observation_points` | 2 points | Camera x, y, z and looked-at x, y, z, 6 values per point |
| `observation_rolls_deg` | `[0, -20, 20]` | Rolls about the optical axis, alternative goals |
| `grasp_view_distance` | `0.40` | Camera - object center |
| `grasp_views_deg` | `[0,0, 35,90, 35,270]` | (tilt from the vertical, azimuth) pairs of the close views |
| `grasp_view_rolls_deg` | `[0, 90, 180, 270]` | Alternatives (VGN uses depth only) |
| `acquire_timeout` | `0.0` | 0 = updater default |
| `clear_scene_on_start` | `true` | |

**Pick**

| Parameter | Default | |
|---|---|---|
| `pick_region` | `[0.10, 0.50, -0.70, 0.42, 0.95, 1.40]` | xmin, xmax, ymin, ymax, zmin, zmax of the box centers to pick |
| `support_z` | `1.015` | Rail top: object boxes extend down to it |
| `search_box_region` | `[0.10, 0.50, -0.70, 0.42]` | xmin, xmax, ymin, ymax of a keep-out box on the work surface, present during observation and grasp views, removed before the pre-grasp |
| `search_box_height` | `0.15` | Height of the keep-out box above `support_z`; `0` = no box |
| `max_grasp_candidates` | `10` | Best grasps sent to the planner (each also flipped by 180°) |
| `min_grasp_score` | `0.0` | |
| `max_grasp_width` | `0.085` | Robotiq 2F-85 |
| `min_approach_down` | `0.5` | Approach · (−z) ≥ this: at most 60° from the vertical |
| `grasp_depth_offset` | `0.0` | Added to `Grasp.depth` |
| `min_tip_height` | `0.01` | TCP at least this above `support_z` |
| `pregrasp_distance` | `0.10` | Along the approach |
| `lift_height` | `0.15` | |
| `max_attempts_per_object` | `2` | |
| `max_cycles` | `20` | |
| `object_box_margin` | `0.01` | MoveIt boxes of the objects |
| `scene_settle_time` | `0.5` | [s] after a planning scene change (`collision_checker_server` reads it at 10 Hz) |

**Place**

| Parameter | Default | |
|---|---|---|
| `basket_center` | `[0.30, 0.62, 1.015]` | Center of the bottom (as in `worlds/picking.sdf`) |
| `basket_size` | `[0.30, 0.32, 0.13]` | Outer x, y, wall height |
| `basket_wall` | `0.01` | |
| `place_clearance` | `0.05` | Object bottom above the rim |
| `place_yaws_deg` | every 45° | Alternative goals above the basket |
| `retreat_height` | `0.10` | |

### Log

| Message | Meaning |
|---|---|
| `Observation k: n detections, added [...], moved [...]` | Result of `acquire` at observation point k |
| `<id>: n grasps detected in m views, ...` | Result of `acquire_grasps` |
| `Picking <id> (<class>, <color>) at (...)` | Chosen object |
| `<id>: same object as [...]` | Duplicate detections removed with it |
| `<id> is in the basket` | Place done, object removed from `scene_db` |
| `no IK solution at all: pose out of reach`, `IK solutions of the first goal: {...}` | Why a Cartesian goal failed: count of IK solutions per cause (`out of bounds: <links>`, colliding pairs `a-b`, `feasible`) |
| `Done: placed [...], failed [...]` | End |

---

## sim_grasp_manager

Gazebo does not hold objects reliably with finger contacts. The robot
description has a `DetachableJoint` from `ur10e_wrist_3_link` to each object
of `config/objects.yaml`; this node drives them.

- At startup it detaches all of them (Gazebo attaches them when the plugin loads)
  and logs `All the objects are free`.
- Knuckle above `close_threshold` with no held object: the object whose origin is
  within `attach_radius` horizontally from `tcp_frame` (and between
  `-attach_below` and `+attach_above` below it) is attached (`Holding <name>`).
  If there is none: `Gripper closing with no object between the fingers`.
- Knuckle below `open_threshold`: the held object is detached (`Released <name>`).
  The hysteresis ignores the bounces of the fingers on the object.

| Parameter | Default | |
|---|---|---|
| `objects_file` | `config/objects.yaml` | |
| `world` | `sharework_cell` | Gazebo world name |
| `gripper_joint` | `ur10e_robotiq_85_left_knuckle_joint` | |
| `tcp_frame`, `fixed_frame` | `open_tip`, `world` | |
| `close_threshold` | `0.10` | [rad] attach above |
| `open_threshold` | `0.04` | [rad] detach below |
| `attach_radius` | `0.10` | [m] |
| `attach_below`, `attach_above` | `0.03`, `0.25` | [m] TCP height relative to the object origin |
| `startup_detach_period`, `startup_detach_timeout` | `1.0`, `120.0` | [s] |

| Kind | Name | Type |
|---|---|---|
| sub | `/joint_states` | `sensor_msgs/JointState` |
| pub | `~/held_object` | `std_msgs/String` (`''` = none) |
| gz pub | `/picking/<object>/attach`, `/picking/<object>/detach` | `gz.msgs.Empty` |
| gz sub | `/picking/<object>/state` | `gz.msgs.StringMsg` (`attached` / `detached`) |
| gz sub | `/world/<world>/pose/info` | `gz.msgs.Pose_V` |

---

## Robot description and world

`urdf/sharework_picking.urdf.xacro` (passed to `sharework_gazebo` with
`description_file`, and to MoveIt, collision checker and RViz):

- includes `sharework_cell_gz.urdf.xacro` of `sharework_gazebo`, which has the
  wrist RealSense: `wrist_link` on `ur10e_tool0` (xyz `0.00648373 -0.0742079 0.0741869`,
  quaternion `0.271469 -0.27528 0.644284 0.659866`), `wrist_color_optical_frame`;
  Gazebo RGB-D camera `wrist` (848×480, hfov 69°, 0.1–3 m), bridged to
  `/wrist/color/image_raw`, `/wrist/color/camera_info`,
  `/wrist/aligned_depth_to_color/image_raw`, `/wrist/depth/color/points`;
- one `DetachableJoint` per object of `config/objects.yaml` (args `sim_cameras`).

`worlds/picking.sdf`: the Sharework world plus

| Model | Fuel model | Pose |
|---|---|---|
| `mug_blue` | Cole_Hardware_Mug_Classic_Blue | 0.27, −0.42, 1.02 |
| `mug_white` | Threshold_Porcelain_Coffee_Mug_All_Over_Bead_White | 0.33, −0.12, 1.02 |
| `bowl` | Threshold_Bead_Cereal_Bowl_White | 0.30, 0.18, 1.02 |
| `basket` | static, in the SDF | bottom center 0.30, 0.62, 1.015 |

## Conventions

- **Grasps** (`grasp_detection_msgs/Grasp`, in `fixed_frame`): x = approach,
  y = closing direction, position = grasp center, `depth` = how far the
  fingertips go beyond it.
- **TCP** (`open_tip`, and `ur10e_tool0`): z = approach, x = closing direction
  (the fingers move along x). `geometry.grasp_to_tcp` converts:
  `R_tcp = R_grasp · [[0,0,1],[1,0,0],[0,1,0]]`, `p_tcp = p + (depth + offset) · approach`;
  the 180° flip about z is a second candidate.
- **Cartesian goals** of the planner: `ur10e_tool0` poses in `ur10e_base_link`,
  computed as `T_base_fixed · T_fixed_tcp · T_tcp_flange` with TF.
- **Camera views**: `geometry.look_at` gives an optical frame (z forward) with
  image x horizontal and y down, so the images are upright.
- **Same object**: an object whose box center is inside the footprint of the
  picked object is the same object detected twice; it is excluded from the
  MoveIt scene while grasping and removed from `scene_db` with it.

## Configuration files

| File | Used by |
|---|---|
| `config/pick_and_place.yaml` | `pick_and_place` |
| `config/objects.yaml` | xacro (DetachableJoints), `sim_grasp_manager` |
| `config/scene.yaml` | `scene_db`, `scene_updater_wrist`, `scene_updater_fixed` |
| `config/detector.yaml` | `yolo_rgbd_detector` |
| `config/vgn.yaml` | `vgn_grasp_server` |
| `config/ik_solver.yaml` | `ur_ik_solver` (cnr_param) |
| `config/mirrt_star.yaml` | `parallel_multigoal_planner` (cnr_param) |
| `config/collision_checker.yaml` | `ik_collision_check` |
| `config/time_parametrization.yaml` | `apply_time_parametrization` |
| `rviz/picking.rviz` | RViz |
