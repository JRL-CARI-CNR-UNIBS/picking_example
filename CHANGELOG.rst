^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package picking_example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-11-17)
------------------
* Real Sharework cell (ROS 2 Humble), without Gazebo:
  ``sharework_cembre_bringup`` replaces ``sim.launch.py``.
* ``services_real.launch.py``: model of ``sharework_cembre_description`` /
  ``sharework_cembre_moveit_config``, no simulated time, detector started
  separately (``detector:=false``), ``scene_db`` nodes with the system scipy
  (``PYTHONNOUSERSITE=1``).
* ``config/real``: ``pick_and_place`` (``scaled_fjt_controller``,
  ``robotiq_action_controller``, low speeds, one cycle; geometry of the Gazebo
  world, to be measured), ``detector``, ``vgn``.
* ``pick_and_place``: ``gripper_action_type`` (``parallel`` =
  ``control_msgs/ParallelGripperCommand``, ``gripper_command`` =
  ``control_msgs/GripperCommand``) and ``gripper_max_effort``.
* README: real cell section. LICENSE and CONTRIBUTING.

0.1.0 (2026-11-03)
------------------
* First version: pick and place in the Sharework cell, in Gazebo.
* ``sim.launch.py``: the cell of ``sharework_gazebo`` (``description_file``
  argument) with ``urdf/sharework_picking.urdf.xacro`` (wrist RealSense
  ``wrist_*`` with the calibration of the real cell, one DetachableJoint per
  object), ``worlds/picking.sdf`` (two mugs and a bowl from Gazebo Fuel on the
  rail, a basket), the wrist camera bridge, ``sim_grasp_manager`` and RViz
  (``rviz/picking.rviz``).
* ``sim_grasp_manager``: attaches the object between the fingers when the
  gripper closes, detaches it when it opens (knuckle position with
  hysteresis, ground-truth poses from Gazebo).
* ``services.launch.py``: ``yolo_rgbd_detector`` (``yolo11s-seg``),
  ``scene_db``, ``scene_updater`` on the wrist camera (optionally a fixed one),
  ``ur_ik_solver``, ``collision_checker_server``,
  ``parallel_multigoal_planner`` (MI-RRT*), ``time_parametrization`` (TOTG).
* ``pick_and_place`` node: oblique observations, MoveIt scene from
  ``scene_db``, VGN multi-view grasps from close views, multi-goal pick of the
  best grasps, place in the basket with alternative yaws; duplicate detections
  of one object handled together; logs why a Cartesian goal has no IK.
* Unit tests of the pose math (``picking_example/geometry.py``).
* ``docs/API.md``, ``docs/TUTORIAL.md`` with images.
