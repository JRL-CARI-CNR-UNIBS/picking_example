^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package picking_example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
