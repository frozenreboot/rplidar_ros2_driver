^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rplidar_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.1 (2026-07-18)
------------------
* Fixed the Rolling build by passing the node by reference to
  ``tf2_ros::StaticTransformBroadcaster`` (NodeInterfaces API migration).
* Contributors: JWJ | frozenreboot

1.4.0 (2026-07-17)
------------------
* Raised the minimum required CMake version to 3.16 and verified builds on
  ROS 2 Jazzy and Lyrical (`#38 <https://github.com/frozenreboot/rplidar_driver/issues/38>`_).
* Refactored the node entry point to use the rclcpp_components template macro
  and moved the node into the ``rplidar_node`` namespace (`#36 <https://github.com/frozenreboot/rplidar_driver/issues/36>`_).
* Cleaned up package dependencies and linter configuration, fixing test
  dependency failures on the ROS build farm (`#35 <https://github.com/frozenreboot/rplidar_driver/issues/35>`_).
* Fixed Rolling build compatibility (`#33 <https://github.com/frozenreboot/rplidar_driver/issues/33>`_).
* Added a license audit documenting the vendored Slamtec SDK provenance for
  the first rosdistro release (`#32 <https://github.com/frozenreboot/rplidar_driver/issues/32>`_).
* Converted the changelog to REP-132 format for catkin release tools (`#31 <https://github.com/frozenreboot/rplidar_driver/issues/31>`_).
* Renamed the package to ``rplidar_driver`` following REP-144 naming
  guidelines (`#29 <https://github.com/frozenreboot/rplidar_driver/issues/29>`_).
* Added GTest-based test infrastructure, including mock driver tests and
  lifecycle/publication tests (`#20 <https://github.com/frozenreboot/rplidar_driver/issues/20>`_).
* Clarified that the ``angle_offset`` parameter is specified in radians and
  removed the corresponding launch argument in favor of YAML parameters
  (`#23 <https://github.com/frozenreboot/rplidar_driver/issues/23>`_).
* Fixed QoS parameter inconsistency by initializing the QoS parameter and
  applying the configured policy (`#15 <https://github.com/frozenreboot/rplidar_driver/issues/15>`_).
* Added governance documentation and an AI disclosure section for OSRF
  compliance; added Błażej Sowa as a maintainer (`#34 <https://github.com/frozenreboot/rplidar_driver/issues/34>`_).
* Contributors: Błażej Sowa, JWJ | frozenreboot, cosmicog, wj

v1.3.0 (2026-01-18)
-------------------

* Added scan interpolation via the `interpolated_rays` parameter to generate high-density, mathematical line-segment-based measurements.
* Added `publish_point_cloud` and `intensities_as_angles` parameters for debugging and analysis.
* Added configurable ROS 2 QoS profiles via the `qos_policy` parameter.
* Deprecated `scan_processing` in favor of the explicit `interpolated_rays` parameter.
* Removed the `inverted` parameter. Use TF or `angle_offset` instead.
* Set the default `scan_mode` to `Standard` to prevent ghost points on S-series devices.
* Fixed segmentation faults in `dummy_mode` caused by unsafe dynamic casting.
* Fixed an index out-of-bounds crash in the interpolation loop.
* Fixed `LifecycleNode` activation issues where publishers remained silent after state transition.
* Fixed a `TypeError` in the composition launch file.
* Contributors: cosmicog, frozenreboot

v1.2.0 (2026-01-09)
-------------------

* Updated copyright years to 2026.
* Fixed scan data mirroring issue where left and right were inverted.
* Fixed launch arguments overriding parameter file settings.
* Contributors: frozenreboot

v1.1.0 (2026-01-06)
-------------------

* Refactored `RPlidarNode` into a `rclcpp_component` to enable component composition and improve IPC performance.
* Added diagnostics for frequency and connection status via `diagnostic_updater`.
* Updated scan data output to use `Inf` for out-of-range measurements according to REP-117.
* Contributors: frozenreboot

v1.0.1 (2026-01-03)
-------------------

* Fixed diagnostic messages when the driver is in a disconnected state.
* Contributors: frozenreboot
