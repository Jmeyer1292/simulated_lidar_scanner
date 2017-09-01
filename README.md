# Simulated Lidar Scanner

![Simulated Lidar Scanner][1]

## Description

The Simulated Lidar Scanner package is a ROS implementation of the [Synthetic Lidar Scanner package][2] created by [daviddoria][3].
The package parses an input URDF for all static geometry, and then performs raycasting using VTK libraries to simulate the output of a LiDAR sensor,
Time-of-flight (TOF) camera, or 3D depth sensor.

## Dependencies

This package depends on VTK 6.2.

## Usage

### Simulated Lidar Scanner Node

**Parameters**

*Scanner-specific parameters* (to be saved in a .yaml file)
+ `scanner/scan_frequency` (double; hz)
  - **Required:** update frequency of the scanner
+ `scanner/theta_span` (double; degrees)
  - **Required:** horizontal field of view of the scanner, from -360 to +360
+ `scanner/phi_span` (double; degrees)
  - **Required:** vertical field of view of the scanner, from -180 to +180
+ `scanner/theta_points` (int)
  - **Required:** number of rays to be cast in the horizontal direction
+ `scanner/phi_points` (int)
  - **Required:** number of rays to be cast in the vertical direction
+ `scanner/los_variance` (double; meters; default: 0.0)
  - Scanner noise in the direction of the cast ray
+ `scanner/orthogonal_variance` (double; meters; default: 0.0)
  - Scanner noise orthogonal to the direction of the cast ray
+ `scanner/max_incidence_angle` (double; degrees; default: 0.0)
  - The maximum angle of incidence at which the sensor can pick up a return signal
+ `scanner/max_distance` (double; meters; default: 0.0)
  - The maximum sensor operational distance

*Use-case specific parameters*
+ `params_file` (string)
  - **Required:** file location of the scanner-specific parameters .yaml file
+ `fixed_frame` (string)
  - **Required:** root, fixed frame of the URDF
+ `scanner_frame` (string)
  - **Required:** frame to which the scanner is attached
+ `scanner_name` (string)
  - **Required:** user-defined name of the scanner, used to disambiguate scanner publisher topic names
+ `tf_filter_distance` (double; meters; default: 0.0)
  - Distance by which the scanner frame must move relative to its previous position in order to trigger a new ray-casting operation
+ `tf_filter_angle` (double; degrees; default: 0.0)
  - Angle, by which the scanner frame must move relative to its previous orientation in order to trigger a new ray-casting operation

**Published Topics**
+ `sensor_data/[scanner_name]` (sensor_msgs/PointCloud2)
  - The topic on which the output point cloud is published


### Scanner Relocator Node

This package also contains a node which allows the position and orientation of one or multiple simulated lidar scanners to be manipulated using interactive markers in Rviz.

**Parameters**
+ `scanner_parent_frame` (string)
  - **Required:** frame in which to initially place simulated lidar scanner(s) before manipulation
+ `scanner_frames` (list of strings)
  - **Required:** list of frames corresponding to the `scanner_frame` parameter in each simulated lidar scanner to be manipulated

## Example

This demo creates two simulated lidar scanners (based on the parameters in the `test/test_params.yaml` file) which can be manipulated to observe the object in the URDF.

`roslaunch simulated_lidar_scanner lidar_test.launch`

Currently the interactive markers do not appear when the demo is initiallly launched. To see and interact with them, kill and restart the `scanner_relocator` node:

`rosnode kill /scanner_relocator`

`rosrun simulated_lidar_scanner scanner_relocator`


[1]: simulated_lidar_scanner.gif
[2]: https://github.com/daviddoria/SyntheticLidarScanner
[3]: https://github.com/daviddoria
