# nrs\_path2 (ROS 2 Humble Package)

## Overview

`nrs_path2` is a ROS 2 (Humble) package developed for tool path generation, geodesic path computation, interpolation, and marker visualization on 3D mesh surfaces. It is specifically designed for robotic applications such as polishing, inspection, or machining that require surface-following trajectories.

---

## Key Features

* **Geodesic Path Planning**: Computes shortest paths over triangular meshes using CGAL.
* **Spline Interpolation**: Applies Hermite spline interpolation to create smooth, dense waypoint paths.
* **Waypoint Projection**: Projects user-defined waypoints (via RViz or terminal input) onto mesh surfaces.
* **ROS 2 Native Nodes**: Implemented entirely in C++ with ROS 2 node architecture and services.
* **RViz Visualization**: Publishes marker arrays and point markers for path and waypoint visualization.

---

## ROS 2 Nodes

| Node Name                  | Executable                 | Description                                                                               |
| -------------------------- | -------------------------- | ----------------------------------------------------------------------------------------- |
| `nrs_node_path_generation` | `nrs_node_path_generation` | Main node that handles path planning, projection, interpolation, and service registration |
| `nrs_node_visualization`   | `nrs_node_visualization`   | Optional visualization node to display markers in RViz                                    |

---

## Topics

* `/clicked_point` (geometry\_msgs/msg/PointStamped):

  * Input topic to specify user-selected waypoints.
* `/geodesic_path` (nrs\_path2/msg/Waypoints):

  * Geodesic (straight) path result topic.
* `/interpolated_waypoints` (nrs\_path2/msg/Waypoints):

  * Smoothed spline-interpolated path output.

---

## Services

| Service Name   | Type                 | Description                        |
| -------------- | -------------------- | ---------------------------------- |
| `/spline`      | `std_srvs/srv/Empty` | Generates Hermite spline path      |
| `/straight`    | `std_srvs/srv/Empty` | Generates geodesic (shortest) path |
| `/interpolate` | `std_srvs/srv/Empty` | Applies interpolation & smoothing  |
| `/delete`      | `std_srvs/srv/Empty` | Clears existing paths and markers  |

---

## Installation (Ubuntu 22.04, ROS 2 Humble)

### ROS Dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-visualization-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-pluginlib \
  ros-humble-rosidl-default-generators
```

### System Libraries

```bash
sudo apt install -y \
  libeigen3-dev libcgal-dev libgmp-dev libmpfr-dev \
  libpcl-dev libopencv-dev libvtk-dev libyaml-cpp-dev \
  librealsense2-dkms librealsense2-utils librealsense2-dev
```

---

## Build Instructions

```bash
cd ~/nrs_ws
colcon build --packages-select nrs_path2
source install/setup.bash
```

---

## Usage

### 1. Launch the main path planning node

```bash
ros2 launch nrs_path2 path_planning.launch.py
```

### 2. Publish a clicked point from terminal (example node)

Use `waypoint_publisher.cpp` to simulate `/clicked_point` input:

```bash
ros2 run nrs_waypoint waypoint_publisher
```

### 3. Trigger path generation

```bash
ros2 service call /straight std_srvs/srv/Empty "{}"
ros2 service call /spline std_srvs/srv/Empty "{}"
ros2 service call /interpolate std_srvs/srv/Empty "{}"
```

---

## File Structure

```
nrs_path2/
├── launch/                 # ROS 2 launch files
│   └── path_planning.launch.py
├── rviz/                   # RViz config
│   └── rviz_config.rviz
├── mesh/                   # Mesh files (e.g., STL)
├── data/                   # Output: waypoint logs
├── src/                    # C++ sources
├── include/                # C++ headers
├── msg/                    # ROS 2 custom messages
│   ├── Waypoint.msg
│   └── Waypoints.msg
├── srv/                    # ROS 2 services
│   └── Command.srv
├── CMakeLists.txt
└── package.xml
```

---

## Authors & Maintainers

* Maintained by: \[송은섭], SKKU Intelligent Robotics Lab

---

## License

TBD (Specify MIT, BSD, etc.)
