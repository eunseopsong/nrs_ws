# nrs_path

## Overview

`nrs_path` is a ROS package for generating, projecting, interpolating, and visualizing toolpaths on 3D meshes. It combines C++ and Python nodes to:

- Compute geodesic shortest paths on meshes (using CGAL).
- Apply spline interpolation to waypoints.
- Project user-defined paths onto mesh surfaces (via Open3D Python scripts).
- Simulate and visualize planned trajectories for a UR10e manipulator in RViz and MoveIt.

This package is particularly suited for tasks such as robotic polishing, machining, or inspection where precise surface-following paths are required.

## Features

- **Geodesic Path Planning**: Computes true shortest-path contours on triangular meshes (CGAL Surface_mesh_shortest_path).
- **Waypoint Profiling & Interpolation**: Uniformly spaced waypoints with trapezoidal velocity profiling and spline smoothing.
- **MoveIt Simulation**: Simulate and visualize UR10e trajectories in RViz without a real robot (fake execution).
- **RViz Visualization**: Publish markers for raw and interpolated waypoints.
- **Python Utilities**: Scripts using Open3D for mesh sampling and projection, plus Matplotlib for plotting.

## ROS Nodes

| Node Name                    | Package   | Type                              | Description                                              |
|------------------------------|-----------|-----------------------------------|----------------------------------------------------------|
| `path_projection`            | nrs_path  | C++ (`path_projection.cpp`)       | Interactive projector: click points, publish `Waypoints` |
| `nrs_node_path_generation`   | nrs_path  | C++ (`nrs_node_path_generation`)  | Generate profiled & interpolated waypoint sequences     |
| `nrs_node_simulation`        | nrs_path  | C++ (`nrs_node_simulation`)       | Simulate trajectory execution in MoveIt                 |
| `nrs_node_visualization`     | nrs_path  | C++ (`nrs_node_visualization`)    | Publish RViz markers for waypoints                      |

## Messages

- **Messages**
  - `Waypoint.msg`: contains position (x,y,z), orientation (qw,qx,qy,qz), and force vector (Fx,Fy,Fz).
  - `Waypoints.msg`: array of `Waypoint`.


## Launch Files

- **path_planning.launch**
  - Starts RViz
  - Launches `nrs_node_path_generation` & `nrs_node_simulation`
  - Configures MoveIt (fake execution) for UR10e

- **moveit/**
  - Full MoveIt setup for UR10e: planners, controllers, sensor manager, etc.

## Configuration

All parameters are in `config/`:

- **Mesh & Robot**
  - `g_mesh_file_path`: default path to mesh (`mesh/workpiece.stl`) in code, or set via ROS parameter.
- **MoveIt & Controllers**
  - `*.yaml` files for kinematics, controllers, joint limits, physical parameters, ROS controllers.
- **Planning**
  - OMPL (`ompl_planning.yaml`), CHOMP (`chomp_planning.yaml`), default kinematics, visual parameters.

## Usage

1. **Build**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **Launch full pipeline**
   ```bash
   roslaunch nrs_path path_planning.launch
   ```

3. **Check topics & services**
   - Waypoints topic: `/nrs_path/waypoints`
   - Service: `/nrs_path/command`

## Dependencies

### ROS (Noetic recommended)

- **Buildtool**: `catkin`
- **Build & Exec**:
  - `roscpp`, `rospy`
  - `std_msgs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`
  - `image_transport`, `cv_bridge`, `image_geometry`
  - `pcl_ros`, `pcl_conversions`
  - `moveit_ros_planning_interface`
  - `urdf`, `xacro`
  - `message_generation`, `message_runtime`

### System Libraries

- **Eigen3**
- **CGAL** (with GMP & MPFR)
- **PCL (Point Cloud Library)**
- **OpenCV**
- **VTK** (for MoveIt planning display)
- **yaml-cpp**
- **librealsense2** (if using RealSense)

### Python Packages

- `rospy`
- `numpy`
- `open3d`
- `matplotlib`

## Installation

```bash
# ROS packages
sudo apt-get update && sudo apt-get install -y \
  ros-noetic-ros-base ros-noetic-roscpp ros-noetic-rospy \
  ros-noetic-std-msgs ros-noetic-sensor-msgs ros-noetic-geometry-msgs \
  ros-noetic-visualization-msgs \
  ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-image-geometry \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-moveit-ros-planning-interface \
  ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-tf2-ros ros-noetic-joint-state-publisher \
  ros-noetic-rviz ros-noetic-message-generation ros-noetic-message-runtime \
  ros-noetic-realsense2-camera

# System dependencies
sudo apt-get install -y \
  libeigen3-dev libcgal-dev libgmp-dev libmpfr-dev \
  libpcl-dev libopencv-dev libvtk6-dev libyaml-cpp-dev librealsense2-dev

# Python dependencies
pip3 install numpy open3d matplotlib

# Build the package
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

## File Structure

```
├── launch/
│   ├── path_planning.launch
│   ├── yonoo.launch
│   └── moveit/...
├── config/    # Planning & controller parameters
├── mesh/      # Example STL mesh files
├── msg/       # Waypoint, Waypoints
├── srv/       # Command.srv
├── src/       # C++ nodes & libraries
├── data/      # Python scripts & example path files
├── urdf/      # UR10e robot description
├── rviz/      # RViz displays
├── CMakeLists.txt
└── package.xml
```

## License

TODO: Specify license (e.g., MIT, BSD).

