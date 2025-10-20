# cuRobo Robot Setup

An RViz2 plugin for configuring robots for NVIDIA cuRobo motion planning. This tool provides an interactive GUI for loading URDF files, visualizing robots, editing collision spheres, and generating cuRobo-compatible configuration files.

## Overview

cuRobo Robot Setup streamlines the process of preparing robot configurations for use with NVIDIA's cuRobo motion planning framework. The plugin integrates directly into RViz2 and provides:

- **URDF Loading**: Load and visualize robot URDF files
- **Collision Sphere Editor**: Interactive visual editor for defining collision spheres on robot links
- **Joint Configuration**: Configure joint limits, weights, and motion parameters
- **YAML Export**: Generate cuRobo-compatible configuration files

## Features

### 1. URDF Visualization
- Load URDF files from disk
- Automatic publication to `robot_description` topic
- Real-time robot model visualization in RViz2

### 2. Collision Sphere Management
- Add/remove collision spheres per link
- Adjust sphere radius and position interactively
- Visual feedback with sphere markers in RViz2
- Hierarchical link tree view

### 3. Configuration Generation
- Configure joint names and limits
- Set retract configurations
- Define null space and C-space distance weights
- Configure self-collision ignore pairs and buffers
- Export complete cuRobo YAML configuration

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- RViz2
- Qt5
- yaml-cpp library

### Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    qtbase5-dev \
    libqt5widgets5 \
    libyaml-cpp-dev
```

### Build from Source

1. Clone this package into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url> curobo_robot_setup
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select curobo_robot_setup
```

3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Quick Start

1. Launch RViz2:
```bash
rviz2
```

2. Add the cuRobo Robot Setup panel:
   - Click **Panels** → **Add New Panel**
   - Select **curobo_robot_setup/CuRoboSetupPanel**
   - The panel will appear in the RViz2 interface

3. Load a URDF file:
   - Navigate to the **URDF Loader** tab
   - Click **Load URDF**
   - Select your robot's URDF file

4. Configure collision spheres:
   - Switch to the **Sphere Editor** tab
   - Select links from the tree view
   - Add/edit spheres for each link

5. Configure joints and export:
   - Go to the **Configuration** tab
   - Click **Configure Joints** to set parameters
   - Click **Save YAML** to export the configuration

### Panel Tabs

#### Tab 1: URDF Loader
- **Load URDF**: Browse and load a URDF file
- Status indicator shows loading success/failure
- URDF is automatically published to `robot_description` topic

#### Tab 2: Sphere Editor
- **Link Tree**: Hierarchical view of robot links
- **Sphere List**: Shows all spheres for the selected link
- **Add Sphere**: Create a new collision sphere
- **Delete Sphere**: Remove the selected sphere
- **Sphere Properties**:
  - Position (X, Y, Z) relative to link frame
  - Radius in meters

#### Tab 3: Configuration
- **Base Link**: Root link of the kinematic chain
- **End Effector Link**: Tool/gripper link
- **Configure Joints**: Open dialog to set joint parameters
- **Self-Collision Settings**: Configure collision ignore pairs
- **Save YAML**: Export cuRobo configuration
- **Load YAML**: Import existing configuration

## Configuration File Format

The exported YAML file follows the cuRobo configuration schema:

```yaml
robot_cfg:
  kinematics:
    urdf_path: "/path/to/robot.urdf"
    base_link: "base_link"
    ee_link: "end_effector"

    collision_link_names: ['link1', 'link2', 'link3']
    collision_spheres:
      link1:
        - center: [0.0, 0.0, 0.1]
          radius: 0.05
      link2:
        - center: [0.1, 0.0, 0.0]
          radius: 0.08

    collision_sphere_buffer: 0.01

    self_collision_ignore:
      base_link: ["link1"]
      link1: ["link2"]

    self_collision_buffer:
      link1: 0.0
      link2: 0.0

    cspace:
      joint_names: ['joint1', 'joint2', 'joint3']
      retract_config: [0.0, 0.0, 0.0]
      null_space_weight: [1.0, 1.0, 1.0]
      cspace_distance_weight: [1.0, 1.0, 1.0]
      max_jerk: 500.0
      max_acceleration: 15.0
```

## Topics

### Published Topics

- `/robot_description` (std_msgs/String): Robot URDF description
- `/visualization_marker_array` (visualization_msgs/MarkerArray): Collision sphere markers

## Example

See the [example](example/) directory for a sample robot configuration:
- [m1013.urdf](example/m1013.urdf): Example URDF file
- [m1013.yml](example/m1013.yml): Example cuRobo configuration

## Troubleshooting

### Plugin doesn't appear in RViz2
- Ensure the package is built: `colcon build --packages-select curobo_robot_setup`
- Source the workspace: `source ~/ros2_ws/install/setup.bash`
- Check plugin registration: `ros2 pkg list | grep curobo_robot_setup`

### URDF fails to load
- Verify the URDF file is valid: `check_urdf <file.urdf>`
- Check file permissions
- Ensure all mesh files referenced in URDF are accessible

### Spheres not visible in RViz2
- Add a **MarkerArray** display
- Set the topic to `/visualization_marker_array`
- Ensure the Fixed Frame matches your robot's base frame

### YAML export fails
- Ensure base link and end effector link are set
- Verify joint configuration is complete
- Check write permissions for the output directory

## Contributing

Contributions are welcome! Please submit issues and pull requests to the repository.

## Maintainer

Will-44

## Acknowledgments

This tool is designed to work with [NVIDIA cuRobo](https://github.com/NVlabs/curobo), a GPU-accelerated motion planning framework.
