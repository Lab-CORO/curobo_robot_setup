# cuRobo Robot Setup Tutorial

This tutorial will guide you through the complete process of configuring a robot for use with NVIDIA cuRobo using the cuRobo Robot Setup RViz2 plugin.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Loading Your Robot URDF](#loading-your-robot-urdf)
3. [Defining Collision Spheres](#defining-collision-spheres)
4. [Configuring Joint Parameters](#configuring-joint-parameters)
5. [Setting Up Self-Collision Avoidance](#setting-up-self-collision-avoidance)
6. [Exporting the Configuration](#exporting-the-configuration)
7. [Using the Configuration with cuRobo](#using-the-configuration-with-curobo)
8. [Best Practices](#best-practices)
9. [Advanced Topics](#advanced-topics)

---

## Getting Started

### Prerequisites

Before starting this tutorial, ensure you have:
- ROS 2 installed (Humble or later)
- Built and sourced the `curobo_robot_setup` package
- A URDF file for your robot
- Basic familiarity with RViz2

### Launch RViz2

Open a terminal and launch RViz2:

```bash
source ~/ros2_ws/install/setup.bash
rviz2
```

### Add the cuRobo Robot Setup Panel

1. In RViz2, click **Panels** in the menu bar
2. Select **Add New Panel**
3. In the dialog, find and select **curobo_robot_setup/CuRoboSetupPanel**
4. Click **OK**

The panel will appear, typically on the left side of the RViz2 window. You should see three tabs:
- **URDF Loader**
- **Sphere Editor**
- **Configuration**

---

## Loading Your Robot URDF

### Step 1: Prepare Your URDF

Ensure your URDF file:
- Is syntactically correct (test with `check_urdf your_robot.urdf`)
- Has all mesh files in accessible locations
- Includes joint definitions with proper limits
- Has a clear kinematic chain from base to end effector

### Step 2: Load the URDF

1. Navigate to the **URDF Loader** tab in the plugin panel
2. Click the **Load URDF** button
3. Browse to your URDF file location
4. Select your `.urdf` file and click **Open**

### Step 3: Verify Loading

After loading:
- The status label should show "URDF loaded successfully"
- The URDF is published to the `/robot_description` topic
- To visualize the robot, add a **RobotModel** display in RViz2:
  1. Click **Add** button in the Displays panel
  2. Select **RobotModel**
  3. Set the **Robot Description** topic to `/robot_description`

### Troubleshooting

**Problem**: URDF fails to load
- Check the console for error messages
- Verify file path is correct
- Ensure all mesh files are accessible
- Run `check_urdf` to validate the URDF

**Problem**: Robot doesn't appear in RViz2
- Check the Fixed Frame in RViz2 matches your robot's base frame
- Verify the RobotModel display is subscribed to `/robot_description`
- Check that TF frames are being published if needed

---

## Defining Collision Spheres

Collision spheres are primitive shapes used by cuRobo for fast collision checking. The goal is to approximate your robot's geometry with multiple overlapping spheres.

### Why Collision Spheres?

- GPU-friendly: Sphere-to-sphere collision checks are extremely fast
- Conservative: Properly placed spheres ensure safe motion planning
- Lightweight: Reduces computational overhead compared to mesh collision

### Step 1: Understand the Link Tree

1. Navigate to the **Sphere Editor** tab
2. Examine the link tree on the left side

The tree shows your robot's kinematic structure:
- Parent links contain child links
- Click the arrow to expand/collapse branches
- Each link displays its type (fixed, revolute, prismatic, etc.)

### Step 2: Add Spheres to a Link

Let's add collision spheres to the first arm link:

1. **Select a link**: Click on a link in the tree (e.g., "link1")
2. **Add a sphere**: Click the **Add Sphere** button
3. **Set the radius**: Adjust the **Radius** spin box (e.g., 0.08 meters)
4. **Set the position**: Adjust X, Y, Z spin boxes to position the sphere
   - Position is relative to the link's coordinate frame
   - Use the link's origin as reference

### Step 3: Visualize Spheres in RViz2

To see your spheres in RViz2:

1. Click **Add** in the Displays panel
2. Select **MarkerArray**
3. Set the **Topic** to `/visualization_marker_array`
4. Spheres will appear as colored markers

### Step 4: Add Multiple Spheres per Link

For longer links, you'll need multiple overlapping spheres:

1. Select the link
2. Add another sphere with **Add Sphere**
3. Position it along the link's length
4. Repeat to cover the entire link

**Example for a long arm segment**:
```
Link: link2 (length ~0.6m along X-axis)
Sphere 1: center=[0.0, 0.0, 0.0], radius=0.08
Sphere 2: center=[0.15, 0.0, 0.0], radius=0.08
Sphere 3: center=[0.30, 0.0, 0.0], radius=0.08
Sphere 4: center=[0.45, 0.0, 0.0], radius=0.08
Sphere 5: center=[0.60, 0.0, 0.0], radius=0.08
```

### Step 5: Iterate Through All Links

Repeat the process for each link that needs collision checking:

1. **Base link**: Often needs a large sphere at the robot's mounting point
2. **Arm links**: Multiple spheres covering the entire length
3. **Wrist links**: Smaller spheres for compact geometry
4. **End effector**: Include the tool/gripper geometry

### Best Practices for Sphere Placement

- **Overlap spheres**: Ensure neighboring spheres overlap by 10-20%
- **Conservative sizing**: Slightly oversized spheres are safer than undersized
- **Match geometry**: Place spheres to approximate the actual link shape
- **Avoid gaps**: No gaps should exist in the collision model
- **Check all angles**: Rotate the robot in RViz2 to verify coverage

### Deleting Spheres

To remove a sphere:
1. Select the link in the link tree
2. Click on the sphere in the sphere list
3. Click **Delete Sphere**

---

## Configuring Joint Parameters

Joint configuration defines how cuRobo plans and optimizes motion.

### Step 1: Open Joint Configuration Dialog

1. Navigate to the **Configuration** tab
2. Click the **Configure Joints** button

A dialog window will open showing all joints detected from your URDF.

### Step 2: Configure Each Joint

For each joint, you can set:

#### Joint Name
- Read from the URDF
- Used to identify the joint in cuRobo

#### Position Limits
- **Min Position**: Minimum joint angle/position (radians or meters)
- **Max Position**: Maximum joint angle/position
- Usually read from URDF `<limit>` tags
- Can be adjusted to be more conservative

#### Velocity and Acceleration
- **Max Velocity**: Maximum joint velocity (rad/s or m/s)
- **Max Acceleration**: Maximum joint acceleration
- **Max Jerk**: Maximum rate of acceleration change
- These affect trajectory smoothness and speed

#### Retract Configuration
- **Retract Value**: Safe "home" position for the joint
- Used as a reference configuration
- Typically 0.0 or the robot's neutral pose

#### Weights
- **Null Space Weight**: Priority for this joint in null-space optimization (0.0-1.0)
- **C-Space Distance Weight**: How much this joint contributes to distance metrics
- Higher values = more importance in optimization

### Step 3: Example Configuration

For a 6-DOF robot arm:

```
Joint 1 (Base rotation):
  Min: -3.14, Max: 3.14
  Max Vel: 2.0, Max Accel: 10.0
  Retract: 0.0
  Null Space Weight: 1.0
  C-Space Distance Weight: 1.0

Joint 2 (Shoulder):
  Min: -1.57, Max: 1.57
  Max Vel: 2.0, Max Accel: 10.0
  Retract: -0.5
  Null Space Weight: 1.0
  C-Space Distance Weight: 1.0

[... continue for all joints ...]
```

### Step 4: Set Global Parameters

At the bottom of the dialog:
- **Max Jerk**: Global maximum jerk (default: 500.0)
- **Max Acceleration**: Global maximum acceleration (default: 15.0)

Click **OK** to save the configuration.

---

## Setting Up Self-Collision Avoidance

Self-collision configuration tells cuRobo which link pairs should never collide.

### Step 1: Understand Self-Collision Pairs

Adjacent links in the kinematic chain are often ignored because:
- They're connected by a joint and can't physically collide
- Checking them wastes computation

### Step 2: Configure Ignore Pairs

In the **Configuration** tab:

1. **Base Link**: Select the robot's root link (e.g., "base_link")
2. **End Effector Link**: Select the tool/gripper link (e.g., "link6")

### Step 3: Define Collision Ignore List

Common ignore patterns:

```yaml
self_collision_ignore:
  base_link: ["link1"]              # Base can't hit first link
  link1: ["link2", "link3"]          # Link1 can't hit adjacent links
  link2: ["link3", "link4"]          # Link2 can't hit nearby links
  link3: ["link4", "link5"]
  link4: ["link5", "link6"]
  link5: ["link6"]
```

### Step 4: Set Collision Buffers

Collision buffers add safety margins:

1. **Collision Sphere Buffer**: Global buffer added to all spheres (e.g., 0.01m)
2. **Self-Collision Buffer**: Per-link buffer for self-collision (usually 0.0)

```yaml
collision_sphere_buffer: 0.01

self_collision_buffer:
  link1: 0.0
  link2: 0.0
  link3: 0.0
  link4: 0.0
  link5: 0.0
  link6: 0.0
```

---

## Exporting the Configuration

### Step 1: Verify Configuration

Before exporting, verify:
- ✓ All links have collision spheres
- ✓ Joint parameters are configured
- ✓ Base link and end effector are set
- ✓ Self-collision ignore pairs are defined

### Step 2: Save YAML

1. Navigate to the **Configuration** tab
2. Click **Save YAML**
3. Choose a save location
4. Name your file (e.g., `my_robot_curobo.yaml`)
5. Click **Save**

### Step 3: Inspect the Output

Open the generated YAML file and verify:
- URDF path is correct
- All collision spheres are present
- Joint names match your robot
- Configuration values are reasonable

Example output structure:
```yaml
robot_cfg:
  kinematics:
    urdf_path: "/path/to/robot.urdf"
    base_link: "base_link"
    ee_link: "link6"
    collision_link_names: ['link1', 'link2', ...]
    collision_spheres:
      link1:
        - center: [x, y, z]
          radius: r
    cspace:
      joint_names: ['joint1', 'joint2', ...]
      retract_config: [0.0, 0.0, ...]
```

---

## Using the Configuration with cuRobo

### Step 1: Install cuRobo

Follow the official cuRobo installation guide:
```bash
pip install nvidia-curobo
```

### Step 2: Load Your Configuration

In your Python script:

```python
from curobo.wrap.reacher.motion_gen import MotionGen
from curobo.wrap.model.robot_world import RobotWorld

# Load your custom robot configuration
motion_gen = MotionGen.from_robot_config(
    robot_cfg_file="/path/to/my_robot_curobo.yaml",
    world_cfg_file=None  # Add obstacles if needed
)
```

### Step 3: Plan Motion

```python
# Define goal pose
goal_pose = Pose(
    position=[0.5, 0.0, 0.5],
    orientation=[1.0, 0.0, 0.0, 0.0]  # quaternion
)

# Plan motion
result = motion_gen.plan_single(goal_pose)

if result.success:
    trajectory = result.get_interpolated_plan()
    # Execute trajectory on your robot
```

---

## Best Practices

### Collision Sphere Design

1. **Start with critical links**: Focus on the largest links first
2. **Use uniform spacing**: Keep spheres evenly distributed
3. **Test coverage**: View from multiple angles in RViz2
4. **Err on the side of safety**: Slightly larger spheres are better
5. **Document your choices**: Add comments about sphere placement rationale

### Joint Configuration

1. **Use conservative limits**: Reduce URDF limits by 5-10% for safety
2. **Match real robot**: Test that configured velocities match hardware
3. **Tune weights iteratively**: Start with uniform weights (1.0), adjust if needed
4. **Consider task requirements**: Faster tasks need higher acceleration limits

### Performance Optimization

1. **Minimize spheres**: Use as few as possible while maintaining coverage
2. **Ignore adjacent links**: Don't check links that can't collide
3. **Test in simulation first**: Validate before deploying to hardware

### Workflow Tips

1. **Save frequently**: Use **Save YAML** regularly during configuration
2. **Version control**: Keep multiple versions of configurations
3. **Document changes**: Note what you changed and why
4. **Test incrementally**: Load and test after each major change

---

## Advanced Topics

### Loading Existing Configurations

To modify an existing configuration:

1. Click **Load YAML** in the Configuration tab
2. Select your existing `.yaml` file
3. The tool will load:
   - Collision spheres
   - Joint configurations
   - Base/EE links
   - Self-collision settings

### Multi-Robot Setups

For robots with multiple arms or mobile bases:

1. Configure each arm separately
2. Create separate YAML files
3. Combine in cuRobo using the multi-robot API

### Custom Collision Geometries

For non-standard geometries:

1. Use more spheres for better approximation
2. Consider varying sphere sizes
3. Test collision detection thoroughly

### Integration with MoveIt

If using MoveIt alongside cuRobo:

1. Ensure URDF consistency
2. Match joint names exactly
3. Coordinate collision checking approaches

### Debugging Configuration Issues

If cuRobo reports errors:

1. **"Invalid joint limits"**: Check min < max for all joints
2. **"Missing collision spheres"**: Ensure all links in `collision_link_names` have spheres
3. **"Self-collision detected"**: Adjust ignore pairs or increase buffers
4. **"URDF parse error"**: Validate URDF with `check_urdf`

### Performance Benchmarking

Test your configuration:

```python
# Benchmark motion planning time
import time

start = time.time()
result = motion_gen.plan_single(goal_pose)
planning_time = time.time() - start

print(f"Planning time: {planning_time*1000:.2f} ms")
print(f"Success: {result.success}")
```

Aim for planning times under 100ms for reactive applications.

---

## Conclusion

You now have a complete understanding of how to configure robots for cuRobo using the cuRobo Robot Setup plugin. The key steps are:

1. Load your URDF
2. Define collision spheres for all links
3. Configure joint parameters
4. Set self-collision avoidance rules
5. Export and test the configuration

For additional help:
- Check the [README.md](README.md) for troubleshooting
- Visit the [cuRobo documentation](https://github.com/NVlabs/curobo)
- Examine the example configuration in [example/m1013.yml](example/m1013.yml)

Happy motion planning!
