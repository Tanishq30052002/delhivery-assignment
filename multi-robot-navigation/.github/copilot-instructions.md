# AI Agent Instructions for Multi-Robot Navigation

This ROS2 package implements a multi-robot navigation system for a warehouse environment featuring two types of robots: tugs and pickers. The system includes SLAM, localization, and visualization capabilities.

## Project Architecture

### Key Components
- **Robot Types**:
  - Tug robots (tug_1, tug_2): Basic differential drive with LIDAR and IMU
  - Picker robots (picker_1, picker_2): Enhanced tugs with additional depth cameras

### Critical Directories
- `/urdf/`: Robot model definitions
  - `*.core.xacro`: Top-level robot definitions
  - `*.model.xacro`: Robot structure and dimensions
  - `*.gazebo.xacro`: Gazebo-specific configurations and sensors
- `/launch/`: Launch files for different scenarios
  - `gazebo.launch.py`: Main simulation launcher
  - `spawn_robots.launch.py`: Multi-robot fleet spawning
  - `mapping.launch.py`: SLAM configuration
  - `localization.launch.py`: Multi-robot AMCL setup

### Coordinate Frames and Transformations
- Global frame: `map`
- Per-robot frames follow naming pattern `{robot_ns}/{frame_name}`:
  - Base: `{robot_ns}/robot`
  - Odometry: `{robot_ns}/odom`
  - Sensors: `{robot_ns}/lidar`, `{robot_ns}/imu`, `{robot_ns}/depth_camera` (pickers only)

## Development Workflows

### Building and Running
```bash
# Build package
colcon build --packages-select multi-robot-navigation

# Launch simulation
ros2 launch multi-robot-navigation gazebo.launch.py

# Start mapping (with tug_1 as reference)
ros2 launch multi-robot-navigation mapping.launch.py

# Run localization for all robots
ros2 launch multi-robot-navigation localization.launch.py
```

### Key Integration Points
- ROS2 Topics:
  - `/{robot_ns}/scan`: LaserScan data for navigation
  - `/{robot_ns}/depth/image_raw`: Depth camera data (pickers only)
  - `/{robot_ns}/imu`: IMU measurements
  - `/map`: Shared map for all robots

### Configuration Patterns
- Robot-specific parameters use namespaced YAML files (e.g., `config/amcl_config_tug_1.yaml`)
- Sensors are configured in their respective `.gazebo.xacro` files
- Robot dimensions and physical properties are defined in `.model.xacro` files

## Common Development Tasks

### Adding New Robot Types
1. Create XACRO files in `/urdf/`:
   - `{robot_name}.core.xacro`
   - `{robot_name}.model.xacro`
   - `{robot_name}.gazebo.xacro`
2. Add spawn configuration in `spawn_robots.launch.py`
3. Create AMCL config in `/config/` if using localization

### Modifying Robot Sensors
- Sensor parameters in `.gazebo.xacro` files control:
  - Update rates
  - Noise models
  - Publishing topics
  - Frame names

### Visualization
- RViz configurations in `/rviz/` are optimized for:
  - `slam_toolbox.rviz`: Mapping visualization
  - `localization.rviz`: Multi-robot localization
  - `fleet_config.rviz`: Full fleet visualization