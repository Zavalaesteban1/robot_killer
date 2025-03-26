# Robot Killer Integration Guide

This guide provides a step-by-step approach to testing and integrating the robot_killer package. Follow this sequence to systematically verify all components and ensure a smooth integration process.

## Prerequisites

- ROS 2 Humble installed
- Required dependencies installed
- Physical robot hardware ready (LiDAR, IMU, ESP32)
- A testing area for mapping and navigation

## Initial Setup

```bash
# Clone the repository (if not already done)
cd ~/ros2_ws/src
git clone [repository-url] robot_killer

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select robot_killer

# Source the workspace
source install/setup.bash
```

## Day 1: Hardware Testing

### Step 1: Check Hardware Connections

```bash
# Check available USB ports
ls -l /dev/ttyUSB*

# Set permissions
sudo chmod 666 /dev/ttyUSB*
```

### Step 2: Test Individual Components

```bash
# Test LiDAR
ros2 launch robot_killer lidar.launch.py serial_port:=/dev/ttyUSB0

# In another terminal, verify data
ros2 topic echo /scan

# Test IMU 
ros2 launch robot_killer bno055.launch.py port:=/dev/ttyUSB1

# Verify IMU data
ros2 topic echo /imu

# Test ESP32
ros2 launch robot_killer esp32.launch.py serial_port:=/dev/ttyUSB2

# Verify odometry
ros2 topic echo /odom
```

### Step 3: Basic Hardware Integration

```bash
# Integrated hardware test with teleop
ros2 launch robot_killer teleop.launch.py lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 esp32_port:=/dev/ttyUSB2
```

#### Verification Tasks:
- [ ] Robot responds to teleop keyboard commands
- [ ] LiDAR data appears in RViz
- [ ] IMU orientation data is correct
- [ ] Robot can move forward/backward and turn

## Day 2: SLAM Mapping

### Step 1: Start SLAM with Teleop

```bash
# Launch teleop SLAM
ros2 launch robot_killer teleop_slam.launch.py lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 esp32_port:=/dev/ttyUSB2
```

### Step 2: Create a Complete Map

Mapping best practices:
- Drive slowly around the perimeter first
- Make multiple passes through important areas
- Close loops to improve accuracy
- Cover the entire test environment

### Step 3: Save the Map

```bash
# Save the map when finished
ros2 launch robot_killer save_map.launch.py map_name:=test_area
```

#### Verification Tasks:
- [ ] SLAM produces a clean, accurate map
- [ ] Map shows all walls and obstacles clearly
- [ ] Map is saved correctly to the robot_killer/maps directory
- [ ] Map files include both .pgm and .yaml files

## Day 3: Navigation Testing

### Step 1: Test Navigation with Teleop Backup

```bash
# Launch navigation with teleop
ros2 launch robot_killer teleop_nav.launch.py map_file:=robot_killer/maps/test_area.yaml
```

### Step 2: Test Navigation Goals

In RViz:
1. Click "2D Nav Goal" button
2. Click on the map to set a destination
3. Observe the robot planning and following a path

### Step 3: Test Basic Navigation

```bash
# Launch navigation without teleop
ros2 launch robot_killer main.launch.py mode:=nav map_file:=robot_killer/maps/test_area.yaml
```

#### Verification Tasks:
- [ ] Robot correctly localizes in the map
- [ ] Path planning works for various destinations
- [ ] Robot follows planned paths accurately
- [ ] Recovery behaviors work when path is blocked

## Day 4: Autonomous Features Testing

### Step 1: Test Temperature Detection (If Available)

```bash
# Run temperature simulator if you have one
ros2 run robot_killer temperature_simulator
```

### Step 2: Test Full Autonomous Mode

```bash
# Launch autonomous navigation
ros2 launch robot_killer main.launch.py mode:=nav autonomous:=true map_file:=robot_killer/maps/test_area.yaml
```

### Step 3: Keep Emergency Teleop Ready

```bash
# In another terminal, for emergency control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Verification Tasks:
- [ ] Mission controller cycles through states properly
- [ ] Robot explores environment systematically
- [ ] Fire detection works (if temperature source available)
- [ ] Robot returns to starting position after mission complete

## Integration Troubleshooting

### Transform Issues

```bash
# Visualize transform tree
ros2 run tf2_tools view_frames

# Check transform publications
ros2 topic echo /tf
```

Common problems:
- Missing transforms between frames
- Incorrect parent/child relationships
- Transforms published at wrong rate

### Topic Communication Issues

```bash
# List all topics
ros2 topic list

# Check node connections
ros2 node info /node_name

# View entire node graph
rqt_graph
```

### Navigation Problems

```bash
# Check costmap
ros2 topic echo /global_costmap/costmap

# Monitor robot pose
ros2 topic echo /amcl_pose
```

Common issues:
- Poor localization (robot not where it should be on map)
- Obstacles not detected correctly
- Path planning timeout or failures

### Port Assignment Conflicts

If multiple devices try to use the same port:
1. Identify which device is on which port: `ls -l /dev/ttyUSB*`
2. Explicitly assign ports when launching:
   ```bash
   ros2 launch robot_killer main.launch.py lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 esp32_port:=/dev/ttyUSB2
   ```

## Launch Commands Reference

### Basic Launch Options

```bash
# Basic teleop control
ros2 launch robot_killer teleop.launch.py

# Mapping with teleop
ros2 launch robot_killer teleop_slam.launch.py

# Navigation with teleop
ros2 launch robot_killer teleop_nav.launch.py map_file:=robot_killer/maps/your_map.yaml

# Full system with round1 launch file
ros2 launch robot_killer round1.launch.py mode:=nav use_rviz:=true

# Main entry point
ros2 launch robot_killer main.launch.py mode:=slam

# Save a map
ros2 launch robot_killer save_map.launch.py map_name:=your_map
```

### Launch with Custom Ports

```bash
ros2 launch robot_killer main.launch.py mode:=slam lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 esp32_port:=/dev/ttyUSB2
```

### Launch with Autonomous Features

```bash
ros2 launch robot_killer main.launch.py mode:=nav autonomous:=true map_file:=robot_killer/maps/your_map.yaml
```

## Common ROS 2 Commands for Debugging

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic_name

# View topic info
ros2 topic info /topic_name

# View node info
ros2 node info /node_name

# List parameters for a node
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name parameter_name
```

## System Architecture Summary

1. **Hardware Layer**
   - LiDAR provides scan data
   - IMU provides orientation
   - ESP32 handles motor control and odometry

2. **Transform System**
   - robot_state_publisher provides transforms from URDF
   - All sensors positioned correctly in robot frame

3. **Mapping System**
   - SLAM Toolbox creates and updates maps
   - Map server provides maps for navigation

4. **Navigation System**
   - AMCL for localization
   - Nav2 planner for path planning
   - Nav2 controller for path following

5. **Autonomous Layer**
   - mission_controller handles high-level behavior
   - fire_detector identifies potential fires
   - Map-based navigation to mission objectives

6. **User Interface**
   - RViz for visualization
   - Teleop for manual control
   - Status reporting for mission feedback

## Team Roles During Testing

For efficient testing, consider assigning these roles:

1. **Driver**: Operates teleop controls
2. **Observer**: Watches physical robot behavior
3. **Monitor**: Tracks system state and ROS topics
4. **Coordinator**: Guides the testing process

## Conclusion

By following this systematic approach, you can verify each component works individually and then test their integration. The key is to build confidence in each subsystem before moving to more complex behaviors. If problems arise, isolate them to specific components before attempting to fix the entire system.

Remember that ROS 2 provides many tools for introspection and debugging - use them liberally to understand what's happening in your system. 