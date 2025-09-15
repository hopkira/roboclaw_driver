# RoboClaw Driver

A ROS2 driver for RoboClaw motor controllers, designed to replicate the functionality of the TeensyV2 real-time motor control system.

## Features

- **High-frequency motor control**: ~67Hz command rate following TeensyV2 patterns
- **Real-time odometry**: Encoder-based odometry calculation and publishing
- **Differential drive kinematics**: Converts cmd_vel to motor speeds
- **RoboClaw communication**: Direct serial communication with error handling
- **Safety features**: Command timeout and motor stop functionality
- **Configurable parameters**: Extensive configuration through ROS2 parameters

## Hardware Requirements

- RoboClaw motor controller (tested with 2x7A, 2x15A, 2x30A models)
- USB to serial adapter or direct serial connection
- Differential drive robot with encoders

## Installation

```bash
# Clone into your ROS2 workspace
cd ~/your_workspace/src
git clone <repository_url>

# Build the package
cd ~/your_workspace
colcon build --packages-select roboclaw_driver

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch roboclaw_driver roboclaw_driver.launch.py
```

### Custom Configuration

```bash
ros2 launch roboclaw_driver roboclaw_driver.launch.py config_file:=/path/to/your/config.yaml
```

### Manual Node Launch

```bash
ros2 run roboclaw_driver roboclaw_driver_node --ros-args --params-file /path/to/config.yaml
```

## Configuration

Key parameters in `config/motor_driver.yaml`:

### Device Settings
- `device_name`: Serial device path (default: "/dev/ttyUSB0")
- `baud_rate`: Communication baud rate (default: 38400)
- `device_timeout`: Command timeout in milliseconds (default: 100)

### Robot Parameters
- `wheel_diameter`: Wheel diameter in meters (default: 0.167)
- `wheel_separation`: Distance between wheels in meters (default: 0.495)
- `encoder_counts_per_revolution`: Total encoder counts per wheel revolution (default: 1440)

### Motor Control
- `max_velocity`: Maximum linear velocity in m/s (default: 1.0)
- `max_angular_velocity`: Maximum angular velocity in rad/s (default: 2.0)
- `accel`: Motor acceleration in encoder counts per second squared (default: 2000)

### PID Parameters
- `m1_p`, `m1_i`, `m1_d`, `m1_qpps`: Motor 1 PID values
- `m2_p`, `m2_i`, `m2_d`, `m2_qpps`: Motor 2 PID values

## Topics

### Subscribed Topics
- `cmd_vel` (geometry_msgs/Twist): Velocity commands

### Published Topics
- `odom` (nav_msgs/Odometry): Robot odometry
- `joint_states` (sensor_msgs/JointState): Wheel joint states (optional)

### TF Frames
- Publishes transform from `odom` to `base_link` (configurable)

## Architecture

This driver follows the TeensyV2 architecture patterns:

1. **Single-threaded main loop**: 67Hz execution frequency
2. **Atomic cmd_vel caching**: Thread-safe velocity command handling  
3. **State machine sensor reading**: Distributed status reads across cycles
4. **getFreshEncoders pattern**: Reliable encoder reading with retries
5. **SpeedAccelDistanceM1M2 commands**: TeensyV2-compatible motor control

## Debugging

Enable debug output in configuration:

```yaml
roboclaw_driver:
  ros__parameters:
    debug_mode: true
    debug_commands: true
    debug_encoders: true
    debug_timing: true
```

## Safety Features

- **Command timeout**: Motors stop if no cmd_vel received within timeout period
- **Connection monitoring**: Automatic reconnection on communication failures
- **Error handling**: Robust retry logic for serial communication
- **Parameter validation**: Sanity checks on configuration values

## Compatibility

- **ROS2**: Jazzy (primary), Humble (should work)
- **OS**: Linux (Ubuntu 22.04/24.04 tested)
- **Hardware**: RoboClaw 2x7A, 2x15A, 2x30A controllers
- **Architecture**: TeensyV2-compatible command patterns

## Troubleshooting

### Common Issues

1. **Permission denied on serial device**:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

2. **No communication with RoboClaw**:
   - Check device path: `ls /dev/ttyUSB*`
   - Verify baud rate matches RoboClaw configuration
   - Test with RoboClaw utility software first

3. **Encoder readings invalid**:
   - Check encoder wiring
   - Verify encoder counts per revolution parameter
   - Enable debug_encoders for diagnostic output

4. **Motors not responding**:
   - Check motor wiring and power
   - Verify PID parameters are set correctly
   - Monitor RoboClaw error status

### Debug Commands

```bash
# Monitor topics
ros2 topic echo /odom
ros2 topic echo /joint_states

# Send test commands
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" --once

# Check parameters
ros2 param list /roboclaw_driver
ros2 param get /roboclaw_driver device_name
```

## Performance Notes

- Target frequency: 67Hz main loop (following TeensyV2)
- Encoder reading: ~67Hz for fresh odometry data
- Motor commands: Rate-limited to prevent serial overload
- Status reading: Distributed across cycles (10Hz effective rate)

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch  
5. Create a Pull Request

## Acknowledgments

- Based on TeensyV2 RoboClaw implementation by Wimble Robotics
- RoboClaw protocol implementation adapted from BasicMicro examples
- Differential drive kinematics from standard robotics references
