# RoboClaw Driver for ROS2

A professional ROS2 driver for BasicMicro RoboClaw motor controllers, designed for differential drive robots. This driver provides reliable motor control, accurate odometry, and comprehensive robot status monitoring.

## What This Driver Does

The RoboClaw driver allows you to:
- **Control your robot's movement** using standard ROS2 velocity commands (`cmd_vel`)
- **Track your robot's position** with built-in odometry calculation
- **Monitor motor health** including current, temperature, and error status
- **Integrate with ROS2 navigation** stack for autonomous robot operation

## Hardware Requirements

### Essential Components
- **RoboClaw motor controller** (tested with 2x7A, 2x15A, 2x30A models)
- **Differential drive robot** with two motors
- **Quadrature encoders** on both drive wheels
- **Serial connection** (USB cable or UART pins)

### Compatible Robots
- Two-wheeled differential drive robots
- Tracked vehicles with separate left/right drive
- Skid-steer platforms

## Quick Start Guide

### 1. Installation

```bash
# Navigate to your ROS2 workspace
cd ~/your_ros2_workspace/src

# Clone this repository
git clone <repository_url>

# Build the package
cd ~/your_ros2_workspace
colcon build --packages-select roboclaw_driver

# Source the workspace
source install/setup.bash
```

### 2. Hardware Setup

1. **Connect your RoboClaw to your computer:**
   - USB: Connect USB cable from RoboClaw to computer
   - Serial: Connect TX/RX pins to UART adapter

2. **Check device permissions:**
   ```bash
   # Find your device (usually /dev/ttyUSB0 or /dev/ttyAMA0)
   ls /dev/tty*
   
   # Add yourself to dialout group for USB access
   sudo usermod -a -G dialout $USER
   # Log out and back in for this to take effect
   ```

3. **Test basic communication:**
   ```bash
   # Check if device exists and is accessible
   ls -l /dev/ttyUSB0  # or your device path
   ```

### 3. Configuration

Edit the configuration file at `config/motor_driver.yaml` with your robot's specifications:

```yaml
roboclaw_driver:
  ros__parameters:
    # YOUR ROBOT'S MEASUREMENTS - IMPORTANT!
    wheel_radius: 0.051112072              # Measure your wheel radius in meters
    wheel_separation: 0.3906               # Measure distance between wheel centers
    encoder_counts_per_revolution: 1000    # Check your encoder specification
    
    # SERIAL CONNECTION - UPDATE FOR YOUR SETUP
    device_name: "/dev/ttyUSB0"            # Your RoboClaw device path
    baud_rate: 230400                      # Match your RoboClaw settings
```

### 4. Launch the Driver

```bash
# Use default configuration
ros2 launch roboclaw_driver roboclaw_driver.launch.py

# Or specify custom config
ros2 launch roboclaw_driver roboclaw_driver.launch.py config_file:=/path/to/your/config.yaml
```

### 5. Test Your Robot

```bash
# Send a test command to move forward slowly
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" --once

# Check odometry
ros2 topic echo /odom

# Monitor robot status
ros2 topic echo /roboclaw_status
```

## Configuration Parameters Explained

### Device Communication
```yaml
device_name: "/dev/ttyUSB0"           # Serial device path for RoboClaw
baud_rate: 230400                     # Serial communication speed (must match RoboClaw setting)
device_timeout: 100                   # Command timeout in milliseconds
```

### Robot Physical Parameters ⚠️ **CRITICAL - MEASURE YOUR ROBOT**
```yaml
wheel_radius: 0.051112072             # Wheel radius in meters - MEASURE THIS!
wheel_separation: 0.3906              # Distance between wheel centers - MEASURE THIS!
encoder_counts_per_revolution: 1000   # Total encoder counts per wheel revolution - CHECK SPECS!
```

### Publishing Rates (How Often Data is Sent)
```yaml
odometry_rate: 67.0                   # How often position updates are published (Hz)
joint_states_rate: 30.0               # How often wheel joint data is published (Hz)  
status_rate: 10.0                     # How often motor status is published (Hz)
```

### Safety Limits
```yaml
max_linear_velocity: 0.3              # Maximum forward/backward speed (m/s)
max_angular_velocity: 0.7             # Maximum turning speed (rad/s)
max_seconds_uncommanded_travel: 0.2   # Safety timeout - motors stop if no new commands
```

### Motor Control (PID Parameters)
```yaml
# Motor 1 (usually left) PID settings
m1_p: 7.26239                         # Proportional gain
m1_i: 2.43                            # Integral gain  
m1_d: 0.0                             # Derivative gain
m1_qpps: 2437                         # Max quadrature pulses per second

# Motor 2 (usually right) PID settings  
m2_p: 7.26239                         # Proportional gain
m2_i: 2.43                            # Integral gain
m2_d: 0.0                             # Derivative gain  
m2_qpps: 2437                         # Max quadrature pulses per second

accel: 3000                           # Motor acceleration (encoder counts/s²)
decel: 6000                           # Motor deceleration (encoder counts/s²)
emergency_decel: 12000                 # Full stop motor deceleration (encoder counts/s²)
```

### What to Publish
```yaml
publish_odom: false                   # Publish odometry messages (/odom topic)
publish_tf: false                     # Publish transform data (requires publish_odom: true)
publish_joint_states: false           # Publish wheel joint states (/joint_states topic)
```

### Debugging
```yaml
do_debug: false                       # Enable detailed command logging
do_low_level_debug: false             # Enable low-level serial communication logging
```

## ROS2 Topics

### What Your Robot Listens To
- **`/cmd_vel`** (geometry_msgs/Twist): Send velocity commands here to move your robot

### What Your Robot Publishes
- **`/odom`** (nav_msgs/Odometry): Robot position and velocity (if `publish_odom: true`)
- **`/joint_states`** (sensor_msgs/JointState): Wheel positions and speeds (if `publish_joint_states: true`)
- **`/roboclaw_status`** (std_msgs/String): Motor health information (always published)

## Common Problems and Solutions

### "Permission denied" on serial device
```bash
# Add your user to the dialout group
sudo usermod -a -G dialout $USER
# Log out and back in, then try again
```

### Robot moves in wrong direction
- Swap motor connections M1 ↔ M2 on RoboClaw, or
- Modify the `wheel_separation` sign in config, or  
- Check encoder wiring polarity

### Robot spins instead of going straight
- Check `wheel_separation` measurement
- Verify both wheels are the same size
- Check encoder counts per revolution for both wheels

### Robot doesn't move at all
1. Check power connections to motors
2. Verify RoboClaw is powered on (LED indicators)
3. Test with BasicMicro Motion Studio software first
4. Check `baud_rate` matches RoboClaw configuration
5. Verify `device_name` path is correct

### Jerky or unstable movement
- Adjust PID parameters (`m1_p`, `m1_i`, `m1_d`, etc.)
- Use BasicMicro Motion Studio to tune PID values first
- Check for loose encoder connections
- Reduce `max_linear_velocity` and `max_angular_velocity`

### Poor odometry accuracy
- Carefully re-measure `wheel_radius` and `wheel_separation`
- Verify `encoder_counts_per_revolution` matches your encoders
- Check for wheel slippage or mechanical backlash
- Calibrate by driving known distances and measuring error

## Getting Help

### Check System Status
```bash
# See what parameters are loaded
ros2 param list /roboclaw_driver

# Check a specific parameter  
ros2 param get /roboclaw_driver wheel_radius

# Monitor data flow
ros2 topic hz /odom  # Should show ~67 Hz if publishing
ros2 topic hz /cmd_vel  # Shows how often you're sending commands
```

### Enable Debug Output
Set `do_debug: true` in your config file to see detailed logging information.

### Test Hardware Separately
Before using this driver, test your RoboClaw with BasicMicro's Motion Studio software to ensure:
- Serial communication works
- Motors respond to commands  
- Encoders provide clean signals
- PID parameters are reasonable

### Getting Support
1. Check the troubleshooting section above
2. Enable debug logging and examine output
3. Test with BasicMicro Motion Studio first
4. Open an issue with detailed logs and configuration

## Advanced Topics

### ROS2 Transform Frames
The driver publishes transforms between coordinate frames:
- **`odom`** → **`base_link`**: Robot position in odometry frame (if `publish_tf: true`)

Frame names are configurable:
```yaml
base_frame: "base_link"              # Robot's main coordinate frame
odom_frame: "odom"                   # Odometry coordinate frame
```

### Integration with Navigation Stack
To use with ROS2 navigation:

1. Enable odometry and transforms:
   ```yaml
   publish_odom: true
   publish_tf: true
   ```

2. Tune parameters for your robot's performance
3. Use with nav2 stack for autonomous navigation

### Motor PID Tuning Guide

PID parameters control how smoothly and accurately your motors respond:

- **P (Proportional)**: Higher = more aggressive response, but may oscillate
- **I (Integral)**: Helps eliminate steady-state error, but can cause overshoot  
- **D (Derivative)**: Reduces overshoot, but may cause noise sensitivity
- **QPPS**: Maximum speed - should match your motor's capabilities

**Tuning Process:**
1. Start with all gains at 0
2. Increase P until robot responds well but may oscillate slightly
3. Add I to eliminate steady-state error
4. Add D to reduce overshoot if needed
5. Use BasicMicro Motion Studio for fine-tuning

## Architecture Details

This driver implements several advanced patterns for reliable robot control:

### TeensyV2 Compatibility
The timing and command patterns match the proven TeensyV2 real-time controller:
- 30Hz main control loop
- Distributed sensor reading across cycles
- Atomic command caching for thread safety
- Buffered motor commands with safety timeouts

### Thread Safety
- Velocity commands are cached atomically to prevent race conditions
- All RoboClaw communication happens in the main thread
- Publisher rate limiting prevents communication overload

### Error Handling
- Automatic retry logic for serial communication failures
- Comprehensive error status decoding and reporting
- Safety timeouts prevent runaway robot behavior
- Parameter validation during startup

## Technical Specifications

### Performance Characteristics
- **Main loop frequency**: 30 Hz (TeensyV2 compatible)
- **Odometry update rate**: Configurable (default 67 Hz)
- **Command latency**: < 50ms typical
- **Serial communication**: Full error handling and retry logic

### Resource Usage
- **CPU**: Low impact, single-threaded design
- **Memory**: Minimal footprint
- **Serial bandwidth**: Optimized command scheduling

## License and Credits

This project is licensed under the Apache-2.0 License.

### Acknowledgments
- Based on TeensyV2 RoboClaw implementation by Wimble Robotics
- RoboClaw protocol implementation adapted from BasicMicro documentation
- Differential drive kinematics from standard robotics references

## Development and Contributing

### Building from Source
```bash
git clone <repository_url>
cd roboclaw_driver
colcon build
```

### Running Tests
```bash
colcon test --packages-select roboclaw_driver
```

### Code Style
This project follows ROS2 C++ style guidelines with professional documentation standards.

---

**Need help?** Check the troubleshooting section above or open an issue with your configuration file and log output.
