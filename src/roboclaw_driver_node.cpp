// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 WimbleRobotics
// https://github.com/wimblerobotics/Sigyn

#include "roboclaw_driver/roboclaw_driver_node.hpp"

#include <rcutils/logging_macros.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sstream>

#include "nav_msgs/msg/odometry.hpp"
#include "roboclaw_driver/roboclaw_cmd_do_buffered_m1m2_drive_speed_accel_distance.h"
#include "roboclaw_driver/roboclaw_cmd_read_encoder.h"
#include "roboclaw_driver/roboclaw_cmd_read_encoder_speed.h"
#include "roboclaw_driver/roboclaw_cmd_read_firmware_version.h"
#include "roboclaw_driver/roboclaw_cmd_read_logic_battery_voltage.h"
#include "roboclaw_driver/roboclaw_cmd_read_main_battery_voltage.h"
#include "roboclaw_driver/roboclaw_cmd_read_motor_currents.h"
#include "roboclaw_driver/roboclaw_cmd_read_status.h"
#include "roboclaw_driver/roboclaw_cmd_read_temperature.h"
#include "roboclaw_driver/roboclaw_cmd_set_encoder_value.h"
#include "roboclaw_driver/roboclaw_cmd_set_pid.h"

using namespace std::chrono_literals;

#define SetWORDval(arg) (uint8_t)(((uint32_t)arg) >> 8), (uint8_t)arg
#define SetDWORDval(arg) \
  (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg

/**
 * @brief Constructor for RoboClawDriverNode
 *
 * Initializes the ROS2 node, sets up parameters, creates RoboClaw interface,
 * and starts the main control loop. Follows TeensyV2 architecture patterns
 * for compatibility with existing robot configurations.
 */
RoboClawDriverNode::RoboClawDriverNode()
    : Node("roboclaw_driver"),
      roboclaw_(nullptr),
      address_(ROBOCLAW_ADDRESS),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      linear_velocity_(0.0),
      angular_velocity_(0.0),
      first_encoder_reading_(true),
      current_status_state_(READ_BATTERY),
      motors_initialized_(false) {
  // Initialize parameters in separate functions for better organization
  declare_parameters();
  load_parameters();
  log_parameters();

  // Log all parameters for visibility during startup
  RCUTILS_LOG_INFO("=== RoboClaw Driver Parameters ===");
  RCUTILS_LOG_INFO("accel: %d", accel_);
  RCUTILS_LOG_INFO("base_frame: %s", base_frame_.c_str());
  RCUTILS_LOG_INFO("baud_rate: %d", baud_rate_);
  RCUTILS_LOG_INFO("do_debug: %s", do_debug_ ? "true" : "false");
  RCUTILS_LOG_INFO("do_low_level_debug: %s", do_low_level_debug_ ? "true" : "false");
  RCUTILS_LOG_INFO("device_name: %s", device_name_.c_str());
  RCUTILS_LOG_INFO("device_timeout: %d", device_timeout_);
  RCUTILS_LOG_INFO("encoder_counts_per_revolution: %d", encoder_counts_per_revolution_);
  RCUTILS_LOG_INFO("joint_states_rate: %.1f", joint_states_rate_);
  RCUTILS_LOG_INFO("m1_d: %.6f", m1_d_);
  RCUTILS_LOG_INFO("m1_i: %.6f", m1_i_);
  RCUTILS_LOG_INFO("m1_p: %.6f", m1_p_);
  RCUTILS_LOG_INFO("m1_qpps: %d", m1_qpps_);
  RCUTILS_LOG_INFO("m2_d: %.6f", m2_d_);
  RCUTILS_LOG_INFO("m2_i: %.6f", m2_i_);
  RCUTILS_LOG_INFO("m2_p: %.6f", m2_p_);
  RCUTILS_LOG_INFO("m2_qpps: %d", m2_qpps_);
  RCUTILS_LOG_INFO("max_angular_velocity: %.1f", max_angular_velocity_);
  RCUTILS_LOG_INFO("max_linear_velocity: %.1f", max_linear_velocity_);
  RCUTILS_LOG_INFO("max_seconds_uncommanded_travel: %.3f", max_seconds_uncommanded_travel_);
  RCUTILS_LOG_INFO("odom_frame: %s", odom_frame_.c_str());
  RCUTILS_LOG_INFO("odometry_rate: %.1f", odometry_rate_);
  RCUTILS_LOG_INFO("publish_joint_states: %s", publish_joint_states_ ? "true" : "false");
  RCUTILS_LOG_INFO("publish_odom: %s", publish_odom_ ? "true" : "false");
  RCUTILS_LOG_INFO("publish_tf: %s", publish_tf_ ? "true" : "false");
  RCUTILS_LOG_INFO("status_rate: %.1f", status_rate_);
  RCUTILS_LOG_INFO("wheel_radius: %.3f", wheel_radius_);
  RCUTILS_LOG_INFO("wheel_separation: %.3f", wheel_separation_);
  RCUTILS_LOG_INFO("===================================");

  // Initialize RoboClaw hardware interface with dummy current limits (0.0, 0.0)
  // Current limiting is handled by RoboClaw firmware, not this driver
  roboclaw_ = std::make_unique<RoboClaw>(0.0, 0.0, device_name_, address_, baud_rate_, do_debug_,
                                         do_low_level_debug_);

  if (!initialize_roboclaw()) {
    RCUTILS_LOG_ERROR("Failed to initialize RoboClaw driver");
    return;
  }

  // Initialize ROS2 publishers and subscribers based on configuration
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&RoboClawDriverNode::cmd_vel_callback, this, std::placeholders::_1));

  // Create publishers conditionally based on configuration
  if (publish_odom_) {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // TF broadcaster requires odometry to be enabled
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  } else if (publish_tf_) {
    // Invalid configuration: TF requires odometry
    RCUTILS_LOG_WARN(
        "publish_tf is enabled but publish_odom is disabled. TF "
        "requires odometry. Disabling TF publishing.");
    publish_tf_ = false;
  }

  if (publish_joint_states_) {
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }

  // Always create status publisher for monitoring robot health
  status_pub_ = this->create_publisher<std_msgs::msg::String>("roboclaw_status", 10);

  // Initialize timing variables for publishing rate control
  auto now = this->get_clock()->now();
  last_odom_time_ = now;
  last_odometry_publish_ = now;
  last_joint_states_publish_ = now;
  last_status_publish_ = now;

  // Start main control loop at fixed frequency (TeensyV2 compatible)
  auto timer_period = std::chrono::duration<double>(1.0 / MAIN_LOOP_FREQUENCY);
  main_timer_ =
      this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
                              std::bind(&RoboClawDriverNode::main_loop, this));

  RCUTILS_LOG_INFO("RoboClaw driver node initialized successfully");
}

RoboClawDriverNode::~RoboClawDriverNode() {
  if (roboclaw_) {
    // Stop motors on shutdown
    roboclaw_->stop();
  }
}

/**
 * @brief Initialize communication with RoboClaw controller
 *
 * Establishes serial connection, reads firmware version, configures PID parameters,
 * and resets encoder values. This must be called before normal operation.
 *
 * @return true if initialization successful, false otherwise
 */
bool RoboClawDriverNode::initialize_roboclaw() {
  RCUTILS_LOG_INFO("Connected to RoboClaw on %s at %d baud", device_name_.c_str(), baud_rate_);

  // Read and display firmware version for verification
  std::string version;
  CmdReadFirmwareVersion cmd(*roboclaw_, version);
  cmd.execute();

  RCUTILS_LOG_INFO("RoboClaw Firmware Version: %s", version.c_str());

  // Configure PID parameters for both motors
  // These values control how the motors respond to speed commands
  CmdSetPid command_m1_pid(*roboclaw_, RoboClaw::kM1, m1_p_, m1_i_, m1_d_, m1_qpps_);
  command_m1_pid.execute();
  CmdSetPid command_m2_pid(*roboclaw_, RoboClaw::kM2, m2_p_, m2_i_, m2_d_, m2_qpps_);
  command_m2_pid.execute();

  // Reset encoder counts to establish known starting position
  CmdSetEncoderValue m1(*roboclaw_, RoboClaw::kM1, 0);
  m1.execute();
  CmdSetEncoderValue m2(*roboclaw_, RoboClaw::kM2, 0);
  m2.execute();

  motors_initialized_ = true;
  return true;
}

/**
 * @brief Main control loop executed at fixed frequency
 *
 * Handles velocity commands, reads sensors, calculates odometry, and publishes
 * data at configured rates. Follows TeensyV2 timing patterns for compatibility.
 */
void RoboClawDriverNode::main_loop() {
  auto now = this->get_clock()->now();

  // Process velocity commands and send motor commands
  handle_cmd_vel();

  // Read sensor data from RoboClaw and update internal state
  read_sensors();

  // Publish odometry data if enabled and rate limit reached
  if (publish_odom_) {
    double odometry_dt = (now - last_odometry_publish_).seconds();
    if (odometry_dt >= (1.0 / odometry_rate_)) {
      publish_odometry();
      last_odometry_publish_ = now;
    }
  }

  // Publish joint states if enabled and rate limit reached
  if (publish_joint_states_) {
    double joint_dt = (now - last_joint_states_publish_).seconds();
    if (joint_dt >= (1.0 / joint_states_rate_)) {
      publish_joint_states();
      last_joint_states_publish_ = now;
    }
  }

  // Always publish status information at configured rate
  double status_dt = (now - last_status_publish_).seconds();
  if (status_dt >= (1.0 / status_rate_)) {
    publish_status();
    last_status_publish_ = now;
  }
}

/**
 * @brief Callback for incoming velocity commands
 *
 * Thread-safe storage of cmd_vel messages with timestamp and sequence tracking.
 * Commands are processed in the main loop to maintain timing consistency.
 *
 * @param msg Twist message containing linear and angular velocity commands
 */
void RoboClawDriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  static rclcpp::Time time_of_last_cmd_vel = this->get_clock()->now();

  // Thread-safe update of command cache
  std::lock_guard<std::mutex> lock(last_cmd_vel_.mutex);
  last_cmd_vel_.cmd_vel = *msg;
  last_cmd_vel_.sequence_number++;
  last_cmd_vel_.timestamp = this->get_clock()->now();
}

/**
 * @brief Process cached velocity commands and send to motors
 *
 * Converts twist commands to motor speeds, applies safety limits, and sends
 * buffered commands to RoboClaw. Only processes new commands to avoid redundancy.
 */
void RoboClawDriverNode::handle_cmd_vel() {
  static uint32_t last_sequence_number = 0;

  std::lock_guard<std::mutex> lock(last_cmd_vel_.mutex);

  // Skip if no new command received
  if (last_cmd_vel_.sequence_number <= last_sequence_number) {
    return;
  }

  last_sequence_number = last_cmd_vel_.sequence_number;

  // Convert twist to individual motor speeds
  int32_t target_left_speed;
  int32_t target_right_speed;
  convert_twist_to_motor_speeds(last_cmd_vel_.cmd_vel, target_left_speed, target_right_speed);

  // Calculate maximum distance for safety timeout
  // Motors will stop after traveling this distance without new commands
  const int32_t m1_max_distance_quad_pulses =
      (int32_t)fabs(target_left_speed * max_seconds_uncommanded_travel_);
  const int32_t m2_max_distance_quad_pulses =
      (int32_t)fabs(target_right_speed * max_seconds_uncommanded_travel_);

  // Send buffered command with acceleration and distance limits
  CmdDoBufferedM1M2DriveSpeedAccelDistance cmd(*roboclaw_, accel_, target_left_speed,
                                               m1_max_distance_quad_pulses, target_right_speed,
                                               m2_max_distance_quad_pulses);
  cmd.execute();

  // Log timing information if debug enabled
  if (do_debug_) {
    rclcpp::Time now = this->get_clock()->now();
    double lag_time = (now - last_cmd_vel_.timestamp).seconds();
    RCUTILS_LOG_INFO("lag_time=%.3f, sequence=%u", lag_time, last_cmd_vel_.sequence_number);
  }
}

/**
 * @brief Convert ROS twist commands to individual motor speeds
 *
 * Implements differential drive kinematics to convert linear and angular
 * velocities into left and right wheel speeds. Applies safety velocity limits.
 *
 * @param twist Input velocity command (linear.x and angular.z used)
 * @param left_speed Output left motor speed in quadrature pulses per second
 * @param right_speed Output right motor speed in quadrature pulses per second
 */
void RoboClawDriverNode::convert_twist_to_motor_speeds(const geometry_msgs::msg::Twist& twist,
                                                       int32_t& left_speed, int32_t& right_speed) {
  // Apply safety limits to input velocities
  double linear_vel = std::clamp(twist.linear.x, -max_linear_velocity_, max_linear_velocity_);
  double angular_vel = std::clamp(twist.angular.z, -max_angular_velocity_, max_angular_velocity_);

  // Differential drive kinematics: convert robot velocities to wheel velocities
  double left_wheel_vel = linear_vel - (angular_vel * wheel_separation_ / 2.0);
  double right_wheel_vel = linear_vel + (angular_vel * wheel_separation_ / 2.0);

  // Convert wheel velocities (m/s) to encoder counts per second (QPPS)
  double left_qpps =
      (left_wheel_vel / (2.0 * M_PI * wheel_radius_)) * encoder_counts_per_revolution_;
  double right_qpps =
      (right_wheel_vel / (2.0 * M_PI * wheel_radius_)) * encoder_counts_per_revolution_;

  // Convert to integer speeds for RoboClaw commands
  left_speed = static_cast<int32_t>(left_qpps);
  right_speed = static_cast<int32_t>(right_qpps);
}

bool RoboClawDriverNode::get_fresh_encoders(RoboClaw::EncodeResult& enc1,
                                            RoboClaw::EncodeResult& enc2) {
  if (!motors_initialized_) {
    return false;
  }

  CmdReadEncoder cmd1(*roboclaw_, RoboClaw::kM1, enc1);
  CmdReadEncoder cmd2(*roboclaw_, RoboClaw::kM2, enc2);

  cmd1.execute();
  cmd2.execute();
  return true;
}

/**
 * @brief Read sensor data from RoboClaw using distributed state machine
 *
 * Implements TeensyV2-compatible sensor reading pattern that distributes
 * different sensor readings across multiple control cycles to prevent
 * overwhelming the serial communication channel.
 */
void RoboClawDriverNode::read_sensors() {
  // Always get fresh encoder readings for real-time odometry
  get_fresh_encoders(roboclaw_state_.m1_enc_result, roboclaw_state_.m2_enc_result);

  // Distribute status readings across cycles to avoid overwhelming serial communication
  // This follows the TeensyV2 pattern of state machine for different readings
  auto now = this->get_clock()->now();
  static auto last_status_time = now;

  // Rate limit status readings to 10Hz to prevent serial overload
  if ((now - last_status_time).seconds() >= 0.1) {
    switch (current_status_state_) {
      case READ_BATTERY: {
        // Read both logic and main battery voltages
        CmdReadLogicBatteryVoltage cmd_logic_batt(*roboclaw_,
                                                  roboclaw_state_.logic_battery_voltage);
        cmd_logic_batt.execute();
        CmdReadMainBatteryVoltage cmd_main_batt(*roboclaw_, roboclaw_state_.main_battery_voltage);
        cmd_main_batt.execute();
      } break;

      case READ_TEMPERATURES: {
        // Read temperature sensors (if available on your RoboClaw model)
        CmdReadTemperature cmd_temp1(*roboclaw_, RoboClaw::kTemperature1,
                                     roboclaw_state_.temperature1);
        cmd_temp1.execute();
        CmdReadTemperature cmd_temp2(*roboclaw_, RoboClaw::kTemperature2,
                                     roboclaw_state_.temperature2);
        cmd_temp2.execute();
      } break;

      case READ_CURRENTS: {
        // Read motor current consumption for monitoring
        CmdReadMotorCurrents cmd_currents(*roboclaw_, roboclaw_state_.motorCurrents);
        cmd_currents.execute();
      } break;

      case READ_ERROR: {
        // Read error status register for fault detection
        CmdReadStatus cmd_status(*roboclaw_, roboclaw_state_.error_status);
        cmd_status.execute();
      } break;

      case READ_SPEEDS: {
        // Read actual motor speeds for feedback
        CmdReadEncoderSpeed cmd_speed_m1(*roboclaw_, RoboClaw::kM1, roboclaw_state_.m1_speed);
        cmd_speed_m1.execute();
        CmdReadEncoderSpeed cmd_speed_m2(*roboclaw_, RoboClaw::kM2, roboclaw_state_.m2_speed);
        cmd_speed_m2.execute();
      } break;
    }

    // Cycle through all sensor reading states
    current_status_state_ = static_cast<StatusReadState>((current_status_state_ + 1) % 5);
    last_status_time = now;
  }
}

/**
 * @brief Calculate robot odometry from encoder readings
 *
 * Implements standard differential drive odometry calculation using encoder
 * deltas to estimate robot position and velocity. Updates global pose estimates.
 */
void RoboClawDriverNode::calculate_odometry() {
  auto now = this->get_clock()->now();

  // Skip calculation on first reading to establish baseline
  if (first_encoder_reading_) {
    last_odom_time_ = now;
    first_encoder_reading_ = false;
    return;
  }

  // Calculate time delta - skip if too small to avoid numerical issues
  double dt = (now - last_odom_time_).seconds();
  if (dt < 0.01) {
    return;
  }

  // Calculate encoder deltas since last reading
  static uint32_t prev_enc1 = roboclaw_state_.m1_enc_result.value;
  static uint32_t prev_enc2 = roboclaw_state_.m2_enc_result.value;

  // Handle encoder wraparound using signed arithmetic
  int32_t delta_enc1 = static_cast<int32_t>(roboclaw_state_.m1_enc_result.value - prev_enc1);
  int32_t delta_enc2 = static_cast<int32_t>(roboclaw_state_.m2_enc_result.value - prev_enc2);

  prev_enc1 = roboclaw_state_.m1_enc_result.value;
  prev_enc2 = roboclaw_state_.m2_enc_result.value;

  // Convert encoder counts to linear distances (meters)
  double delta_left = (delta_enc1 * M_PI * wheel_radius_ * 2.0f) / encoder_counts_per_revolution_;
  double delta_right = (delta_enc2 * M_PI * wheel_radius_ * 2.0f) / encoder_counts_per_revolution_;

  // Differential drive odometry calculations
  double delta_distance = (delta_left + delta_right) / 2.0;             // Forward distance
  double delta_theta = (delta_right - delta_left) / wheel_separation_;  // Angular change

  // Update robot pose using differential drive kinematics
  double delta_x = delta_distance * cos(theta_ + delta_theta / 2.0);
  double delta_y = delta_distance * sin(theta_ + delta_theta / 2.0);

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;
  theta_ = normalize_angle(theta_);  // Keep angle in [-π, π] range

  // Calculate velocities for this time step
  linear_velocity_ = delta_distance / dt;
  angular_velocity_ = delta_theta / dt;

  last_odom_time_ = now;
}

void RoboClawDriverNode::publish_odometry() {
  calculate_odometry();
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  // Position
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;

  // Orientation
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom_msg.pose.pose.orientation = tf2::toMsg(q);

  // Velocity
  odom_msg.twist.twist.linear.x = linear_velocity_;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_velocity_;

  // Covariance (simplified)
  const double position_cov = 0.1;
  const double orientation_cov = 0.1;
  const double velocity_cov = 0.1;

  odom_msg.pose.covariance[0] = position_cov;      // x
  odom_msg.pose.covariance[7] = position_cov;      // y
  odom_msg.pose.covariance[35] = orientation_cov;  // yaw

  odom_msg.twist.covariance[0] = velocity_cov;   // vx
  odom_msg.twist.covariance[35] = velocity_cov;  // vyaw

  odom_pub_->publish(odom_msg);

  // Publish transform
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom_msg.header.stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
  }
}

void RoboClawDriverNode::publish_joint_states() {
  if (!joint_states_pub_) return;

  auto joint_msg = sensor_msgs::msg::JointState();
  joint_msg.header.stamp = this->get_clock()->now();

  joint_msg.name = {"left_wheel_joint", "right_wheel_joint"};

  // Convert encoder counts to radians
  double left_angle =
      (2.0 * M_PI * roboclaw_state_.m1_enc_result.value) / encoder_counts_per_revolution_;
  double right_angle =
      (2.0 * M_PI * roboclaw_state_.m2_enc_result.value) / encoder_counts_per_revolution_;

  joint_msg.position = {left_angle, right_angle};

  // Wheel velocities in rad/s
  double left_vel =
      (linear_velocity_ - angular_velocity_ * wheel_separation_ / 2.0) / wheel_radius_;
  double right_vel =
      (linear_velocity_ + angular_velocity_ * wheel_separation_ / 2.0) / wheel_radius_;

  joint_msg.velocity = {left_vel, right_vel};

  joint_states_pub_->publish(joint_msg);
}

void RoboClawDriverNode::publish_status() {
  char decoded_error_status[256];
  decodeErrorStatus(roboclaw_state_.error_status, decoded_error_status,
                    sizeof(decoded_error_status));

  // Create JSON string with RoboClaw state data
  std::ostringstream error_status_hex;
  error_status_hex << std::setw(8) << std::setfill('0') << std::hex << std::uppercase
                   << roboclaw_state_.error_status;

  // Get current ROS2 time as seconds.nanoseconds (like header.stamp)
  auto stamp = this->get_clock()->now();

  // Format timestamp as "seconds.nanoseconds" (e.g., "1758159135.361360569")
  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(9)
                   << stamp.seconds() + (stamp.nanoseconds() % 1000000000) / 1e9;

  std::string json_status =
      "{\"timestamp\":\"" + timestamp_stream.str() +
      "\",\"m1_current\":" + std::to_string(roboclaw_state_.motorCurrents.m1Current) +
      ",\"m2_current\":" + std::to_string(roboclaw_state_.motorCurrents.m2Current) +
      ",\"m1_speed\":" + std::to_string(roboclaw_state_.m1_speed) +
      ",\"m1_enc_value\":" + std::to_string(roboclaw_state_.m1_enc_result.value) +
      ",\"m1_enc_status\":" +
      std::to_string(static_cast<int>(roboclaw_state_.m1_enc_result.status)) +
      ",\"m2_speed\":" + std::to_string(roboclaw_state_.m2_speed) +
      ",\"m2_enc_value\":" + std::to_string(roboclaw_state_.m2_enc_result.value) +
      ",\"m2_enc_status\":" +
      std::to_string(static_cast<int>(roboclaw_state_.m2_enc_result.status)) +
      ",\"logic_battery\":" + std::to_string(roboclaw_state_.logic_battery_voltage) +
      ",\"main_battery\":" + std::to_string(roboclaw_state_.main_battery_voltage) +
      ", \"temperature1\":" + std::to_string(roboclaw_state_.temperature1) +
      ", \"temperature2\":" + std::to_string(roboclaw_state_.temperature2) +
      ", \"error_status\":\"" + error_status_hex.str() + "\"" + ", \"decoded_error_status\":\"" +
      std::string(decoded_error_status) + "\"}";

  // RCUTILS_LOG_INFO( "RoboClaw Status: %s",
  // json_status.c_str());
  status_pub_->publish(std_msgs::msg::String().set__data(json_status));
}

double RoboClawDriverNode::normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void RoboClawDriverNode::declare_parameters() {
  // Declare all parameters alphabetically with proper default values
  // This ensures consistent parameter handling and avoids truncation issues

  this->declare_parameter("accel", 3000);
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("baud_rate", 230400);  // Match config file default
  this->declare_parameter("device_name",
                          "/dev/ttyAMA0");  // Match config file default
  this->declare_parameter("device_timeout", 100);
  this->declare_parameter("do_debug", false);
  this->declare_parameter("do_low_level_debug", false);
  this->declare_parameter("encoder_counts_per_revolution",
                          1000);  // Match config file default
  this->declare_parameter("joint_states_rate", 30.0);
  this->declare_parameter("m1_d", 0.0);
  this->declare_parameter("m1_i", 2.43);
  this->declare_parameter("m1_p", 7.26239);
  this->declare_parameter("m1_qpps", 2437);
  this->declare_parameter("m2_d", 0.0);
  this->declare_parameter("m2_i", 2.43);
  this->declare_parameter("m2_p", 7.26239);
  this->declare_parameter("m2_qpps", 2437);
  this->declare_parameter("max_angular_velocity",
                          0.07);  // Match config file default
  this->declare_parameter("max_linear_velocity",
                          0.3);  // Match config file default
  this->declare_parameter("max_seconds_uncommanded_travel",
                          0.2);  // Match config file default
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("odometry_rate", 67.0);
  this->declare_parameter("publish_joint_states",
                          false);                  // Match config file default
  this->declare_parameter("publish_odom", false);  // Match config file default
  this->declare_parameter("publish_tf", false);    // Match config file default
  this->declare_parameter("status_rate", 10.0);
  this->declare_parameter("wheel_radius",
                          0.051112072);  // Match config file default
  this->declare_parameter("wheel_separation",
                          0.3906);  // Match config file default
}

void RoboClawDriverNode::load_parameters() {
  // Load all parameters using get_parameter_or for robust error handling
  // This prevents truncation and type conversion issues

  accel_ = this->get_parameter_or("accel", 3000);
  base_frame_ = this->get_parameter_or("base_frame", std::string("base_link"));
  baud_rate_ = this->get_parameter_or("baud_rate", 230400);
  device_name_ = this->get_parameter_or("device_name", std::string("/dev/ttyAMA0"));
  device_timeout_ = this->get_parameter_or("device_timeout", 100);
  do_debug_ = this->get_parameter_or("do_debug", false);
  do_low_level_debug_ = this->get_parameter_or("do_low_level_debug", false);
  encoder_counts_per_revolution_ = this->get_parameter_or("encoder_counts_per_revolution", 1000);
  joint_states_rate_ = this->get_parameter_or("joint_states_rate", 30.0);
  m1_d_ = this->get_parameter_or("m1_d", 0.0);
  m1_i_ = this->get_parameter_or("m1_i", 2.43);
  m1_p_ = this->get_parameter_or("m1_p", 7.26239);
  m1_qpps_ = this->get_parameter_or("m1_qpps", static_cast<uint32_t>(2437));
  m2_d_ = this->get_parameter_or("m2_d", 0.0);
  m2_i_ = this->get_parameter_or("m2_i", 2.43);
  m2_p_ = this->get_parameter_or("m2_p", 7.26239);
  m2_qpps_ = this->get_parameter_or("m2_qpps", static_cast<uint32_t>(2437));
  max_angular_velocity_ = this->get_parameter_or("max_angular_velocity", 0.07);
  max_linear_velocity_ = this->get_parameter_or("max_linear_velocity", 0.3);
  max_seconds_uncommanded_travel_ = this->get_parameter_or("max_seconds_uncommanded_travel", 0.2);
  odom_frame_ = this->get_parameter_or("odom_frame", std::string("odom"));
  odometry_rate_ = this->get_parameter_or("odometry_rate", 67.0);
  publish_joint_states_ = this->get_parameter_or("publish_joint_states", false);
  publish_odom_ = this->get_parameter_or("publish_odom", false);
  publish_tf_ = this->get_parameter_or("publish_tf", false);
  status_rate_ = this->get_parameter_or("status_rate", 10.0);
  wheel_radius_ = this->get_parameter_or("wheel_radius", 0.051112072);
  wheel_separation_ = this->get_parameter_or("wheel_separation", 0.3906);
}

void RoboClawDriverNode::log_parameters() {
  // Log all parameters for debugging and verification
  // This helps identify parameter loading issues and truncation problems

  if (do_debug_) {
    RCUTILS_LOG_INFO("=== RoboClaw Driver Parameters ===");

    // Communication parameters
    RCUTILS_LOG_INFO("Communication:");
    RCUTILS_LOG_INFO("  device_name: %s", device_name_.c_str());
    RCUTILS_LOG_INFO("  baud_rate: %d", baud_rate_);
    RCUTILS_LOG_INFO("  device_timeout: %d ms", device_timeout_);

    // Physical parameters
    RCUTILS_LOG_INFO("Robot Physical:");
    RCUTILS_LOG_INFO("  wheel_radius: %.6f m", wheel_radius_);
    RCUTILS_LOG_INFO("  wheel_separation: %.6f m", wheel_separation_);
    RCUTILS_LOG_INFO("  encoder_counts_per_revolution: %d", encoder_counts_per_revolution_);
    RCUTILS_LOG_INFO("  accel: %u quad pulses/s²", accel_);

    // Safety parameters
    RCUTILS_LOG_INFO("Safety:");
    RCUTILS_LOG_INFO("  max_linear_velocity: %.3f m/s", max_linear_velocity_);
    RCUTILS_LOG_INFO("  max_angular_velocity: %.3f rad/s", max_angular_velocity_);
    RCUTILS_LOG_INFO("  max_seconds_uncommanded_travel: %.3f s", max_seconds_uncommanded_travel_);

    // PID parameters (high precision logging to catch truncation)
    RCUTILS_LOG_INFO("Motor 1 PID:");
    RCUTILS_LOG_INFO("  m1_p: %.8f", m1_p_);
    RCUTILS_LOG_INFO("  m1_i: %.8f", m1_i_);
    RCUTILS_LOG_INFO("  m1_d: %.8f", m1_d_);
    RCUTILS_LOG_INFO("  m1_qpps: %u", m1_qpps_);

    RCUTILS_LOG_INFO("Motor 2 PID:");
    RCUTILS_LOG_INFO("  m2_p: %.8f", m2_p_);
    RCUTILS_LOG_INFO("  m2_i: %.8f", m2_i_);
    RCUTILS_LOG_INFO("  m2_d: %.8f", m2_d_);
    RCUTILS_LOG_INFO("  m2_qpps: %u", m2_qpps_);

    // Publishing parameters
    RCUTILS_LOG_INFO("Publishing:");
    RCUTILS_LOG_INFO("  publish_odom: %s", publish_odom_ ? "true" : "false");
    RCUTILS_LOG_INFO("  publish_tf: %s", publish_tf_ ? "true" : "false");
    RCUTILS_LOG_INFO("  publish_joint_states: %s", publish_joint_states_ ? "true" : "false");
    RCUTILS_LOG_INFO("  odometry_rate: %.1f Hz", odometry_rate_);
    RCUTILS_LOG_INFO("  joint_states_rate: %.1f Hz", joint_states_rate_);
    RCUTILS_LOG_INFO("  status_rate: %.1f Hz", status_rate_);

    // Frame parameters
    RCUTILS_LOG_INFO("Frames:");
    RCUTILS_LOG_INFO("  base_frame: %s", base_frame_.c_str());
    RCUTILS_LOG_INFO("  odom_frame: %s", odom_frame_.c_str());

    // Debug parameters
    RCUTILS_LOG_INFO("Debug:");
    RCUTILS_LOG_INFO("  do_debug: %s", do_debug_ ? "true" : "false");
    RCUTILS_LOG_INFO("  do_low_level_debug: %s", do_low_level_debug_ ? "true" : "false");

    RCUTILS_LOG_INFO("=== End Parameters ===");
  }
}

void RoboClawDriverNode::decodeErrorStatus(uint32_t error_status, char* buffer, size_t size) const {
  if (error_status == 0) {
    strncpy(buffer, "No errors", size);
    buffer[size - 1] = '\0';
    return;
  }

  buffer[0] = '\0';
  size_t current_len = 0;
  bool first = true;

  auto append_error = [&](const char* error_str) {
    if (current_len >= size - 1) return;
    size_t needed = strlen(error_str) + (first ? 0 : 2);
    if (current_len + needed < size) {
      if (!first) {
        strcat(buffer, ", ");
        current_len += 2;
      }
      strcat(buffer, error_str);
      current_len += strlen(error_str);
      first = false;
    }
  };

  // Check error flags (bits 0-15)
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_ESTOP))
    append_error("ERROR_ESTOP");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_TEMP))
    append_error("ERROR_TEMP");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_TEMP2))
    append_error("ERROR_TEMP2");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_LBATHIGH))
    append_error("ERROR_LBATHIGH");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_LBATLOW))
    append_error("ERROR_LBATLOW");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_FAULTM1))
    append_error("ERROR_FAULTM1");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_FAULTM2))
    append_error("ERROR_FAULTM2");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_SPEED1))
    append_error("ERROR_SPEED1");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_SPEED2))
    append_error("ERROR_SPEED2");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_POS1))
    append_error("ERROR_POS1");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_POS2))
    append_error("ERROR_POS2");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_CURRENTM1))
    append_error("ERROR_CURRENTM1");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_CURRENTM2))
    append_error("ERROR_CURRENTM2");

  // Check warning flags (bits 16-31)
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_OVERCURRENTM1))
    append_error("WARN_OVERCURRENTM1");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_OVERCURRENTM2))
    append_error("WARN_OVERCURRENTM2");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_MBATHIGH))
    append_error("WARN_MBATHIGH");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_MBATLOW))
    append_error("WARN_MBATLOW");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_TEMP))
    append_error("WARN_TEMP");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_TEMP2))
    append_error("WARN_TEMP2");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_S4))
    append_error("WARN_S4");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_S5))
    append_error("WARN_S5");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_CAN))
    append_error("WARN_CAN");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_BOOT))
    append_error("WARN_BOOT");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_OVERREGENM1))
    append_error("WARN_OVERREGENM1");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_OVERREGENM2))
    append_error("WARN_OVERREGENM2");

  // Report any unknown bits
  uint32_t known_errors = 0xF000EFFF;  // All defined error and warning bits
  uint32_t unknown_errors = error_status & ~known_errors;
  if (unknown_errors != 0) {
    if (current_len < size - 1 && !first) {
      strcat(buffer, ", ");
      current_len += 2;
    }
    snprintf(buffer + current_len, size - current_len, "UNKNOWN:0x%X",
             (unsigned int)unknown_errors);
  }
}
