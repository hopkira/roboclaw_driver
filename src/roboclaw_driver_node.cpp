#include "roboclaw_driver/roboclaw_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sstream>

#include "roboclaw_driver/roboclaw_cmd_do_buffered_m1m2_drive_speed_accel_distance.h"
#include "roboclaw_driver/roboclaw_cmd_read_buffer_length.h"
#include "roboclaw_driver/roboclaw_cmd_read_firmware_version.h"
#include "roboclaw_driver/roboclaw_cmd_read_status.h"

using namespace std::chrono_literals;

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
      motors_initialized_(false),
      encoder_init_done_(false),
      encoder_retries_(0) {
  // Declare parameters
  this->declare_parameter("accel", 3000);
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("baud_rate", 38400);
  this->declare_parameter("device_name", "/dev/roboclaw");
  this->declare_parameter("device_timeout", 100);
  this->declare_parameter("encoder_counts_per_revolution", 0);
  this->declare_parameter("joint_states_rate", 30.0);
  this->declare_parameter("max_m1_current", 0.0);
  this->declare_parameter("max_m2_current", 0.0);
  this->declare_parameter("max_seconds_uncommanded_travel", 0.05);
  this->declare_parameter("max_angular_velocity", 0.0);
  this->declare_parameter("max_linear_velocity", 0.0);
  this->declare_parameter("max_retries", 3);
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("odometry_rate", 67.0);
  this->declare_parameter("status_rate", 10.0);
  this->declare_parameter("wheel_radius", 0.0);
  this->declare_parameter("wheel_separation", 0.0);

  {
    rcl_interfaces::msg::ParameterDescriptor pd;
    pd.dynamic_typing = true;
    this->declare_parameter("m1_p", rclcpp::ParameterValue(7.26239), pd);
    this->declare_parameter("m1_i", rclcpp::ParameterValue(2.43), pd);
    this->declare_parameter("m1_d", rclcpp::ParameterValue(0.0), pd);
    this->declare_parameter("m1_qpps", rclcpp::ParameterValue(2437), pd);

    this->declare_parameter("m2_p", rclcpp::ParameterValue(7.26239), pd);
    this->declare_parameter("m2_i", rclcpp::ParameterValue(2.43), pd);
    this->declare_parameter("m2_d", rclcpp::ParameterValue(0.0), pd);
    this->declare_parameter("m2_qpps", rclcpp::ParameterValue(2437), pd);
  }

  this->declare_parameter("cmd_vel_timeout", 1.0);

  this->declare_parameter("publish_odom", true);
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("publish_joint_states", true);

  this->declare_parameter("do_debug", false);
  this->declare_parameter("do_low_level_debug", false);

  // Get parameters
  device_name_ = this->get_parameter("device_name").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  device_timeout_ = this->get_parameter("device_timeout").as_int();
  max_seconds_uncommanded_travel_ =
      this->get_parameter("max_seconds_uncommanded_travel").as_double();
  max_retries_ = this->get_parameter("max_retries").as_int();

  odometry_rate_ = this->get_parameter("odometry_rate").as_double();
  joint_states_rate_ = this->get_parameter("joint_states_rate").as_double();
  status_rate_ = this->get_parameter("status_rate").as_double();

  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  wheel_separation_ = this->get_parameter("wheel_separation").as_double();
  encoder_counts_per_revolution_ =
      this->get_parameter("encoder_counts_per_revolution").as_int();

  base_frame_ = this->get_parameter("base_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();

  max_angular_velocity_ =
      this->get_parameter("max_angular_velocity").as_double();
  max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();

  max_m1_current_ = this->get_parameter("max_m1_current").as_double();
  max_m2_current_ = this->get_parameter("max_m2_current").as_double();

  // Helper to read numeric parameter as double regardless of integer/double
  // YAML types
  auto get_num = [this](const std::string& name, double def) {
    rclcpp::Parameter p;
    if (!this->get_parameter(name, p)) return def;
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return p.as_double();
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<double>(p.as_int());
      default:
        return def;
    }
  };

  // Read PID parameters as floats and convert to fixed-point (TeensyV2
  // compatibility)
  double m1_p_float = get_num("m1_p", 7.26239);
  double m1_i_float = get_num("m1_i", 2.43);
  double m1_d_float = get_num("m1_d", 0.0);
  m1_p_ = static_cast<uint32_t>(m1_p_float * 65536.0);
  m1_i_ = static_cast<uint32_t>(m1_i_float * 65536.0);
  m1_d_ = static_cast<uint32_t>(m1_d_float * 65536.0);
  m1_qpps_ = static_cast<int>(get_num("m1_qpps", 2437));

  double m2_p_float = get_num("m2_p", 7.26239);
  double m2_i_float = get_num("m2_i", 2.43);
  double m2_d_float = get_num("m2_d", 0.0);
  m2_p_ = static_cast<uint32_t>(m2_p_float * 65536.0);
  m2_i_ = static_cast<uint32_t>(m2_i_float * 65536.0);
  m2_d_ = static_cast<uint32_t>(m2_d_float * 65536.0);
  m2_qpps_ = static_cast<int>(get_num("m2_qpps", 2437));

  accel_ = this->get_parameter("accel").as_int();

  cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();

  publish_odom_ = this->get_parameter("publish_odom").as_bool();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  publish_joint_states_ = this->get_parameter("publish_joint_states").as_bool();

  do_debug_ = this->get_parameter("do_debug").as_bool();
  do_low_level_debug_ = this->get_parameter("do_low_level_debug").as_bool();

  // Log all parameters alphabetically
  RCLCPP_INFO(this->get_logger(), "=== RoboClaw Driver Parameters ===");
  RCLCPP_INFO(this->get_logger(), "accel: %d", accel_);
  RCLCPP_INFO(this->get_logger(), "base_frame: %s", base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "baud_rate: %d", baud_rate_);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_timeout: %.1f", cmd_vel_timeout_);
  RCLCPP_INFO(this->get_logger(), "do_debug: %s", do_debug_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "do_low_level_debug: %s",
              do_low_level_debug_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "device_name: %s", device_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "device_timeout: %d", device_timeout_);
  RCLCPP_INFO(this->get_logger(), "encoder_counts_per_revolution: %d",
              encoder_counts_per_revolution_);
  RCLCPP_INFO(this->get_logger(), "joint_states_rate: %.1f",
              joint_states_rate_);
  RCLCPP_INFO(this->get_logger(), "m1_d: %.6f -> %u", m1_d_float, m1_d_);
  RCLCPP_INFO(this->get_logger(), "m1_i: %.6f -> %u", m1_i_float, m1_i_);
  RCLCPP_INFO(this->get_logger(), "m1_p: %.6f -> %u", m1_p_float, m1_p_);
  RCLCPP_INFO(this->get_logger(), "m1_qpps: %d", m1_qpps_);
  RCLCPP_INFO(this->get_logger(), "m2_d: %.6f -> %u", m2_d_float, m2_d_);
  RCLCPP_INFO(this->get_logger(), "m2_i: %.6f -> %u", m2_i_float, m2_i_);
  RCLCPP_INFO(this->get_logger(), "m2_p: %.6f -> %u", m2_p_float, m2_p_);
  RCLCPP_INFO(this->get_logger(), "m2_qpps: %d", m2_qpps_);
  RCLCPP_INFO(this->get_logger(), "max_angular_velocity: %.1f",
              max_angular_velocity_);
  RCLCPP_INFO(this->get_logger(), "max_linear_velocity: %.1f",
              max_linear_velocity_);
  RCLCPP_INFO(this->get_logger(), "max_m1_current: %.1f", max_m1_current_);
  RCLCPP_INFO(this->get_logger(), "max_m2_current: %.1f", max_m2_current_);
  RCLCPP_INFO(this->get_logger(), "max_retries: %d", max_retries_);
  RCLCPP_INFO(this->get_logger(), "max_seconds_uncommanded_travel: %.3f",
              max_seconds_uncommanded_travel_);
  RCLCPP_INFO(this->get_logger(), "odom_frame: %s", odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "odometry_rate: %.1f", odometry_rate_);
  RCLCPP_INFO(this->get_logger(), "publish_joint_states: %s",
              publish_joint_states_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "publish_odom: %s",
              publish_odom_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s",
              publish_tf_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "status_rate: %.1f", status_rate_);
  RCLCPP_INFO(this->get_logger(), "wheel_radius: %.3f", wheel_radius_);
  RCLCPP_INFO(this->get_logger(), "wheel_separation: %.3f", wheel_separation_);
  RCLCPP_INFO(this->get_logger(), "===================================");

  // Initialize RoboClaw
  RoboClaw::TPIDQ m1_pid = {m1_p_float, m1_i_float, m1_d_float, m1_qpps_, 0.0f};
  RoboClaw::TPIDQ m2_pid = {m2_p_float, m2_i_float, m2_d_float, m2_qpps_, 0.0f};
  roboclaw_ = std::make_unique<RoboClaw>(
      m1_pid, m2_pid, max_m1_current_, max_m2_current_, device_name_, address_,
      baud_rate_, do_debug_, do_low_level_debug_);

  if (!initialize_roboclaw()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize RoboClaw driver");
    return;
  }

  // Initialize ROS2 interface
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1,
      std::bind(&RoboClawDriverNode::cmd_vel_callback, this,
                std::placeholders::_1));

  // Create publishers based on configuration
  if (publish_odom_) {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Only create TF broadcaster if both odom and tf are enabled
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  } else if (publish_tf_) {
    RCLCPP_WARN(this->get_logger(),
                "publish_tf is enabled but publish_odom is disabled. TF "
                "requires odometry. Disabling TF publishing.");
    publish_tf_ = false;
  }

  if (publish_joint_states_) {
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
  }

  // Initialize timing
  auto now = this->get_clock()->now();
  last_odom_time_ = now;
  last_odometry_publish_ = now;
  last_joint_states_publish_ = now;
  last_status_publish_ = now;

  // Start main loop timer
  auto timer_period = std::chrono::duration<double>(1.0 / MAIN_LOOP_FREQUENCY);
  main_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&RoboClawDriverNode::main_loop, this));

  RCLCPP_INFO(this->get_logger(),
              "RoboClaw driver node initialized successfully");
}

RoboClawDriverNode::~RoboClawDriverNode() {
  if (roboclaw_) {
    // Stop motors on shutdown
    roboclaw_->stop();
  }
}

bool RoboClawDriverNode::initialize_roboclaw() {
  RCLCPP_INFO(this->get_logger(), "Connected to RoboClaw on %s at %d baud",
              device_name_.c_str(), baud_rate_);
  std::string version;
  CmdReadFirmwareVersion cmd(*roboclaw_, version);
  cmd.execute();

  RCLCPP_INFO(this->get_logger(), "RoboClaw Firmware Version: %s",
              version.c_str());

  motors_initialized_ = true;
  return true;
}

void RoboClawDriverNode::main_loop() {
  auto now = this->get_clock()->now();

  // Handle cmd_vel processing and motor commands
  // handle_cmd_vel();
  RCLCPP_INFO(this->get_logger(), "LOOP");
  int32_t target_left_speed;
  int32_t target_right_speed;
  last_cmd_vel_.cmd_vel.angular.z = 0.0;
  last_cmd_vel_.cmd_vel.linear.x = 0.2;
  convert_twist_to_motor_speeds(last_cmd_vel_.cmd_vel, target_left_speed,
                                target_right_speed);

  const int32_t m1_max_distance_quad_pulses =
      (int32_t)fabs(target_left_speed * max_seconds_uncommanded_travel_);
  const int32_t m2_max_distance_quad_pulses =
      (int32_t)fabs(target_right_speed * max_seconds_uncommanded_travel_);
  // CmdDoBufferedM1M2DriveSpeedAccelDistance cmd2(
  //     *roboclaw_, accel_, target_left_speed, m1_max_distance_quad_pulses,
  //     target_right_speed, m2_max_distance_quad_pulses);
  // cmd2.execute();
  target_left_speed = 0;
  target_right_speed = 0;
  roboclaw_->writeN2(10, address_, 37, SetDWORDval(target_left_speed),
                     SetDWORDval(target_right_speed));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));  // ###
  uint32_t current_status;
  CmdReadStatus cmd3(*roboclaw_, current_status);
  cmd3.execute();
  RCLCPP_INFO(this->get_logger(), "Status: 0x%04X", current_status);

  // roboclaw_->writeN2(
  //     23, address_, RoboClaw::MIXEDSPEEDACCELDIST, SetDWORDval(3000),
  //     SetDWORDval(target_left_speed),
  //     SetDWORDval(m1_max_distance_quad_pulses),
  //     SetDWORDval(target_right_speed),
  //     SetDWORDval(m2_max_distance_quad_pulses), 1 /* Cancel any previous
  //     command. */
  // );
  uint8_t m1_buf_len = 0;
  uint8_t m2_buf_len = 0;
  CmdReadBufferLength cmd4(*roboclaw_, m1_buf_len, m2_buf_len);
  cmd4.execute();
  RCLCPP_INFO(this->get_logger(), "M1 buf len: %u, M2 buf len: %u", m1_buf_len,
              m2_buf_len);

  // Read sensor data and update odometry
  // read_sensors();
  // calculate_odometry();

  // // Publishing based on time intervals
  // if (publish_odom_) {
  //   double odometry_dt = (now - last_odometry_publish_).seconds();
  //   if (odometry_dt >= (1.0 / odometry_rate_)) {
  //     publish_odometry();
  //     last_odometry_publish_ = now;
  //   }
  // }

  // if (publish_joint_states_) {
  //   double joint_dt = (now - last_joint_states_publish_).seconds();
  //   if (joint_dt >= (1.0 / joint_states_rate_)) {
  //     publish_joint_states();
  //     last_joint_states_publish_ = now;
  //   }
  // }

  // double status_dt = (now - last_status_publish_).seconds();
  // if (status_dt >= (1.0 / status_rate_)) {
  //   publish_status();
  //   last_status_publish_ = now;
  // }
}

#define SetDWORDval(arg) \
  (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg

void RoboClawDriverNode::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  static rclcpp::Time time_of_last_cmd_vel = this->get_clock()->now();

  // std::lock_guard<std::mutex> lock(last_cmd_vel_.mutex); //###
  last_cmd_vel_.cmd_vel = *msg;
  last_cmd_vel_.sequence_number++;
  last_cmd_vel_.timestamp = this->get_clock()->now();

  // std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  // current_cmd_vel_ = *msg;
  // cmd_vel_received_ = true;
  // last_cmd_vel_time_ = this->get_clock()->now();

  // if (do_debug_) {
  //   double delta_time =
  //       (last_cmd_vel_.timestamp - time_of_last_cmd_vel).seconds();
  //   RCLCPP_INFO(this->get_logger(),
  //               "Received cmd_vel: linear=%.3f, angular=%.3f at %f,
  //               delta_time "
  //               "= %f, sequence=%u",
  //               msg->linear.x, msg->angular.z,
  //               last_cmd_vel_.timestamp.seconds(), delta_time,
  //               last_cmd_vel_.sequence_number);
  //   time_of_last_cmd_vel = last_cmd_vel_.timestamp;
  // }
  int32_t target_left_speed;
  int32_t target_right_speed;
  convert_twist_to_motor_speeds(last_cmd_vel_.cmd_vel, target_left_speed,
                                target_right_speed);

  const int32_t m1_max_distance_quad_pulses =
      (int32_t)fabs(target_left_speed * max_seconds_uncommanded_travel_);
  const int32_t m2_max_distance_quad_pulses =
      (int32_t)fabs(target_right_speed * max_seconds_uncommanded_travel_);
  CmdDoBufferedM1M2DriveSpeedAccelDistance cmd(
      *roboclaw_, accel_, target_left_speed, m1_max_distance_quad_pulses,
      target_right_speed, m2_max_distance_quad_pulses);
  cmd.execute();
}

void RoboClawDriverNode::handle_cmd_vel() {
  // static uint32_t last_sequence_number = 0;
  // static rclcpp::Time time_of_last_cmd_vel = this->get_clock()->now();

  // // std::lock_guard<std::mutex> lock(last_cmd_vel_.mutex); //###

  // if (last_cmd_vel_.sequence_number <= last_sequence_number) {
  //   // No new command since last processed
  //   return;
  // }

  // last_sequence_number = last_cmd_vel_.sequence_number;

  // int32_t target_left_speed;
  // int32_t target_right_speed;
  // convert_twist_to_motor_speeds(last_cmd_vel_.cmd_vel, target_left_speed,
  //                               target_right_speed);

  // const int32_t m1_max_distance_quad_pulses =
  //     (int32_t)fabs(target_left_speed * max_seconds_uncommanded_travel_);
  // const int32_t m2_max_distance_quad_pulses =
  //     (int32_t)fabs(target_right_speed * max_seconds_uncommanded_travel_);
  // CmdDoBufferedM1M2DriveSpeedAccelDistance cmd(
  //     *roboclaw_, accel_, target_left_speed, m1_max_distance_quad_pulses,
  //     target_right_speed, m2_max_distance_quad_pulses);
  // cmd.execute();
  // if (do_debug_) {
  //   rclcpp::Time now = this->get_clock()->now();
  //   double delta_time = (now - time_of_last_cmd_vel).seconds();
  //   double lag_time = (now - last_cmd_vel_.timestamp).seconds();
  //   time_of_last_cmd_vel = last_cmd_vel_.timestamp;
  //   RCLCPP_INFO(this->get_logger(),
  //               "delta_time=%.3f, lag_time=%.3f, sequence=%u", delta_time,
  //               lag_time, last_cmd_vel_.sequence_number);
  // }
}

void RoboClawDriverNode::convert_twist_to_motor_speeds(
    const geometry_msgs::msg::Twist& twist, int32_t& left_speed,
    int32_t& right_speed) {
  // Differential drive kinematics
  double linear_vel =
      std::clamp(twist.linear.x, -max_linear_velocity_, max_linear_velocity_);
  double angular_vel = std::clamp(twist.angular.z, -max_angular_velocity_,
                                  max_angular_velocity_);

  // Calculate wheel velocities in m/s
  double left_wheel_vel = linear_vel - (angular_vel * wheel_separation_ / 2.0);
  double right_wheel_vel = linear_vel + (angular_vel * wheel_separation_ / 2.0);

  // Convert to encoder counts per second (QPPS - Quadrature Pulses Per
  // Second)
  double left_qpps = (left_wheel_vel / (2.0 * M_PI * wheel_radius_)) *
                     encoder_counts_per_revolution_;
  double right_qpps = (right_wheel_vel / (2.0 * M_PI * wheel_radius_)) *
                      encoder_counts_per_revolution_;

  left_speed = static_cast<int32_t>(left_qpps);
  right_speed = static_cast<int32_t>(right_qpps);
}

bool RoboClawDriverNode::get_fresh_encoders(uint32_t& enc1, uint32_t& enc2) {
  // if (!motors_initialized_) {
  //   return false;
  // }

  // uint8_t status1, status2;
  // bool valid1, valid2;

  // enc1 = roboclaw_->ReadEncM1(address_, &status1, &valid1);
  // enc2 = roboclaw_->ReadEncM2(address_, &status2, &valid2);

  // if (!valid1 || !valid2) {
  //   encoder_retries_++;
  //   if (encoder_retries_ > MAX_ENCODER_RETRIES) {
  //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
  //                          "Persistent encoder reading failures,
  //                          retries=%u", encoder_retries_);
  //     encoder_retries_ = 0;  // Reset counter
  //   }
  //   return false;
  // }

  // encoder_retries_ = 0;

  return true;
}

void RoboClawDriverNode::read_sensors() {
  // // Get fresh encoder readings
  // uint32_t enc1, enc2;
  // if (get_fresh_encoders(enc1, enc2)) {
  //   last_enc1_ = enc1;
  //   last_enc2_ = enc2;
  //   encoder_init_done_ = true;
  // }

  // // Spread status readings across cycles to avoid overwhelming serial
  // // communication This follows the TeensyV2 pattern of state machine for
  // // different readings
  // auto now = this->get_clock()->now();
  // static auto last_status_time = now;

  // if ((now - last_status_time).seconds() >= 0.1) {  // 10Hz status reading
  //   switch (current_status_state_) {
  //     case READ_BATTERY:
  //       // Read battery voltage (could publish this)
  //       break;
  //     case READ_TEMPERATURES:
  //       // Read temperature sensors
  //       break;
  //     case READ_CURRENTS:
  //       // Read motor currents
  //       break;
  //     case READ_ERROR:
  //       // Read error status
  //       break;
  //     case READ_PWMS:
  //       // Read PWM values
  //       break;
  //     case READ_BUFFERS:
  //       // Read command buffer depths
  //       break;
  //   }

  //   // Advance to next state
  //   current_status_state_ =
  //       static_cast<StatusReadState>((current_status_state_ + 1) % 6);
  //   last_status_time = now;
  // }
}

void RoboClawDriverNode::calculate_odometry() {
  // if (!encoder_init_done_) {
  //   return;
  // }

  // auto now = this->get_clock()->now();

  // if (first_encoder_reading_) {
  //   last_odom_time_ = now;
  //   first_encoder_reading_ = false;
  //   return;
  // }

  // double dt = (now - last_odom_time_).seconds();
  // if (dt < 0.01) {  // Skip if time delta too small
  //   return;
  // }

  // // This is a simplified odometry calculation - in a real implementation
  // you'd
  // // want to track encoder differences between readings, not absolute
  // values static uint32_t prev_enc1 = last_enc1_; static uint32_t prev_enc2
  // = last_enc2_;

  // int32_t delta_enc1 = static_cast<int32_t>(last_enc1_ - prev_enc1);
  // int32_t delta_enc2 = static_cast<int32_t>(last_enc2_ - prev_enc2);

  // prev_enc1 = last_enc1_;
  // prev_enc2 = last_enc2_;

  // double delta_left =
  //     (delta_enc1 * M_PI * wheel_diameter_) /
  //     encoder_counts_per_revolution_;
  // double delta_right =
  //     (delta_enc2 * M_PI * wheel_diameter_) /
  //     encoder_counts_per_revolution_;

  // // Calculate robot motion
  // double delta_distance = (delta_left + delta_right) / 2.0;
  // double delta_theta = (delta_right - delta_left) / wheel_separation_;

  // // Update pose
  // double delta_x = delta_distance * cos(theta_ + delta_theta / 2.0);
  // double delta_y = delta_distance * sin(theta_ + delta_theta / 2.0);

  // x_ += delta_x;
  // y_ += delta_y;
  // theta_ += delta_theta;
  // theta_ = normalize_angle(theta_);

  // // Calculate velocities
  // linear_velocity_ = delta_distance / dt;
  // angular_velocity_ = delta_theta / dt;

  // last_odom_time_ = now;
}

void RoboClawDriverNode::publish_odometry() {
  // auto odom_msg = nav_msgs::msg::Odometry();
  // odom_msg.header.stamp = this->get_clock()->now();
  // odom_msg.header.frame_id = odom_frame_;
  // odom_msg.child_frame_id = base_frame_;

  // // Position
  // odom_msg.pose.pose.position.x = x_;
  // odom_msg.pose.pose.position.y = y_;
  // odom_msg.pose.pose.position.z = 0.0;

  // // Orientation
  // tf2::Quaternion q;
  // q.setRPY(0, 0, theta_);
  // odom_msg.pose.pose.orientation = tf2::toMsg(q);

  // // Velocity
  // odom_msg.twist.twist.linear.x = linear_velocity_;
  // odom_msg.twist.twist.linear.y = 0.0;
  // odom_msg.twist.twist.angular.z = angular_velocity_;

  // // Covariance (simplified)
  // const double position_cov = 0.1;
  // const double orientation_cov = 0.1;
  // const double velocity_cov = 0.1;

  // odom_msg.pose.covariance[0] = position_cov;      // x
  // odom_msg.pose.covariance[7] = position_cov;      // y
  // odom_msg.pose.covariance[35] = orientation_cov;  // yaw

  // odom_msg.twist.covariance[0] = velocity_cov;   // vx
  // odom_msg.twist.covariance[35] = velocity_cov;  // vyaw

  // odom_pub_->publish(odom_msg);

  // // Publish transform
  // if (publish_tf_) {
  //   geometry_msgs::msg::TransformStamped transform;
  //   transform.header.stamp = odom_msg.header.stamp;
  //   transform.header.frame_id = odom_frame_;
  //   transform.child_frame_id = base_frame_;

  //   transform.transform.translation.x = x_;
  //   transform.transform.translation.y = y_;
  //   transform.transform.translation.z = 0.0;
  //   transform.transform.rotation = odom_msg.pose.pose.orientation;

  //   tf_broadcaster_->sendTransform(transform);
  // }
}

void RoboClawDriverNode::publish_joint_states() {
  // if (!joint_states_pub_) return;

  // auto joint_msg = sensor_msgs::msg::JointState();
  // joint_msg.header.stamp = this->get_clock()->now();

  // joint_msg.name = {"left_wheel_joint", "right_wheel_joint"};

  // // Convert encoder counts to radians
  // double left_angle =
  //     (2.0 * M_PI * last_enc1_) / encoder_counts_per_revolution_;
  // double right_angle =
  //     (2.0 * M_PI * last_enc2_) / encoder_counts_per_revolution_;

  // joint_msg.position = {left_angle, right_angle};

  // // Wheel velocities in rad/s
  // double wheel_radius = wheel_diameter_ / 2.0;
  // double left_vel =
  //     (linear_velocity_ - angular_velocity_ * wheel_separation_ / 2.0) /
  //     wheel_radius;
  // double right_vel =
  //     (linear_velocity_ + angular_velocity_ * wheel_separation_ / 2.0) /
  //     wheel_radius;

  // joint_msg.velocity = {left_vel, right_vel};

  // joint_states_pub_->publish(joint_msg);
}

void RoboClawDriverNode::publish_status() {
  // // This could publish battery voltage, temperatures, currents, errors,
  // etc.
  // // For now, we'll just log some basic status
  // if (do_debug_) {
  //   RCLCPP_DEBUG(this->get_logger(), "Status: connected=%s,
  //   encoders_valid=%s",
  //                motors_initialized_ ? "true" : "false",
  //                encoder_init_done_ ? "true" : "false");
  // }
}

double RoboClawDriverNode::normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
