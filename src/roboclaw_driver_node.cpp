#include "roboclaw_driver/roboclaw_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sstream>

#include "nav_msgs/msg/odometry.hpp"
#include "roboclaw_driver/roboclaw_cmd_do_buffered_m1m2_drive_speed_accel_distance.h"
#include "roboclaw_driver/roboclaw_cmd_read_buffer_length.h"
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
      encoder_init_done_(false) {
  // Initialize parameters in separate functions for better organization
  declare_parameters();
  load_parameters();
  log_parameters();

  // Log all parameters alphabetically
  RCUTILS_LOG_INFO("=== RoboClaw Driver Parameters ===");
  RCUTILS_LOG_INFO("accel: %d", accel_);
  RCUTILS_LOG_INFO("base_frame: %s", base_frame_.c_str());
  RCUTILS_LOG_INFO("baud_rate: %d", baud_rate_);
  RCUTILS_LOG_INFO("cmd_vel_timeout: %.1f", cmd_vel_timeout_);
  RCUTILS_LOG_INFO("do_debug: %s", do_debug_ ? "true" : "false");
  RCUTILS_LOG_INFO("do_low_level_debug: %s",
                   do_low_level_debug_ ? "true" : "false");
  RCUTILS_LOG_INFO("device_name: %s", device_name_.c_str());
  RCUTILS_LOG_INFO("device_timeout: %d", device_timeout_);
  RCUTILS_LOG_INFO("encoder_counts_per_revolution: %d",
                   encoder_counts_per_revolution_);
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
  RCUTILS_LOG_INFO("max_m1_current: %.1f", max_m1_current_);
  RCUTILS_LOG_INFO("max_m2_current: %.1f", max_m2_current_);
  RCUTILS_LOG_INFO("max_retries: %d", max_retries_);
  RCUTILS_LOG_INFO("max_seconds_uncommanded_travel: %.3f",
                   max_seconds_uncommanded_travel_);
  RCUTILS_LOG_INFO("odom_frame: %s", odom_frame_.c_str());
  RCUTILS_LOG_INFO("odometry_rate: %.1f", odometry_rate_);
  RCUTILS_LOG_INFO("publish_joint_states: %s",
                   publish_joint_states_ ? "true" : "false");
  RCUTILS_LOG_INFO("publish_odom: %s", publish_odom_ ? "true" : "false");
  RCUTILS_LOG_INFO("publish_tf: %s", publish_tf_ ? "true" : "false");
  RCUTILS_LOG_INFO("status_rate: %.1f", status_rate_);
  RCUTILS_LOG_INFO("wheel_radius: %.3f", wheel_radius_);
  RCUTILS_LOG_INFO("wheel_separation: %.3f", wheel_separation_);
  RCUTILS_LOG_INFO("===================================");

  // Initialize RoboClaw
  roboclaw_ = std::make_unique<RoboClaw>(max_m1_current_, max_m2_current_,
                                         device_name_, address_, baud_rate_,
                                         do_debug_, do_low_level_debug_);

  if (!initialize_roboclaw()) {
    RCUTILS_LOG_ERROR("Failed to initialize RoboClaw driver");
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
    RCUTILS_LOG_WARN(
        "publish_tf is enabled but publish_odom is disabled. TF "
        "requires odometry. Disabling TF publishing.");
    publish_tf_ = false;
  }

  if (publish_joint_states_) {
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
  }

  status_pub_ =
      this->create_publisher<std_msgs::msg::String>("roboclaw_status", 10);

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

  RCUTILS_LOG_INFO("RoboClaw driver node initialized successfully");
}

RoboClawDriverNode::~RoboClawDriverNode() {
  if (roboclaw_) {
    // Stop motors on shutdown
    roboclaw_->stop();
  }
}

bool RoboClawDriverNode::initialize_roboclaw() {
  RCUTILS_LOG_INFO("Connected to RoboClaw on %s at %d baud",
                   device_name_.c_str(), baud_rate_);
  std::string version;
  CmdReadFirmwareVersion cmd(*roboclaw_, version);
  cmd.execute();

  RCUTILS_LOG_INFO("RoboClaw Firmware Version: %s", version.c_str());

  CmdSetPid command_m1_pid(*roboclaw_, RoboClaw::kM1, m1_p_, m1_i_, m1_d_,
                           m1_qpps_);
  command_m1_pid.execute();
  CmdSetPid command_m2_pid(*roboclaw_, RoboClaw::kM2, m2_p_, m2_i_, m2_d_,
                           m2_qpps_);
  command_m2_pid.execute();

  CmdSetEncoderValue m1(*roboclaw_, RoboClaw::kM1, 0);
  m1.execute();
  CmdSetEncoderValue m2(*roboclaw_, RoboClaw::kM2, 0);
  m2.execute();

  motors_initialized_ = true;
  return true;
}

void RoboClawDriverNode::main_loop() {
  auto now = this->get_clock()->now();

  // RCUTILS_LOG_INFO( "LOOP");

  // Handle cmd_vel processing and motor commands
  handle_cmd_vel();

  // Read sensor data and update odometry
  read_sensors();

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

  double status_dt = (now - last_status_publish_).seconds();
  if (status_dt >= (1.0 / status_rate_)) {
    publish_status();
    last_status_publish_ = now;
  }
}

#define SetDWORDval(arg) \
  (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg

void RoboClawDriverNode::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  static rclcpp::Time time_of_last_cmd_vel = this->get_clock()->now();

  std::lock_guard<std::mutex> lock(last_cmd_vel_.mutex);  // ###
  last_cmd_vel_.cmd_vel = *msg;
  last_cmd_vel_.sequence_number++;
  last_cmd_vel_.timestamp = this->get_clock()->now();
}

void RoboClawDriverNode::handle_cmd_vel() {
  static uint32_t last_sequence_number = 0;
  static rclcpp::Time time_of_last_cmd_vel = this->get_clock()->now();

  std::lock_guard<std::mutex> lock(last_cmd_vel_.mutex);

  if (last_cmd_vel_.sequence_number <= last_sequence_number) {
    // No new command since last processed
    return;
  }

  last_sequence_number = last_cmd_vel_.sequence_number;

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
  if (do_debug_) {
    rclcpp::Time now = this->get_clock()->now();
    double delta_time = (now - time_of_last_cmd_vel).seconds();
    double lag_time = (now - last_cmd_vel_.timestamp).seconds();
    time_of_last_cmd_vel = last_cmd_vel_.timestamp;
    RCUTILS_LOG_INFO("delta_time=%.3f, lag_time=%.3f, sequence=%u", delta_time,
                     lag_time, last_cmd_vel_.sequence_number);
  }
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

void RoboClawDriverNode::read_sensors() {
  // Get fresh encoder readings
  get_fresh_encoders(roboclaw_state_.m1_enc_result,
                     roboclaw_state_.m2_enc_result);

  // Spread status readings across cycles to avoid overwhelming serial
  // communication This follows the TeensyV2 pattern of state machine for
  // different readings
  auto now = this->get_clock()->now();
  static auto last_status_time = now;

  if ((now - last_status_time).seconds() >= 0.1) {  // 10Hz status reading
    switch (current_status_state_) {
      case READ_BATTERY: {
        CmdReadLogicBatteryVoltage cmd_logic_batt(
            *roboclaw_, roboclaw_state_.logic_battery_voltage);
        cmd_logic_batt.execute();
        CmdReadMainBatteryVoltage cmd_main_batt(
            *roboclaw_, roboclaw_state_.main_battery_voltage);
        cmd_main_batt.execute();
      }

      break;

      case READ_TEMPERATURES: {
        // Read temperature sensors
        CmdReadTemperature cmd_temp1(*roboclaw_, RoboClaw::kTemperature1,
                                     roboclaw_state_.temperature1);
        cmd_temp1.execute();
        CmdReadTemperature cmd_temp2(*roboclaw_, RoboClaw::kTemperature2,
                                     roboclaw_state_.temperature2);
        cmd_temp2.execute();
      }

      break;

      case READ_CURRENTS:
        // Read motor currents
        {
          CmdReadMotorCurrents cmd_currents(*roboclaw_,
                                            roboclaw_state_.motorCurrents);
          cmd_currents.execute();
        }
        break;

      case READ_ERROR: {
        CmdReadStatus cmd_status(*roboclaw_, roboclaw_state_.error_status);
        cmd_status.execute();
      }

      break;

      case READ_SPEEDS: {
        CmdReadEncoderSpeed cmd_speed_m1(*roboclaw_, RoboClaw::kM1,
                                         roboclaw_state_.m1_speed);
        cmd_speed_m1.execute();
        CmdReadEncoderSpeed cmd_speed_m2(*roboclaw_, RoboClaw::kM2,
                                         roboclaw_state_.m2_speed);
        cmd_speed_m2.execute();
      }

      break;

      case READ_BUFFERS:
        // Read command buffer depths
        break;
    }

    // Advance to next state
    current_status_state_ =
        static_cast<StatusReadState>((current_status_state_ + 1) % 6);
    last_status_time = now;
  }
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
  char decoded_error_status[256];
  decodeErrorStatus(roboclaw_state_.error_status, decoded_error_status,
                    sizeof(decoded_error_status));

  // Create JSON string with RoboClaw state data
  std::ostringstream error_status_hex;
  error_status_hex << std::setw(8) << std::setfill('0') << std::hex
                   << std::uppercase << roboclaw_state_.error_status;

  // Get current ROS2 time as seconds.nanoseconds (like header.stamp)
  auto stamp = this->get_clock()->now();

  // Format timestamp as "seconds.nanoseconds" (e.g., "1758159135.361360569")
  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(9)
                   << stamp.seconds() +
                          (stamp.nanoseconds() % 1000000000) / 1e9;

  std::string json_status =
      "{\"timestamp\":\"" + timestamp_stream.str() + "\",\"m1_current\":" +
      std::to_string(roboclaw_state_.motorCurrents.m1Current) +
      ",\"m2_current\":" +
      std::to_string(roboclaw_state_.motorCurrents.m2Current) +
      ",\"m1_speed\":" + std::to_string(roboclaw_state_.m1_speed) +
      ",\"m1_enc_value\":" +
      std::to_string(roboclaw_state_.m1_enc_result.value) +
      ",\"m1_enc_status\":" +
      std::to_string(static_cast<int>(roboclaw_state_.m1_enc_result.status)) +
      ",\"m2_speed\":" + std::to_string(roboclaw_state_.m2_speed) +
      ",\"m2_enc_value\":" +
      std::to_string(roboclaw_state_.m2_enc_result.value) +
      ",\"m2_enc_status\":" +
      std::to_string(static_cast<int>(roboclaw_state_.m2_enc_result.status)) +
      ",\"logic_battery\":" +
      std::to_string(roboclaw_state_.logic_battery_voltage) +
      ",\"main_battery\":" +
      std::to_string(roboclaw_state_.main_battery_voltage) +
      ", \"temperature1\":" + std::to_string(roboclaw_state_.temperature1) +
      ", \"temperature2\":" + std::to_string(roboclaw_state_.temperature2) +
      ", \"error_status\":\"" + error_status_hex.str() + "\"" +
      ", \"decoded_error_status\":\"" + std::string(decoded_error_status) +
      "\"}";

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
  this->declare_parameter("cmd_vel_timeout", 1.0);
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
  this->declare_parameter("max_m1_current", 0.0);
  this->declare_parameter("max_m2_current", 0.0);
  this->declare_parameter("max_retries", 3);
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
  cmd_vel_timeout_ = this->get_parameter_or("cmd_vel_timeout", 1.0);
  device_name_ =
      this->get_parameter_or("device_name", std::string("/dev/ttyAMA0"));
  device_timeout_ = this->get_parameter_or("device_timeout", 100);
  do_debug_ = this->get_parameter_or("do_debug", false);
  do_low_level_debug_ = this->get_parameter_or("do_low_level_debug", false);
  encoder_counts_per_revolution_ =
      this->get_parameter_or("encoder_counts_per_revolution", 1000);
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
  max_m1_current_ = this->get_parameter_or("max_m1_current", 0.0);
  max_m2_current_ = this->get_parameter_or("max_m2_current", 0.0);
  max_retries_ = this->get_parameter_or("max_retries", 3);
  max_seconds_uncommanded_travel_ =
      this->get_parameter_or("max_seconds_uncommanded_travel", 0.2);
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
    RCUTILS_LOG_INFO("  max_retries: %d", max_retries_);

    // Physical parameters
    RCUTILS_LOG_INFO("Robot Physical:");
    RCUTILS_LOG_INFO("  wheel_radius: %.6f m", wheel_radius_);
    RCUTILS_LOG_INFO("  wheel_separation: %.6f m", wheel_separation_);
    RCUTILS_LOG_INFO("  encoder_counts_per_revolution: %d",
                     encoder_counts_per_revolution_);
    RCUTILS_LOG_INFO("  accel: %u quad pulses/s²", accel_);

    // Safety parameters
    RCUTILS_LOG_INFO("Safety:");
    RCUTILS_LOG_INFO("  max_linear_velocity: %.3f m/s", max_linear_velocity_);
    RCUTILS_LOG_INFO("  max_angular_velocity: %.3f rad/s",
                     max_angular_velocity_);
    RCUTILS_LOG_INFO("  max_seconds_uncommanded_travel: %.3f s",
                     max_seconds_uncommanded_travel_);
    RCUTILS_LOG_INFO("  cmd_vel_timeout: %.3f s", cmd_vel_timeout_);
    RCUTILS_LOG_INFO("  max_m1_current: %.3f A", max_m1_current_);
    RCUTILS_LOG_INFO("  max_m2_current: %.3f A", max_m2_current_);

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
    RCUTILS_LOG_INFO("  publish_joint_states: %s",
                     publish_joint_states_ ? "true" : "false");
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
    RCUTILS_LOG_INFO("  do_low_level_debug: %s",
                     do_low_level_debug_ ? "true" : "false");

    RCUTILS_LOG_INFO("=== End Parameters ===");
  }
}

void RoboClawDriverNode::decodeErrorStatus(uint32_t error_status, char* buffer,
                                           size_t size) const {
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
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_ESTOP))
    append_error("ERROR_ESTOP");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_TEMP))
    append_error("ERROR_TEMP");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_TEMP2))
    append_error("ERROR_TEMP2");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_LBATHIGH))
    append_error("ERROR_LBATHIGH");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_LBATLOW))
    append_error("ERROR_LBATLOW");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_FAULTM1))
    append_error("ERROR_FAULTM1");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_FAULTM2))
    append_error("ERROR_FAULTM2");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_SPEED1))
    append_error("ERROR_SPEED1");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_SPEED2))
    append_error("ERROR_SPEED2");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_POS1))
    append_error("ERROR_POS1");
  if (error_status & static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_POS2))
    append_error("ERROR_POS2");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_CURRENTM1))
    append_error("ERROR_CURRENTM1");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::ERROR_CURRENTM2))
    append_error("ERROR_CURRENTM2");

  // Check warning flags (bits 16-31)
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_OVERCURRENTM1))
    append_error("WARN_OVERCURRENTM1");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_OVERCURRENTM2))
    append_error("WARN_OVERCURRENTM2");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_MBATHIGH))
    append_error("WARN_MBATHIGH");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_MBATLOW))
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
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_OVERREGENM1))
    append_error("WARN_OVERREGENM1");
  if (error_status &
      static_cast<uint32_t>(RoboClaw::RoboClawError::WARN_OVERREGENM2))
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
