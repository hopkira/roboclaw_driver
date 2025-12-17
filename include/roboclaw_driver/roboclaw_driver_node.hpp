// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 WimbleRobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file roboclaw_driver_node.hpp
 * @brief ROS2 driver node for BasicMicro RoboClaw motor controllers
 *
 * This driver provides high-level robot control through ROS2 interfaces while
 * maintaining compatibility with TeensyV2 timing patterns. It handles differential
 * drive kinematics, odometry calculation, and comprehensive motor monitoring.
 */

#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "roboclaw_driver/RoboClaw.h"

/**
 * @brief Main driver node for RoboClaw motor controller integration
 *
 * Provides ROS2 interface for differential drive robots using BasicMicro RoboClaw
 * controllers. Implements TeensyV2-compatible timing and command patterns for
 * reliable robot control and accurate odometry.
 */
class RoboClawDriverNode : public rclcpp::Node {
 public:
  RoboClawDriverNode();
  ~RoboClawDriverNode();

 private:
  // ============================================================================
  // CORE CONTROL FUNCTIONS
  // ============================================================================

  /** @brief Main control loop executed at fixed frequency */
  void main_loop();

  /** @brief Callback for incoming velocity commands */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /** @brief Decode RoboClaw error status bits into human-readable strings */
  void decodeErrorStatus(uint32_t error_status, char* buffer, size_t size) const;

  // ============================================================================
  // HARDWARE INTERFACE FUNCTIONS
  // ============================================================================

  /** @brief Initialize communication with RoboClaw controller */
  bool initialize_roboclaw();

  /** @brief Process cached velocity commands and send to motors */
  void handle_cmd_vel();

  /** @brief Read sensor data from RoboClaw (encoders, status, etc.) */
  void read_sensors();

  /** @brief Get fresh encoder readings with error handling */
  bool get_fresh_encoders(RoboClaw::EncodeResult& enc1, RoboClaw::EncodeResult& enc2);

  // ============================================================================
  // DATA PUBLISHING FUNCTIONS
  // ============================================================================

  /** @brief Calculate and publish robot odometry */
  void publish_odometry();

  /** @brief Publish wheel joint states for visualization */
  void publish_joint_states();

  /** @brief Publish motor status and health information */
  void publish_status();

  /** @brief Calculate odometry from encoder readings */
  void calculate_odometry();

  // ============================================================================
  // PARAMETER MANAGEMENT
  // ============================================================================

  /** @brief Declare all ROS2 parameters with default values */
  void declare_parameters();

  /** @brief Load parameter values into member variables */
  void load_parameters();

  /** @brief Log all parameter values for debugging */
  void log_parameters();

  // ============================================================================
  // UTILITY FUNCTIONS
  // ============================================================================

  /** @brief Convert ROS Twist to individual motor speeds using differential drive kinematics */
  void convert_twist_to_motor_speeds(const geometry_msgs::msg::Twist& twist, int32_t& left_speed,
                                     int32_t& right_speed);

  /** @brief Normalize angle to [-π, π] range */
  double normalize_angle(double angle);

  // ============================================================================
  // MEMBER VARIABLES
  // ============================================================================

  // Hardware interface
  std::unique_ptr<RoboClaw> roboclaw_;  ///< Interface to RoboClaw motor controller

  // ROS2 communication
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_sub_;                                                 ///< Velocity command subscriber
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;  ///< Odometry publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_pub_;                                            ///< Joint states publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;  ///< Status publisher
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;   ///< Transform broadcaster
  rclcpp::TimerBase::SharedPtr main_timer_;                         ///< Main control loop timer

  // Hardware configuration parameters
  uint8_t address_;          ///< RoboClaw device address (typically 0x80)
  std::string device_name_;  ///< Serial device path (e.g., "/dev/ttyUSB0")
  int32_t baud_rate_;        ///< Serial communication baud rate
  int device_timeout_;       ///< Command timeout in milliseconds

  // Publishing rates
  double odometry_rate_;
  double joint_states_rate_;
  double status_rate_;

  // Robot parameters
  double wheel_radius_;
  double wheel_separation_;
  int encoder_counts_per_revolution_;

  // Frame names
  std::string base_frame_;
  std::string odom_frame_;

  // Motor settings
  double max_linear_velocity_;
  double max_angular_velocity_;
  double max_seconds_uncommanded_travel_{0.05};

  // PID parameters
  double m1_p_;
  double m1_i_;
  double m1_d_;
  uint32_t m1_qpps_;
  double m2_p_;
  double m2_i_;
  double m2_d_;
  uint32_t m2_qpps_;
  uint32_t accel_;
  uint32_t decel_;
  uint32_t emergency_decel_;

  // Publishing control
  bool publish_odom_;
  bool publish_tf_;
  bool publish_joint_states_;

  // Debug settings
  bool do_debug_;
  bool do_low_level_debug_;

  // State variables
  struct {
    geometry_msgs::msg::Twist cmd_vel;
    std::mutex mutex;
    uint32_t sequence_number{0};
    rclcpp::Time timestamp{0};
  } last_cmd_vel_;

  // Odometry state
  double x_, y_, theta_;
  double linear_velocity_, angular_velocity_;
  bool first_encoder_reading_;
  rclcpp::Time last_odom_time_;

  // Status reading state machine
  enum StatusReadState { READ_BATTERY, READ_TEMPERATURES, READ_CURRENTS, READ_ERROR, READ_SPEEDS };
  StatusReadState current_status_state_;

  // Timing variables
  rclcpp::Time last_odometry_publish_;
  rclcpp::Time last_joint_states_publish_;
  rclcpp::Time last_status_publish_;

  // Motor command variables
  bool motors_initialized_;

  struct RoboclawState {
    uint32_t error_status{0};
    float logic_battery_voltage{0.0};
    RoboClaw::EncodeResult m1_enc_result;
    RoboClaw::EncodeResult m2_enc_result;
    int32_t m1_speed{0};
    int32_t m2_speed{0};
    float main_battery_voltage{0.0};
    RoboClaw::TMotorCurrents motorCurrents;
    float temperature1{0.0};
    float temperature2{0.0};
  } roboclaw_state_;

  // Constants
  static constexpr double MAIN_LOOP_FREQUENCY = 30.0;  // Hz
  static constexpr uint8_t ROBOCLAW_ADDRESS = 0x80;
};
