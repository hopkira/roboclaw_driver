#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
// #include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "roboclaw_driver/RoboClaw.h"

class RoboClawDriverNode : public rclcpp::Node {
 public:
  RoboClawDriverNode();
  ~RoboClawDriverNode();

 private:
  // Main loop function
  void main_loop();

  // Callback functions
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Error decoding
  void decodeErrorStatus(uint32_t error_status, char* buffer,
                         size_t size) const;

  // Core functionality
  bool initialize_roboclaw();
  void handle_cmd_vel();
  void read_sensors();
  void publish_odometry();
  void publish_joint_states();
  void publish_status();
  void calculate_odometry();
  bool get_fresh_encoders(RoboClaw::EncodeResult& enc1,
                          RoboClaw::EncodeResult& enc2);

  // Parameter management
  void declare_parameters();
  void load_parameters();
  void log_parameters();

  // Utility functions
  void convert_twist_to_motor_speeds(const geometry_msgs::msg::Twist& twist,
                                     int32_t& left_speed, int32_t& right_speed);
  double normalize_angle(double angle);

  // RoboClaw interface
  std::unique_ptr<RoboClaw> roboclaw_;

  // ROS2 interface
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  // Configuration parameters
  uint8_t address_;
  std::string device_name_;
  int32_t baud_rate_;
  int device_timeout_;
  int max_retries_;

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
  double max_m1_current_{0.0};
  double max_m2_current_{0.0};

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

  // Safety settings
  double cmd_vel_timeout_;

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

  // std::atomic<bool> cmd_vel_received_;
  // geometry_msgs::msg::Twist current_cmd_vel_;
  // rclcpp::Time last_cmd_vel_time_;
  // std::mutex cmd_vel_mutex_;  // Odometry state
  double x_, y_, theta_;
  double linear_velocity_, angular_velocity_;
  uint32_t last_enc1_, last_enc2_;
  bool first_encoder_reading_;
  rclcpp::Time last_odom_time_;

  // Status reading state machine
  enum StatusReadState {
    READ_BATTERY,
    READ_TEMPERATURES,
    READ_CURRENTS,
    READ_ERROR,
    READ_SPEEDS,
    READ_BUFFERS
  };
  StatusReadState current_status_state_;

  // Timing variables
  rclcpp::Time last_odometry_publish_;
  rclcpp::Time last_joint_states_publish_;
  rclcpp::Time last_status_publish_;

  // Motor command variables
  bool motors_initialized_;

  // Encoder management
  bool encoder_init_done_;
  static constexpr uint32_t MAX_ENCODER_RETRIES = 5;

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
    // int16_t m1_pwm;
    // int16_t m2_pwm;
    // uint16_t m1_buffer;
    // uint16_t m2_buffer;
  } roboclaw_state_;

  // Constants
  static constexpr double MAIN_LOOP_FREQUENCY = 30.0;  // Hz
  static constexpr uint8_t ROBOCLAW_ADDRESS = 0x80;
};
