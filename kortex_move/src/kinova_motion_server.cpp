#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kortex_interfaces/action/kinova_command_velocity.hpp"
#include "kortex_interfaces/srv/switch_control_mode.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class KinovaMotionServer : public rclcpp::Node {
 public:
  using ArmMotion = kinova_interfaces::action::KinovaCommandVelocity;
  using GoalHandleArmMotion = rclcpp_action::ServerGoalHandle<ArmMotion>;

  explicit KinovaMotionServer(const rclcpp::NodeOptions& options)
      : Node("kinova_motion_server", options) {
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    switcher_client_ =
        this->create_client<kinova_interfaces::srv::SwitchControlMode>(
            "/switch_control_mode", rmw_qos_profile_services_default,
            callback_group_);

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/twist_controller/commands", 10);

    force_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "load_cell_data", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          this->current_force_ = msg->data;
        },
        sub_opt);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this), "manipulator");

    this->action_server_ = rclcpp_action::create_server<ArmMotion>(
        this, "kinova_arm_motion",
        std::bind(&KinovaMotionServer::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&KinovaMotionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&KinovaMotionServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Kinova Motion Server is ready.");
  }

 private:
  rclcpp_action::Server<ArmMotion>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<kinova_interfaces::srv::SwitchControlMode>::SharedPtr
      switcher_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double current_force_ = 0.0;
  std::atomic<bool> cancel_requested_{false};

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&, std::shared_ptr<const ArmMotion::Goal>);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleArmMotion>);
  void handle_accepted(const std::shared_ptr<GoalHandleArmMotion>);
  void execute(const std::shared_ptr<GoalHandleArmMotion>);
  // Flag to indicate if an action is in progress
  std::atomic<bool> action_in_progress_;

  // Our new cleanup function
  void force_cleanup_and_reset();

  // Existing methods
  bool execute_proportional_pose_control(
      const std::shared_ptr<GoalHandleArmMotion>);
  bool execute_proportional_approach_control(
      const std::shared_ptr<GoalHandleArmMotion>);
  bool execute_proportional_pull_control(
      const std::shared_ptr<GoalHandleArmMotion>);

  // NEW: Rotation methods
  bool execute_rotate_to_angle_z(const std::shared_ptr<GoalHandleArmMotion>);
  bool execute_rotate_by_angle_z(const std::shared_ptr<GoalHandleArmMotion>);
  bool execute_orient_towards_point(const std::shared_ptr<GoalHandleArmMotion>);
  geometry_msgs::msg::Quaternion calculate_orientation_to_point(
      const geometry_msgs::msg::Point& from_point,
      const geometry_msgs::msg::Point& to_point);

  // Helper methods
  bool get_current_end_effector_pose(geometry_msgs::msg::Pose& pose);
  double get_current_yaw_angle();
  double normalize_angle(double angle);
  double calculate_rotation_direction(double current_angle, double target_angle,
                                      int desired_direction);
  bool switch_to_mode(const std::string& mode);
  void stop_robot_and_switch_back();
};

rclcpp_action::GoalResponse KinovaMotionServer::handle_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const ArmMotion::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Handle cancellation of an action
rclcpp_action::CancelResponse KinovaMotionServer::handle_cancel(
    const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
  RCLCPP_WARN(this->get_logger(),
              "Cancellation request received! Starting forced cleanup...");

  // Set cancellation flag to interrupt the main action loop
  cancel_requested_.store(true);

  // Immediately call the cleanup and reset function
  force_cleanup_and_reset();

  RCLCPP_INFO(this->get_logger(),
              "Cancellation accepted and cleanup performed.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

// Function to force cleanup and reset robot state
void KinovaMotionServer::force_cleanup_and_reset() {
  RCLCPP_INFO(this->get_logger(), "Executing forced cleanup...");

  // 1. Immediately stop the robot by sending zero velocity
  geometry_msgs::msg::Twist zero_twist;
  zero_twist.linear.x = 0.0;
  zero_twist.linear.y = 0.0;
  zero_twist.linear.z = 0.0;
  zero_twist.angular.x = 0.0;
  zero_twist.angular.y = 0.0;
  zero_twist.angular.z = 0.0;

  // Publish the message multiple times to ensure it's received
  // in case of network or system latency.
  for (int i = 0; i < 5; ++i) {
    twist_publisher_->publish(zero_twist);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  RCLCPP_INFO(this->get_logger(), "Stop command (zero velocity) sent.");

  // 2. Force return to "position" mode
  // This is the fundamental part to unlock future actions.
  if (switch_to_mode("position")) {
    RCLCPP_INFO(this->get_logger(),
                "Controller successfully returned to 'position' mode.");
  } else {
    RCLCPP_ERROR(
        this->get_logger(),
        "CRITICAL ERROR: Unable to return controller to 'position' mode!");
  }

  // 3. Reset internal server state
  action_in_progress_.store(false);

  RCLCPP_INFO(this->get_logger(),
              "Forced cleanup completed. System is ready for new actions.");
}

void KinovaMotionServer::handle_accepted(
    const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
  cancel_requested_.store(false);
  std::thread{
      std::bind(&KinovaMotionServer::execute, this, std::placeholders::_1),
      goal_handle}
      .detach();
}

// Use this alias to make the code more readable
using ArmMotion = kinova_interfaces::action::KinovaCommandVelocity;

void KinovaMotionServer::execute(
    const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "New action received. Starting execution...");

  // Set state flags at the beginning of a new action
  action_in_progress_.store(true);
  cancel_requested_.store(false);

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ArmMotion::Result>();

  // =================================================================
  // STEP 1: DETERMINE AND SET THE CORRECT MODE
  // =================================================================
  bool mode_switch_ok = false;
  std::string required_mode = "";

  // This first switch serves ONLY to determine the required mode
  switch (goal->command_type) {
    // Commands that require VELOCITY mode
    case ArmMotion::Goal::EXECUTE_PULL_WITH_FORCE_LIMIT:
    case ArmMotion::Goal::ROTATE_BY_ANGLE_Z:
    case ArmMotion::Goal::APPROACH_WITH_FORCE_CONTROL:
    case ArmMotion::Goal::ROTATE_TO_ANGLE_Z:  // Added from your version
      required_mode = "velocity";
      break;

    // Commands that require POSITION mode
    case ArmMotion::Goal::GO_TO_CARTESIAN_POSE:  // Left for future robustness
    case ArmMotion::Goal::MOVE_TO_POSE_PROPORTIONAL:
    case ArmMotion::Goal::ORIENT_TOWARDS_POINT:
      required_mode = "position";
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Unrecognized command type: %d",
                   goal->command_type);
      result->success = false;
      result->message = "Unknown command type.";
      goal_handle->abort(result);
      action_in_progress_.store(false);
      return;
  }

  // Execute the mode change
  RCLCPP_INFO(this->get_logger(),
              "Command requires '%s mode'. Starting mode switch...",
              required_mode.c_str());
  mode_switch_ok = switch_to_mode(required_mode);

  // If mode switch fails, immediately abort the action.
  if (!mode_switch_ok) {
    RCLCPP_ERROR(this->get_logger(),
                 "Unable to switch to '%s mode'! Aborting action.",
                 required_mode.c_str());
    result->success = false;
    result->message = "Failed to switch controller mode.";
    goal_handle->abort(result);
    action_in_progress_.store(false);
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Mode set successfully. Executing command...");

  // =================================================================
  // STEP 2: EXECUTE THE SPECIFIC COMMAND (WITH YOUR FUNCTIONS)
  // =================================================================
  bool action_success = false;

  switch (goal->command_type) {
    case ArmMotion::Goal::MOVE_TO_POSE_PROPORTIONAL:
      action_success = execute_proportional_pose_control(goal_handle);
      break;
    case ArmMotion::Goal::APPROACH_WITH_FORCE_CONTROL:
      action_success = execute_proportional_approach_control(goal_handle);
      break;
    case ArmMotion::Goal::EXECUTE_PULL_WITH_FORCE_LIMIT:
      action_success = execute_proportional_pull_control(goal_handle);
      break;
    case ArmMotion::Goal::ROTATE_TO_ANGLE_Z:
      action_success = execute_rotate_to_angle_z(goal_handle);
      break;
    case ArmMotion::Goal::ROTATE_BY_ANGLE_Z:
      action_success = execute_rotate_by_angle_z(goal_handle);
      break;
    case ArmMotion::Goal::ORIENT_TOWARDS_POINT:
      action_success = execute_orient_towards_point(goal_handle);
      break;
  }

  // =================================================================
  // STEP 3: CONCLUSION AND CLEANUP
  // =================================================================

  if (rclcpp::ok()) {
    if (cancel_requested_.load()) {
      RCLCPP_WARN(this->get_logger(), "Action was cancelled during execution.");
      result->success = false;
      result->message = "Action was cancelled.";
      goal_handle->canceled(result);
    } else if (action_success) {
      RCLCPP_INFO(this->get_logger(), "Action completed successfully.");
      result->success = true;
      result->message = "Action completed successfully.";
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Action failed during execution.");
      result->success = false;
      result->message = "Action failed during execution.";
      goal_handle->abort(result);
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "Action ended. Returning to 'position mode' as default state for "
              "safety.");
  switch_to_mode("position");

  action_in_progress_.store(false);
  RCLCPP_INFO(this->get_logger(), "Server ready for new action.");
}

bool KinovaMotionServer::execute_orient_towards_point(
    const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "== COMBINED POSITION + ORIENTATION CONTROL ==");
  const auto goal = goal_handle->get_goal();

  if (!switch_to_mode("velocity")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to switch to velocity mode");
    return false;
  }

  // Get target position and orientation target
  auto target_position = goal->target_position_combined;
  auto target_point = goal->target_point;

  RCLCPP_INFO(this->get_logger(), "Target position: [%.3f, %.3f, %.3f]",
              target_position.x, target_position.y, target_position.z);
  RCLCPP_INFO(this->get_logger(), "Looking towards: [%.3f, %.3f, %.3f]",
              target_point.x, target_point.y, target_point.z);

  // Control parameters for BOTH linear and angular
  double linear_gain = goal->position_kp > 0 ? goal->position_kp : 0.8;
  double angular_gain = goal->angular_kp > 0 ? goal->angular_kp : 2.0;
  double max_linear_speed =
      goal->max_linear_speed > 0 ? goal->max_linear_speed : 0.05;
  double max_angular_speed =
      goal->max_angular_speed > 0 ? goal->max_angular_speed : 0.5;
  double pos_tolerance =
      goal->position_tolerance > 0 ? goal->position_tolerance : 0.005;
  double ang_tolerance =
      goal->angular_tolerance > 0 ? goal->angular_tolerance : 0.05;

  RCLCPP_INFO(this->get_logger(), "Control gains - Linear: %.2f, Angular: %.2f",
              linear_gain, angular_gain);
  RCLCPP_INFO(this->get_logger(),
              "Max speeds - Linear: %.3f m/s, Angular: %.3f rad/s",
              max_linear_speed, max_angular_speed);

  rclcpp::Rate rate(100);  // 100 Hz control loop

  while (rclcpp::ok()) {
    if (cancel_requested_.load()) {
      stop_robot_and_switch_back();
      return false;
    }

    // Get current robot pose
    geometry_msgs::msg::Pose current_pose;
    if (!get_current_end_effector_pose(current_pose)) {
      RCLCPP_WARN(this->get_logger(), "Cannot get current pose, retrying...");
      rate.sleep();
      continue;
    }

    // === POSITION CONTROL ===
    // Calculate linear error (position)
    tf2::Vector3 position_error(target_position.x - current_pose.position.x,
                                target_position.y - current_pose.position.y,
                                target_position.z - current_pose.position.z);
    double distance_error = position_error.length();

    // === ORIENTATION CONTROL ===
    // Calculate target orientation (gripper should look at target_point from
    // target_position)
    auto target_orientation =
        calculate_orientation_to_point(target_position, target_point);

    // Calculate orientation error
    tf2::Quaternion current_quat, target_quat;
    tf2::fromMsg(current_pose.orientation, current_quat);
    tf2::fromMsg(target_orientation, target_quat);

    tf2::Quaternion error_quat = target_quat * current_quat.inverse();
    double orientation_error = error_quat.getAngle();

    // === CONVERGENCE CHECK ===
    if (distance_error < pos_tolerance && orientation_error < ang_tolerance) {
      RCLCPP_INFO(this->get_logger(),
                  "Target position and orientation reached!");
      RCLCPP_INFO(this->get_logger(),
                  "Final errors - Position: %.4f m, Orientation: %.4f rad",
                  distance_error, orientation_error);
      stop_robot_and_switch_back();
      return true;
    }

    // === CALCULATE VELOCITIES ===

    // Linear velocity (in base frame)
    tf2::Vector3 linear_velocity_base = linear_gain * position_error;

    // Limit linear speed
    if (linear_velocity_base.length() > max_linear_speed) {
      linear_velocity_base =
          linear_velocity_base.normalized() * max_linear_speed;
    }

    // Angular velocity (in base frame)
    tf2::Vector3 error_axis = error_quat.getAxis();
    tf2::Vector3 angular_velocity_base =
        angular_gain * orientation_error * error_axis;

    // Limit angular speed
    if (angular_velocity_base.length() > max_angular_speed) {
      angular_velocity_base =
          angular_velocity_base.normalized() * max_angular_speed;
    }

    // === TRANSFORM TO END-EFFECTOR FRAME ===
    tf2::Quaternion q_ee_to_base = current_quat.inverse();

    tf2::Vector3 linear_velocity_ee =
        tf2::quatRotate(q_ee_to_base, linear_velocity_base);
    tf2::Vector3 angular_velocity_ee =
        tf2::quatRotate(q_ee_to_base, angular_velocity_base);

    // === SEND COMBINED TWIST COMMAND ===
    geometry_msgs::msg::Twist twist_cmd;
    // Linear velocities (position control)
    twist_cmd.linear.x = linear_velocity_ee.x();
    twist_cmd.linear.y = linear_velocity_ee.y();
    twist_cmd.linear.z = linear_velocity_ee.z();
    // Angular velocities (orientation control)
    twist_cmd.angular.x = angular_velocity_ee.x();
    twist_cmd.angular.y = angular_velocity_ee.y();
    twist_cmd.angular.z = angular_velocity_ee.z();

    twist_publisher_->publish(twist_cmd);

    // === DEBUG OUTPUT ===
    static int debug_counter = 0;
    if (++debug_counter % 20 == 0) {  // Log every 0.2 seconds
      RCLCPP_INFO(this->get_logger(),
                  "Errors - Pos: %.4f m, Orient: %.3f rad | "
                  "Lin vel: [%.3f, %.3f, %.3f] | "
                  "Ang vel: [%.3f, %.3f, %.3f]",
                  distance_error, orientation_error, linear_velocity_ee.x(),
                  linear_velocity_ee.y(), linear_velocity_ee.z(),
                  angular_velocity_ee.x(), angular_velocity_ee.y(),
                  angular_velocity_ee.z());
    }

    rate.sleep();
  }

  stop_robot_and_switch_back();
  return false;
}

geometry_msgs::msg::Quaternion
KinovaMotionServer::calculate_orientation_to_point(
    const geometry_msgs::msg::Point& from_point,
    const geometry_msgs::msg::Point& to_point) {
  // Calculate direction vector
  tf2::Vector3 direction(to_point.x - from_point.x, to_point.y - from_point.y,
                         to_point.z - from_point.z);

  // Normalize to get Z-axis
  direction.normalize();

  // Define up vector
  tf2::Vector3 up(0, 0, 1);

  // Calculate X-axis (cross product of up and direction)
  tf2::Vector3 x_axis = up.cross(direction);
  x_axis.normalize();

  // Calculate Y-axis
  tf2::Vector3 y_axis = direction.cross(x_axis);

  // Create rotation matrix
  tf2::Matrix3x3 rotation_matrix(x_axis.x(), y_axis.x(), direction.x(),
                                 x_axis.y(), y_axis.y(), direction.y(),
                                 x_axis.z(), y_axis.z(), direction.z());

  // Convert to quaternion
  tf2::Quaternion quaternion;
  rotation_matrix.getRotation(quaternion);

  return tf2::toMsg(quaternion);
}

bool KinovaMotionServer::execute_rotate_to_angle_z(
    const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "== START ABSOLUTE GRIPPER ROTATION ==");
  const auto goal = goal_handle->get_goal();

  if (!switch_to_mode("velocity")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to switch to 'velocity' mode.");
    return false;
  }

  double target_angle = normalize_angle(goal->target_angle_z);
  double angular_gain = goal->angular_kp > 0 ? goal->angular_kp : 2.0;
  double angular_tolerance = goal->angular_tolerance > 0
                                 ? goal->angular_tolerance
                                 : 0.05;  // ~3 degrees
  int desired_direction =
      goal->rotation_direction;  // 1 = counterclockwise, -1 = clockwise
  double max_angular_speed =
      goal->max_angular_speed > 0 ? goal->max_angular_speed : 0.5;  // rad/s

  RCLCPP_INFO(this->get_logger(),
              "Target angle: %.2f rad (%.1f°), Direction: %s", target_angle,
              target_angle * 180.0 / M_PI,
              desired_direction == 1 ? "counterclockwise" : "clockwise");

  rclcpp::Rate rate(100);

  while (rclcpp::ok()) {
    if (cancel_requested_.load()) {
      stop_robot_and_switch_back();
      return false;
    }

    double current_angle = get_current_yaw_angle();
    if (std::isnan(current_angle)) {
      RCLCPP_WARN(this->get_logger(), "Cannot get current angle, retrying...");
      rate.sleep();
      continue;
    }

    double angular_error = calculate_rotation_direction(
        current_angle, target_angle, desired_direction);

    // Check if we have reached the target
    if (std::abs(angular_error) < angular_tolerance) {
      RCLCPP_INFO(this->get_logger(),
                  "Target angle reached! Error: %.3f rad (%.1f°)",
                  angular_error, angular_error * 180.0 / M_PI);
      stop_robot_and_switch_back();
      return true;
    }

    // Proportional control
    double angular_velocity = angular_gain * angular_error;

    // Limit maximum speed
    if (std::abs(angular_velocity) > max_angular_speed) {
      angular_velocity =
          (angular_velocity > 0) ? max_angular_speed : -max_angular_speed;
    }

    geometry_msgs::msg::Twist twist_cmd;
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = angular_velocity;

    twist_publisher_->publish(twist_cmd);

    static int debug_counter = 0;
    if (++debug_counter % 20 == 0) {  // Log every 0.2 seconds
      RCLCPP_INFO(this->get_logger(),
                  "Angle: %.2f° -> %.2f° | Error: %.2f° | Vel: %.3f rad/s",
                  current_angle * 180.0 / M_PI, target_angle * 180.0 / M_PI,
                  angular_error * 180.0 / M_PI, angular_velocity);
    }

    rate.sleep();
  }

  stop_robot_and_switch_back();
  return false;
}

bool KinovaMotionServer::execute_rotate_by_angle_z(
    const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "== START RELATIVE GRIPPER ROTATION ==");
  const auto goal = goal_handle->get_goal();

  if (!switch_to_mode("velocity")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to switch to 'velocity' mode.");
    return false;
  }

  // Get current angle as starting point
  double start_angle = get_current_yaw_angle();
  if (std::isnan(start_angle)) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get starting angle.");
    stop_robot_and_switch_back();
    return false;
  }

  double rotation_amount = goal->rotation_amount;
  int desired_direction = goal->rotation_direction;
  double signed_rotation = rotation_amount * desired_direction;

  // NEW: Track total angle rotated
  double total_rotated = 0.0;
  double last_angle = start_angle;

  double angular_gain = goal->angular_kp > 0 ? goal->angular_kp : 2.0;
  double angular_tolerance =
      goal->angular_tolerance > 0 ? goal->angular_tolerance : 0.05;
  double max_angular_speed =
      goal->max_angular_speed > 0 ? goal->max_angular_speed : 0.5;

  RCLCPP_INFO(this->get_logger(), "Start: %.1f°, Target rotation: %.1f° %s",
              start_angle * 180.0 / M_PI, rotation_amount * 180.0 / M_PI,
              desired_direction == 1 ? "counterclockwise" : "clockwise");

  rclcpp::Rate rate(100);

  while (rclcpp::ok()) {
    if (cancel_requested_.load()) {
      stop_robot_and_switch_back();
      return false;
    }

    double current_angle = get_current_yaw_angle();
    if (std::isnan(current_angle)) {
      RCLCPP_WARN(this->get_logger(), "Cannot get current angle, retrying...");
      rate.sleep();
      continue;
    }

    // Calculate rotation from last cycle
    double angle_diff = current_angle - last_angle;
    // Handle wrap-around (-π to π)
    if (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    if (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    total_rotated += angle_diff;
    last_angle = current_angle;

    // Calculate error based on total rotated
    double remaining_rotation = signed_rotation - total_rotated;

    // Check if we have reached the target
    if (std::abs(remaining_rotation) < angular_tolerance) {
      RCLCPP_INFO(this->get_logger(),
                  "Rotation completed! Total rotated: %.1f°",
                  total_rotated * 180.0 / M_PI);
      stop_robot_and_switch_back();
      return true;
    }

    // Proportional control based on remaining rotation
    double angular_velocity = angular_gain * remaining_rotation;

    // Limit maximum speed
    if (std::abs(angular_velocity) > max_angular_speed) {
      angular_velocity =
          (angular_velocity > 0) ? max_angular_speed : -max_angular_speed;
    }

    geometry_msgs::msg::Twist twist_cmd;
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = angular_velocity;

    twist_publisher_->publish(twist_cmd);

    static int debug_counter = 0;
    if (++debug_counter % 20 == 0) {  // Log every 0.2 seconds
      RCLCPP_INFO(this->get_logger(),
                  "Total rotated: %.1f° | Remaining: %.1f° | Vel: %.3f rad/s",
                  total_rotated * 180.0 / M_PI,
                  remaining_rotation * 180.0 / M_PI, angular_velocity);
    }

    rate.sleep();
  }

  stop_robot_and_switch_back();
  return false;
}

double KinovaMotionServer::get_current_yaw_angle() {
  try {
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("base_link", "end_effector_link",
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(0.2));

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return normalize_angle(yaw);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "TF lookup for yaw failed: %s", ex.what());
    return std::nan("");
  }
}

double KinovaMotionServer::normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double KinovaMotionServer::calculate_rotation_direction(double current_angle,
                                                        double target_angle,
                                                        int desired_direction) {
  // Calculate direct error
  double direct_error = target_angle - current_angle;
  direct_error = normalize_angle(direct_error);

  // If no direction specified, use shortest path
  if (desired_direction == 0) {
    return direct_error;
  }

  // If desired direction matches sign of direct error, use it
  if ((desired_direction > 0 && direct_error > 0) ||
      (desired_direction < 0 && direct_error < 0)) {
    return direct_error;
  }

  // Otherwise, calculate error in opposite direction
  if (desired_direction > 0) {
    // We want counterclockwise, but direct error is clockwise
    return direct_error + 2.0 * M_PI;
  } else {
    // We want clockwise, but direct error is counterclockwise
    return direct_error - 2.0 * M_PI;
  }
}

bool KinovaMotionServer::execute_proportional_pose_control(
    const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "== START PROPORTIONAL POSE CONTROL ==");
  const auto goal = goal_handle->get_goal();

  if (!switch_to_mode("velocity")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to switch to 'velocity' mode.");
    return false;
  }

  geometry_msgs::msg::Pose target_pose;
  target_pose.position = goal->target_position_proportional;

  geometry_msgs::msg::Pose initial_pose;
  if (!get_current_end_effector_pose(initial_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get initial pose.");
    stop_robot_and_switch_back();
    return false;
  }
  target_pose.orientation = initial_pose.orientation;

  double linear_gain = goal->position_kp > 0 ? goal->position_kp : 0.8;
  double angular_gain = goal->angular_kp > 0 ? goal->angular_kp : 1.2;
  double pos_tolerance =
      goal->position_tolerance > 0 ? goal->position_tolerance : 0.005;
  double ang_tolerance =
      goal->angular_tolerance > 0 ? goal->angular_tolerance : 0.02;

  rclcpp::Rate rate(100);

  while (rclcpp::ok()) {
    if (cancel_requested_.load()) {
      stop_robot_and_switch_back();
      return false;
    }

    geometry_msgs::msg::Pose current_pose;
    if (!get_current_end_effector_pose(current_pose)) {
      rate.sleep();
      continue;
    }

    tf2::Vector3 error_linear(target_pose.position.x - current_pose.position.x,
                              target_pose.position.y - current_pose.position.y,
                              target_pose.position.z - current_pose.position.z);
    double dist_error = error_linear.length();

    tf2::Quaternion q_current, q_target;
    tf2::fromMsg(current_pose.orientation, q_current);
    tf2::fromMsg(target_pose.orientation, q_target);
    tf2::Quaternion q_error = q_target * q_current.inverse();
    double ang_error_norm = q_error.getAngle();

    if (dist_error < pos_tolerance && ang_error_norm < ang_tolerance) {
      RCLCPP_INFO(this->get_logger(), "Target pose reached!");
      stop_robot_and_switch_back();
      return true;
    }

    tf2::Vector3 error_angular = q_error.getAxis() * ang_error_norm;

    tf2::Vector3 linear_velocity_base = linear_gain * error_linear;
    tf2::Vector3 angular_velocity_base = angular_gain * error_angular;

    tf2::Quaternion q_ee_to_base = q_current.inverse();

    tf2::Vector3 linear_velocity_ee =
        tf2::quatRotate(q_ee_to_base, linear_velocity_base);
    tf2::Vector3 angular_velocity_ee =
        tf2::quatRotate(q_ee_to_base, angular_velocity_base);

    geometry_msgs::msg::Twist twist_cmd;
    twist_cmd.linear.x = linear_velocity_ee.x();
    twist_cmd.linear.y = linear_velocity_ee.y();
    twist_cmd.linear.z = linear_velocity_ee.z();
    twist_cmd.angular.x = angular_velocity_ee.x();
    twist_cmd.angular.y = angular_velocity_ee.y();
    twist_cmd.angular.z = angular_velocity_ee.z();

    twist_publisher_->publish(twist_cmd);
    rate.sleep();
  }
  stop_robot_and_switch_back();
  return false;
}

bool KinovaMotionServer::execute_proportional_approach_control(
    const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "== START APPROACH WITH FORCE CONTROL ==");
  const auto goal = goal_handle->get_goal();

  if (!switch_to_mode("velocity")) {
    return false;
  }

  double force_gain = goal->force_kp > 0 ? goal->force_kp : 0.002;
  double max_speed =
      goal->max_linear_speed > 0 ? goal->max_linear_speed : 0.015;
  double target_force = std::abs(goal->target_force_threshold);
  double min_speed =
      goal->min_speed_threshold > 0 ? goal->min_speed_threshold : 0.001;

  tf2::Vector3 direction_ee(goal->movement_direction.x,
                            goal->movement_direction.y,
                            goal->movement_direction.z);
  direction_ee.normalize();

  RCLCPP_INFO(this->get_logger(),
              "Approach params - force_kp: %.4f, target_force: %.1f, "
              "direction_ee: [%.2f, %.2f, %.2f]",
              force_gain, target_force, direction_ee.x(), direction_ee.y(),
              direction_ee.z());

  rclcpp::Rate rate(100);

  while (rclcpp::ok()) {
    if (cancel_requested_.load()) {
      stop_robot_and_switch_back();
      return false;
    }

    double current_force_abs = std::abs(current_force_);

    if (current_force_abs >= target_force) {
      RCLCPP_INFO(this->get_logger(),
                  "Force threshold reached! Current force: %.1f",
                  current_force_abs);
      stop_robot_and_switch_back();
      return true;
    }

    double force_margin = target_force - current_force_abs;
    double desired_speed = std::min(force_gain * force_margin, max_speed);

    if (desired_speed < min_speed && current_force_abs < target_force * 0.9) {
      desired_speed = min_speed;
    }

    tf2::Vector3 linear_velocity_ee = direction_ee * desired_speed;

    geometry_msgs::msg::Twist twist_cmd;
    twist_cmd.linear.x = linear_velocity_ee.x();
    twist_cmd.linear.y = linear_velocity_ee.y();
    twist_cmd.linear.z = linear_velocity_ee.z();
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;

    twist_publisher_->publish(twist_cmd);

    static int debug_counter = 0;
    if (++debug_counter % 20 == 0) {
      RCLCPP_INFO(this->get_logger(), "Force: %.1f/%.1f | Speed: %.4f",
                  current_force_abs, target_force, desired_speed);
    }
    rate.sleep();
  }
  stop_robot_and_switch_back();
  return false;
}

bool KinovaMotionServer::execute_proportional_pull_control(
    const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "== START PULLING WITH FORCE LIMIT ==");
  const auto goal = goal_handle->get_goal();

  if (!switch_to_mode("velocity")) {
    return false;
  }

  double force_gain = goal->force_kp > 0 ? goal->force_kp : 0.001;
  double max_speed =
      goal->max_linear_speed > 0 ? goal->max_linear_speed : 0.010;
  double max_force = std::abs(goal->max_force_threshold);
  double min_speed =
      goal->min_speed_threshold > 0 ? goal->min_speed_threshold : 0.0005;

  tf2::Vector3 direction_ee(goal->movement_direction.x,
                            goal->movement_direction.y,
                            goal->movement_direction.z);
  direction_ee.normalize();

  RCLCPP_INFO(this->get_logger(),
              "Pull params - force_kp: %.4f, max_force: %.1f, direction_ee: "
              "[%.2f, %.2f, %.2f]",
              force_gain, max_force, direction_ee.x(), direction_ee.y(),
              direction_ee.z());

  rclcpp::Rate rate(100);

  while (rclcpp::ok()) {
    if (cancel_requested_.load()) {
      stop_robot_and_switch_back();
      return false;
    }

    double current_force_abs = std::abs(current_force_);

    static int debug_counter_detailed = 0;
    if (++debug_counter_detailed % 50 == 0) {
      RCLCPP_INFO(
          this->get_logger(),
          "PULL DEBUG - Force: %.1f, Max: %.1f, Continue: %s, Cancel: %s",
          current_force_abs, max_force,
          (current_force_abs >= max_force) ? "NO" : "YES",
          cancel_requested_.load() ? "TRUE" : "FALSE");
    }

    if (current_force_abs >= max_force) {
      RCLCPP_INFO(this->get_logger(),
                  "Maximum force limit reached! Current force: %.1f",
                  current_force_abs);
      stop_robot_and_switch_back();
      return true;
    }

    double force_margin = max_force - current_force_abs;
    double desired_speed = std::min(force_gain * force_margin, max_speed);

    if (desired_speed < min_speed) {
      desired_speed = min_speed;
    }

    tf2::Vector3 linear_velocity_ee = direction_ee * desired_speed;

    geometry_msgs::msg::Twist twist_cmd;
    twist_cmd.linear.x = linear_velocity_ee.x();
    twist_cmd.linear.y = linear_velocity_ee.y();
    twist_cmd.linear.z = linear_velocity_ee.z();
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    // twist_cmd.angular.z = 0.0;

    if (goal->rotation_angular_speed > 0.0) {
      double angular_vel =
          goal->rotation_angular_speed * goal->pull_rotation_direction;
      twist_cmd.angular.z = angular_vel;
    } else {
      twist_cmd.angular.z = 0.0;
    }

    twist_publisher_->publish(twist_cmd);

    static int debug_counter = 0;
    if (++debug_counter % 20 == 0) {
      RCLCPP_INFO(this->get_logger(), "Force: %.1f/%.1f | Speed: %.4f",
                  current_force_abs, max_force, desired_speed);
    }
    rate.sleep();
  }

  stop_robot_and_switch_back();
  return false;
}

bool KinovaMotionServer::get_current_end_effector_pose(
    geometry_msgs::msg::Pose& pose) {
  try {
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("base_link", "end_effector_link",
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(0.2));
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return true;
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "TF lookup for pose failed: %s",
                 ex.what());
    return false;
  }
}

bool KinovaMotionServer::switch_to_mode(const std::string& mode) {
  if (!switcher_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service /switch_control_mode not available.");
    return false;
  }
  auto request =
      std::make_shared<kinova_interfaces::srv::SwitchControlMode::Request>();
  request->mode = mode;
  auto future = switcher_client_->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service call to /switch_control_mode failed (timeout).");
    return false;
  }
  return future.get()->success;
}

void KinovaMotionServer::stop_robot_and_switch_back() {
  RCLCPP_INFO(this->get_logger(),
              "Stopping robot and returning to 'position' mode.");
  geometry_msgs::msg::Twist zero_twist;
  twist_publisher_->publish(zero_twist);
  std::this_thread::sleep_for(200ms);
  twist_publisher_->publish(zero_twist);
  switch_to_mode("position");
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto motion_server =
      std::make_shared<KinovaMotionServer>(rclcpp::NodeOptions());
  executor.add_node(motion_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
