// INCLUDES
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <kinova_action_interfaces/action/kinova_command.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

// CONSTANTS
constexpr double MAX_REACH = 0.85;   // Maximum reach in meters
constexpr double MIN_HEIGHT = 0.05;  // Minimum height from plane in meters
constexpr double MAX_HEIGHT = 1.1;   // Maximum height in meters

// Command types
constexpr int ROTATE_GRIPPER = 1;
constexpr int MOVE_TO_POSITION = 2;
constexpr int TRANSLATE_Z = 3;
constexpr int SET_ORIENTATION = 4;
constexpr int GO_HOME = 5;
constexpr int ROTO_TRANSLATION = 6;  // Command for roto-translation
constexpr int MOVE_WITH_ORIENTATION =
    7;  // Command for movement with orientation

/*
 * Note: For this to work, the
 * kinova_action_interfaces/action/KinovaCommand.action file should be updated
 * to include velocity and acceleration parameters:
 *
 * # Request
 * int32 command_type
 * geometry_msgs/Point position
 * float64 gripper_rotation
 * float64 z_translation
 * geometry_msgs/Point orientation_target
 * float64 velocity_scaling         # New field for custom velocity scaling
 * float64 acceleration_scaling     # New field for custom acceleration scaling
 * ---
 * # Result
 * bool success
 * ---
 * # Feedback
 * string status
 */

// UTILITY FUNCTIONS
// ----------------------------------------------------------------------------------------------

// Convert degrees to radians
double degToRad(double deg) { return deg * M_PI / 180.0; }

// Convert radians to degrees
double radToDeg(double rad) { return rad * 180.0 / M_PI; }

// Validate target position is within workspace
bool validateTargetPosition(const geometry_msgs::msg::Point& target,
                            const rclcpp::Logger& logger) {
  double distance = std::sqrt(target.x * target.x + target.y * target.y);

  if (distance > MAX_REACH) {
    RCLCPP_ERROR(logger, "Target position beyond maximum reach (%.2f m)",
                 MAX_REACH);
    return false;
  }

  if (target.z < MIN_HEIGHT || target.z > MAX_HEIGHT) {
    RCLCPP_ERROR(logger, "Target height outside limits (%.2f - %.2f m)",
                 MIN_HEIGHT, MAX_HEIGHT);
    return false;
  }

  return true;
}

// Wait for robot connection
bool waitForRobotConnection(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>&
        move_group,
    const rclcpp::Logger& logger, int max_attempts = 30,
    int wait_milliseconds = 1000) {
  RCLCPP_INFO(logger, "Waiting for robot connection...");

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    try {
      auto current_state = move_group->getCurrentState(10.0);
      if (current_state) {
        auto joint_model_group =
            current_state->getJointModelGroup("manipulator");
        if (joint_model_group) {
          RCLCPP_INFO(logger, "Robot successfully connected!");
          return true;
        }
      }

      RCLCPP_WARN(logger, "Attempt %d/%d: Waiting for connection...",
                  attempt + 1, max_attempts);
      rclcpp::sleep_for(std::chrono::milliseconds(wait_milliseconds));

    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "Connection attempt error: %s", e.what());
      rclcpp::sleep_for(std::chrono::milliseconds(wait_milliseconds));
    }
  }

  RCLCPP_ERROR(logger, "Failed to connect to robot after %d attempts",
               max_attempts);
  return false;
}

// Calculate gripper orientation towards a point
geometry_msgs::msg::Quaternion calculateGripperOrientation(
    const geometry_msgs::msg::Point& start_point,
    const geometry_msgs::msg::Point& target_point,
    const rclcpp::Logger& logger) {
  // Calculate direction vector from start_point to target_point
  tf2::Vector3 direction(target_point.x - start_point.x,
                         target_point.y - start_point.y,
                         target_point.z - start_point.z);

  double length = direction.length();
  if (length < 1e-6) {
    RCLCPP_WARN(logger, "Points too close, using default orientation");
    return tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
  }
  direction.normalize();

  tf2::Vector3 z_axis = direction;
  tf2::Vector3 up(0, 0, 1);
  tf2::Vector3 x_axis = up.cross(z_axis);

  if (x_axis.length() < 1e-6) {
    up = tf2::Vector3(0, 1, 0);
    x_axis = up.cross(z_axis);
  }
  x_axis.normalize();

  tf2::Vector3 y_axis = z_axis.cross(x_axis);
  y_axis.normalize();

  tf2::Matrix3x3 rotation(x_axis.x(), y_axis.x(), z_axis.x(), x_axis.y(),
                          y_axis.y(), z_axis.y(), x_axis.z(), y_axis.z(),
                          z_axis.z());

  tf2::Quaternion q;
  rotation.getRotation(q);

  return tf2::toMsg(q);
}

// Define home position parameters
geometry_msgs::msg::Pose getHomePosition() {
  geometry_msgs::msg::Pose home;

  // Set position
  home.position.x = 0.454;
  home.position.y = 0.001;
  home.position.z = 0.423;

  // Set orientation (quaternion)
  home.orientation.x = 0.5001;
  home.orientation.y = 0.5001;
  home.orientation.z = 0.4999;
  home.orientation.w = 0.4999;

  return home;
}

// NODE CLASS
// ----------------------------------------------------------------------------------------------

class KinovaActionServer : public rclcpp::Node {
 public:
  explicit KinovaActionServer(const rclcpp::NodeOptions& options)
      : Node("kinova_action_server", options), logger_(get_logger()) {
    RCLCPP_INFO(logger_, "Initializing Kinova Action Server...");

    // Get parameters
    velocity_scaling_ = this->get_parameter_or("velocity_scaling", 0.1);
    acceleration_scaling_ = this->get_parameter_or("acceleration_scaling", 0.1);
    double planning_time = this->get_parameter_or("planning_time", 5.0);
    int num_planning_attempts =
        this->get_parameter_or("num_planning_attempts", 10);

    // Create MoveGroup
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](auto*) {}), "manipulator");

    // Wait for robot connection
    if (!waitForRobotConnection(move_group_, logger_)) {
      throw std::runtime_error("Failed to connect to robot");
    }

    // Setup MoveGroup
    setupMoveGroup(velocity_scaling_, acceleration_scaling_, planning_time,
                   num_planning_attempts);

    // Create the action server
    action_server_ = rclcpp_action::create_server<KinovaAction>(
        this, "kinova_command",
        std::bind(&KinovaActionServer::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&KinovaActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&KinovaActionServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(logger_, "Kinova Action Server initialized and ready");
  }

 private:
  // MoveGroup interface
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // Action server
  using KinovaAction = kinova_action_interfaces::action::KinovaCommand;
  using GoalHandleKinovaAction = rclcpp_action::ServerGoalHandle<KinovaAction>;
  rclcpp_action::Server<KinovaAction>::SharedPtr action_server_;

  // Logger
  rclcpp::Logger logger_;

  // Movement parameters
  double velocity_scaling_ = 0.1;
  double acceleration_scaling_ = 0.1;

  // Setup MoveGroup configuration
  void setupMoveGroup(double vel_scaling, double acc_scaling,
                      double planning_time, int planning_attempts) {
    move_group_->setMaxVelocityScalingFactor(vel_scaling);
    move_group_->setMaxAccelerationScalingFactor(acc_scaling);
    move_group_->setPlanningTime(planning_time);
    move_group_->setNumPlanningAttempts(planning_attempts);
    move_group_->setGoalPositionTolerance(0.001);    // 1mm
    move_group_->setGoalOrientationTolerance(0.01);  // ~0.57 degrees

    RCLCPP_INFO(logger_, "MoveGroup Configuration:");
    RCLCPP_INFO(logger_, "Planning Frame: %s",
                move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(logger_, "End Effector Link: %s",
                move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(logger_, "Default Velocity scaling: %.2f", vel_scaling);
    RCLCPP_INFO(logger_, "Default Acceleration scaling: %.2f", acc_scaling);
  }

  // Goal handling
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const KinovaAction::Goal> goal) {
    (void)uuid;  // Unused

    int command_type = goal->command_type;
    RCLCPP_INFO(logger_, "Received goal request with command type: %d",
                command_type);

    // Check for custom velocity and acceleration settings
    if (goal->velocity_scaling > 0.0 || goal->acceleration_scaling > 0.0) {
      RCLCPP_INFO(
          logger_,
          "Custom scaling provided - Velocity: %.2f, Acceleration: %.2f",
          (goal->velocity_scaling > 0.0) ? goal->velocity_scaling
                                         : velocity_scaling_,
          (goal->acceleration_scaling > 0.0) ? goal->acceleration_scaling
                                             : acceleration_scaling_);
    } else if (goal->velocity_scaling == -1.0) {
      RCLCPP_INFO(logger_, "Full speed mode requested for command type: %d",
                  command_type);
    }

    // Validate command type
    if (command_type != ROTATE_GRIPPER && command_type != MOVE_TO_POSITION &&
        command_type != TRANSLATE_Z && command_type != SET_ORIENTATION &&
        command_type != GO_HOME && command_type != ROTO_TRANSLATION &&
        command_type != MOVE_WITH_ORIENTATION) {
      RCLCPP_ERROR(logger_, "Unknown command type: %d", command_type);
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Cancel handling
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle) {
    RCLCPP_INFO(logger_, "Received request to cancel goal");
    move_group_->stop();
    (void)goal_handle;  // Unused
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Handle accepted goal
  void handle_accepted(
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle) {
    // Start a new thread to execute the goal
    std::thread{
        std::bind(&KinovaActionServer::execute_command, this, goal_handle)}
        .detach();
  }

  // Execute the command
  void execute_command(
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<KinovaAction::Result>();
    auto feedback = std::make_shared<KinovaAction::Feedback>();
    result->success = false;

    try {
      int command_type = goal->command_type;

      // Execute the appropriate command based on type
      switch (command_type) {
        case ROTATE_GRIPPER:
          feedback->status = "Executing gripper rotation...";
          goal_handle->publish_feedback(feedback);
          result->success = execute_gripper_rotation(goal->gripper_rotation,
                                                     goal_handle, feedback);
          break;

        case MOVE_TO_POSITION:
          feedback->status = "Executing move to position...";
          goal_handle->publish_feedback(feedback);
          result->success =
              execute_cartesian_move(goal->position, goal_handle, feedback);
          break;

        case TRANSLATE_Z:
          feedback->status = "Executing Z-axis translation...";
          goal_handle->publish_feedback(feedback);
          result->success =
              execute_z_translation(goal->z_translation, goal_handle, feedback);
          break;

        case SET_ORIENTATION:
          feedback->status = "Setting orientation towards point...";
          goal_handle->publish_feedback(feedback);
          result->success =
              execute_set_orientation(goal->position, goal_handle, feedback);
          break;

        case GO_HOME:
          feedback->status = "Moving to home position...";
          goal_handle->publish_feedback(feedback);
          result->success = execute_go_home(goal_handle, feedback);
          break;

        case ROTO_TRANSLATION:
          feedback->status = "Executing roto-translation...";
          goal_handle->publish_feedback(feedback);
          result->success = execute_roto_translation(goal->gripper_rotation,
                                                     goal->z_translation,
                                                     goal_handle, feedback);
          break;

        case MOVE_WITH_ORIENTATION:
          feedback->status = "Executing movement with orientation...";
          goal_handle->publish_feedback(feedback);
          result->success = execute_move_with_orientation(
              goal->position,            // Destination position
              goal->orientation_target,  // Orientation reference point
              goal_handle, feedback);
          break;

        default:
          RCLCPP_ERROR(logger_, "Invalid command type %d", command_type);
          feedback->status = "Invalid command type";
          goal_handle->publish_feedback(feedback);
          result->success = false;
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Command execution failed: %s", e.what());
      result->success = false;
      feedback->status =
          "Command execution failed with error: " + std::string(e.what());
      goal_handle->publish_feedback(feedback);
    }

    // Finish the action
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
    }
  }

  // Execute gripper rotation by specified angle - MODIFIED FOR CUSTOM VELOCITY
  bool execute_gripper_rotation(
      double rotation_degrees,
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle,
      std::shared_ptr<KinovaAction::Feedback> feedback) {
    const auto goal = goal_handle->get_goal();

    // Use custom velocity and acceleration scaling if provided
    double vel_scale = (goal->velocity_scaling > 0.0) ? goal->velocity_scaling
                                                      : velocity_scaling_;
    double acc_scale = (goal->acceleration_scaling > 0.0)
                           ? goal->acceleration_scaling
                           : acceleration_scaling_;

    RCLCPP_INFO(logger_,
                "Executing gripper rotation of %.2f degrees with velocity "
                "scale: %.2f, acceleration scale: %.2f",
                rotation_degrees, vel_scale, acc_scale);

    // Get the joint names
    const std::vector<std::string>& joint_names = move_group_->getJointNames();

    // Find joint 6 (usually the last one)
    std::string gripper_joint;
    size_t joint_index = 0;
    bool joint_found = false;

    for (size_t i = 0; i < joint_names.size(); i++) {
      const auto& joint = joint_names[i];
      if (joint.find("joint_6") != std::string::npos ||
          joint.find("j2n6") != std::string::npos ||
          joint.find("joint6") != std::string::npos) {
        gripper_joint = joint;
        joint_index = i;
        joint_found = true;
        break;
      }
    }

    if (!joint_found) {
      RCLCPP_ERROR(logger_, "Could not find gripper joint (joint_6)");
      feedback->status = "Failed to find gripper joint";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    RCLCPP_INFO(logger_, "Found gripper joint: %s at index %zu",
                gripper_joint.c_str(), joint_index);

    // Get current joint positions for ALL joints
    std::vector<double> joint_values = move_group_->getCurrentJointValues();

    // Sanity check
    if (joint_values.size() <= joint_index) {
      RCLCPP_ERROR(logger_, "Joint index is out of bounds (%zu >= %zu)",
                   joint_index, joint_values.size());
      feedback->status = "Internal error: joint index out of bounds";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    double current_angle = joint_values[joint_index];
    RCLCPP_INFO(logger_, "Current gripper angle: %.2f degrees",
                radToDeg(current_angle));

    // Calculate target angle (current + rotation)
    double target_angle_rad = current_angle + degToRad(rotation_degrees);

    // Set the target for joint 6 only, while preserving all other joint values
    joint_values[joint_index] = target_angle_rad;

    // Set the ENTIRE joint configuration as target to prevent other joints from
    // moving
    move_group_->setJointValueTarget(joint_values);

    // Plan and execute
    feedback->status = "Planning gripper rotation...";
    goal_handle->publish_feedback(feedback);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = static_cast<bool>(move_group_->plan(plan));

    if (!plan_success) {
      RCLCPP_ERROR(logger_, "Planning failed for rotation to %.2f degrees",
                   radToDeg(target_angle_rad));
      feedback->status = "Planning failed for rotation";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    // Apply custom velocity and acceleration scaling
    robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                         "manipulator");
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), plan.trajectory_);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, vel_scale, acc_scale);  // Using custom values

    rt.getRobotTrajectoryMsg(plan.trajectory_);

    feedback->status = "Executing gripper rotation...";
    goal_handle->publish_feedback(feedback);

    auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Execution failed for rotation to %.2f degrees",
                   radToDeg(target_angle_rad));
      feedback->status = "Execution failed for rotation";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    RCLCPP_INFO(logger_, "Completed rotation to %.2f degrees",
                radToDeg(target_angle_rad));
    feedback->status = "Gripper rotation completed successfully";
    goal_handle->publish_feedback(feedback);

    return true;
  }

  // Execute cartesian position movement - NO VELOCITY CUSTOMIZATION REQUESTED
  bool execute_cartesian_move(
      const geometry_msgs::msg::Point& target_point,
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle,
      std::shared_ptr<KinovaAction::Feedback> feedback) {
    RCLCPP_INFO(logger_,
                "Executing cartesian move to position: (%.3f, %.3f, %.3f)",
                target_point.x, target_point.y, target_point.z);

    // Validate target position
    if (!validateTargetPosition(target_point, logger_)) {
      feedback->status = "Target position out of range";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    // Get current pose to maintain orientation
    auto current_pose = move_group_->getCurrentPose().pose;

    // Create target pose (new position, current orientation)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = target_point;
    target_pose.orientation = current_pose.orientation;

    RCLCPP_INFO(logger_, "Maintaining current orientation during movement");

    // Create a cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;  // disable jump threshold
    const double eef_step = 0.01;       // 1cm resolution

    feedback->status = "Planning cartesian path...";
    goal_handle->publish_feedback(feedback);

    // Plan a cartesian path
    double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.98) {  // Path planned to at least 98% is considered valid
      RCLCPP_INFO(logger_, "Cartesian path planned: %.1f%% complete",
                  fraction * 100.0);

      // Slow down the trajectory - using default values
      robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                           "manipulator");
      rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

      // Apply scaling to trajectory
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      iptp.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);

      // Convert back to ROS message
      rt.getRobotTrajectoryMsg(trajectory);

      // Create the plan
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      feedback->status = "Executing cartesian movement...";
      goal_handle->publish_feedback(feedback);

      auto error_code = move_group_->execute(plan);

      if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Execution failed for cartesian movement");
        feedback->status = "Execution failed for cartesian movement";
        goal_handle->publish_feedback(feedback);
        return false;
      }

      feedback->status = "Cartesian movement completed successfully";
      goal_handle->publish_feedback(feedback);
      return true;
    } else {
      // If cartesian path fails, try with inverse kinematics
      RCLCPP_WARN(logger_, "Cartesian path incomplete (%.1f%%), trying with IK",
                  fraction * 100.0);
      feedback->status = "Cartesian planning incomplete, using IK...";
      goal_handle->publish_feedback(feedback);

      move_group_->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = static_cast<bool>(move_group_->plan(plan));

      if (!plan_success) {
        RCLCPP_ERROR(logger_, "IK planning failed");
        feedback->status = "IK planning failed";
        goal_handle->publish_feedback(feedback);
        return false;
      }

      // Slow down the IK trajectory - using default values
      robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                           "manipulator");
      rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(),
                               plan.trajectory_);

      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      iptp.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);

      rt.getRobotTrajectoryMsg(plan.trajectory_);

      feedback->status = "Executing IK movement...";
      goal_handle->publish_feedback(feedback);

      auto error_code = move_group_->execute(plan);

      if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Execution failed for IK movement");
        feedback->status = "Execution failed for IK movement";
        goal_handle->publish_feedback(feedback);
        return false;
      }

      feedback->status = "IK movement completed successfully";
      goal_handle->publish_feedback(feedback);
      return true;
    }
  }

  // Execute Z-axis translation along gripper direction - MODIFIED FOR CUSTOM
  // VELOCITY AND FULL SPEED
  bool execute_z_translation(
      double z_distance,
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle,
      std::shared_ptr<KinovaAction::Feedback> feedback) {
    const auto goal = goal_handle->get_goal();

    // Check if we should use full speed direct mode
    bool use_direct_mode = false;
    double vel_scale = velocity_scaling_;
    double acc_scale = acceleration_scaling_;

    if (goal->velocity_scaling == -1.0) {
      use_direct_mode = true;
      RCLCPP_INFO(
          logger_,
          "Using DIRECT MODE for Z-axis translation: %.3f meters at FULL SPEED",
          z_distance);
    } else {
      // Use custom velocity and acceleration scaling if provided
      vel_scale = (goal->velocity_scaling > 0.0) ? goal->velocity_scaling
                                                 : velocity_scaling_;
      acc_scale = (goal->acceleration_scaling > 0.0)
                      ? goal->acceleration_scaling
                      : acceleration_scaling_;

      RCLCPP_INFO(logger_,
                  "Executing gripper Z-axis translation: %.3f meters with "
                  "velocity scale: %.2f, acceleration scale: %.2f",
                  z_distance, vel_scale, acc_scale);
    }

    // Get current pose
    auto current_pose = move_group_->getCurrentPose().pose;

    // Extract the gripper's Z axis direction from the orientation
    tf2::Quaternion orientation;
    tf2::fromMsg(current_pose.orientation, orientation);
    tf2::Matrix3x3 rot_matrix(orientation);

    // The Z axis of the gripper is the third column of the rotation matrix
    tf2::Vector3 z_axis(rot_matrix[0][2], rot_matrix[1][2], rot_matrix[2][2]);

    // Calculate new position by moving along Z axis
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += z_axis.x() * z_distance;
    target_pose.position.y += z_axis.y() * z_distance;
    target_pose.position.z += z_axis.z() * z_distance;

    // Validate the new position
    geometry_msgs::msg::Point target_point = target_pose.position;
    if (!validateTargetPosition(target_point, logger_)) {
      feedback->status = "Z-translation target position out of range";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    // Create a cartesian path for smooth motion
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;  // disable jump threshold
    const double eef_step = 0.01;       // 1cm resolution

    feedback->status = "Planning Z-translation path...";
    goal_handle->publish_feedback(feedback);

    // Plan a cartesian path
    double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.98) {  // Path planned to at least 98% is considered valid
      RCLCPP_INFO(logger_, "Z-translation path planned: %.1f%% complete",
                  fraction * 100.0);

      // Check if using direct mode (full speed) or normal mode with time
      // parameterization
      if (!use_direct_mode) {
        // Slow down the trajectory using custom scaling
        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                             "manipulator");
        rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

        // Apply custom scaling to trajectory
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt, vel_scale, acc_scale);

        // Convert back to ROS message
        rt.getRobotTrajectoryMsg(trajectory);
      } else {
        RCLCPP_INFO(
            logger_,
            "Skipping time parameterization for direct movement at full speed");
      }

      // Create the plan
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      feedback->status = "Executing Z-translation movement...";
      goal_handle->publish_feedback(feedback);

      auto error_code = move_group_->execute(plan);

      if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Execution failed for Z-translation");
        feedback->status = "Execution failed for Z-translation";
        goal_handle->publish_feedback(feedback);
        return false;
      }

      feedback->status = "Z-translation completed successfully";
      goal_handle->publish_feedback(feedback);
      return true;
    } else {
      // If cartesian path fails, try with inverse kinematics
      RCLCPP_WARN(logger_,
                  "Z-translation path incomplete (%.1f%%), trying with IK",
                  fraction * 100.0);
      feedback->status = "Z-translation planning incomplete, using IK...";
      goal_handle->publish_feedback(feedback);

      move_group_->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = static_cast<bool>(move_group_->plan(plan));

      if (!plan_success) {
        RCLCPP_ERROR(logger_, "IK planning failed for Z-translation");
        feedback->status = "IK planning failed for Z-translation";
        goal_handle->publish_feedback(feedback);
        return false;
      }

      // Apply custom velocity scaling to the IK trajectory
      if (!use_direct_mode) {
        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                             "manipulator");
        rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(),
                                 plan.trajectory_);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt, vel_scale, acc_scale);

        rt.getRobotTrajectoryMsg(plan.trajectory_);
      } else {
        RCLCPP_INFO(logger_,
                    "Skipping time parameterization for direct IK movement at "
                    "full speed");
      }

      feedback->status = "Executing IK-based Z-translation...";
      goal_handle->publish_feedback(feedback);

      auto error_code = move_group_->execute(plan);

      if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Execution failed for IK-based Z-translation");
        feedback->status = "Execution failed for IK-based Z-translation";
        goal_handle->publish_feedback(feedback);
        return false;
      }

      feedback->status = "IK-based Z-translation completed successfully";
      goal_handle->publish_feedback(feedback);
      return true;
    }
  }

  // Execute set orientation with target point - NO VELOCITY CUSTOMIZATION
  // REQUESTED
  bool execute_set_orientation(
      const geometry_msgs::msg::Point& target_point,
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle,
      std::shared_ptr<KinovaAction::Feedback> feedback) {
    RCLCPP_INFO(logger_,
                "Setting orientation towards point: (%.3f, %.3f, %.3f)",
                target_point.x, target_point.y, target_point.z);

    // Get current position (keep position but change orientation)
    auto current_pose = move_group_->getCurrentPose().pose;
    geometry_msgs::msg::Point current_position = current_pose.position;

    // Create target pose (same position, new orientation)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = current_position;

    // Calculate new orientation from current position to target point
    target_pose.orientation =
        calculateGripperOrientation(current_position, target_point, logger_);

    feedback->status = "Planning orientation change...";
    goal_handle->publish_feedback(feedback);

    // Set pose target
    move_group_->setPoseTarget(target_pose);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = static_cast<bool>(move_group_->plan(plan));

    if (!plan_success) {
      RCLCPP_ERROR(logger_, "Planning failed for orientation change");
      feedback->status = "Planning failed for orientation change";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    // Apply velocity scaling - using default values
    robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                         "manipulator");
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), plan.trajectory_);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);

    rt.getRobotTrajectoryMsg(plan.trajectory_);

    feedback->status = "Executing orientation change...";
    goal_handle->publish_feedback(feedback);

    auto error_code = move_group_->execute(plan);

    if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Execution failed for orientation change");
      feedback->status = "Execution failed for orientation change";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    feedback->status = "Orientation change completed successfully";
    goal_handle->publish_feedback(feedback);
    return true;
  }

  // Execute go to home position - NO VELOCITY CUSTOMIZATION REQUESTED
  bool execute_go_home(
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle,
      std::shared_ptr<KinovaAction::Feedback> feedback) {
    RCLCPP_INFO(logger_, "Executing go to home position");

    feedback->status = "Planning movement to home position...";
    goal_handle->publish_feedback(feedback);

    // Get the home position
    geometry_msgs::msg::Pose home_pose = getHomePosition();

    // Set pose target
    move_group_->setPoseTarget(home_pose);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = static_cast<bool>(move_group_->plan(plan));

    if (!plan_success) {
      RCLCPP_ERROR(logger_, "Planning failed for home position");
      feedback->status = "Planning failed for home position";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    // Apply velocity scaling - using default values
    robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                         "manipulator");
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), plan.trajectory_);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);

    rt.getRobotTrajectoryMsg(plan.trajectory_);

    feedback->status = "Executing movement to home position...";
    goal_handle->publish_feedback(feedback);

    auto error_code = move_group_->execute(plan);

    if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Execution failed for home position");
      feedback->status = "Execution failed for home position";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    feedback->status = "Movement to home position completed successfully";
    goal_handle->publish_feedback(feedback);
    return true;
  }

  // Execute roto-translation as a combined movement - MODIFIED FOR CUSTOM
  // VELOCITY
  bool execute_roto_translation(
      double rotation_degrees, double z_distance,
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle,
      std::shared_ptr<KinovaAction::Feedback> feedback) {
    const auto goal = goal_handle->get_goal();

    // Use custom velocity and acceleration scaling if provided
    double vel_scale = (goal->velocity_scaling > 0.0) ? goal->velocity_scaling
                                                      : velocity_scaling_;
    double acc_scale = (goal->acceleration_scaling > 0.0)
                           ? goal->acceleration_scaling
                           : acceleration_scaling_;

    // Check for direct mode (full speed)
    bool use_direct_mode = (goal->velocity_scaling == -1.0);

    if (use_direct_mode) {
      RCLCPP_INFO(logger_,
                  "Executing combined roto-translation at FULL SPEED: rotation "
                  "of %.2f degrees, Z-translation of %.3f meters",
                  rotation_degrees, z_distance);
    } else {
      RCLCPP_INFO(logger_,
                  "Executing combined roto-translation: rotation of %.2f "
                  "degrees, Z-translation of %.3f meters with velocity scale: "
                  "%.2f, acceleration scale: %.2f",
                  rotation_degrees, z_distance, vel_scale, acc_scale);
    }

    // 1. Get current robot state and pose
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
    auto current_pose = move_group_->getCurrentPose().pose;

    // 2. Calculate new position after Z translation
    tf2::Quaternion orientation;
    tf2::fromMsg(current_pose.orientation, orientation);
    tf2::Matrix3x3 rot_matrix(orientation);
    tf2::Vector3 z_axis(rot_matrix[0][2], rot_matrix[1][2], rot_matrix[2][2]);

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += z_axis.x() * z_distance;
    target_pose.position.y += z_axis.y() * z_distance;
    target_pose.position.z += z_axis.z() * z_distance;

    // Validate the resulting position
    if (!validateTargetPosition(target_pose.position, logger_)) {
      feedback->status =
          "Target position after translation would be out of workspace";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    // 3. Generate intermediate waypoints combining rotation and translation
    std::vector<geometry_msgs::msg::Pose> waypoints;
    int steps = 10;  // Number of intermediate steps

    for (int i = 1; i <= steps; i++) {
      double fraction = static_cast<double>(i) / steps;

      // Intermediate position (linear interpolation)
      geometry_msgs::msg::Pose intermediate_pose;
      intermediate_pose.position.x =
          current_pose.position.x +
          (target_pose.position.x - current_pose.position.x) * fraction;
      intermediate_pose.position.y =
          current_pose.position.y +
          (target_pose.position.y - current_pose.position.y) * fraction;
      intermediate_pose.position.z =
          current_pose.position.z +
          (target_pose.position.z - current_pose.position.z) * fraction;

      // Keep orientation constant for cartesian path
      intermediate_pose.orientation = current_pose.orientation;
      waypoints.push_back(intermediate_pose);
    }

    // 4. Plan cartesian path
    feedback->status = "Planning combined roto-translation path...";
    goal_handle->publish_feedback(feedback);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double path_fraction =
        move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (path_fraction < 0.9) {
      RCLCPP_ERROR(logger_, "Unable to plan complete cartesian path (%.2f%%)",
                   path_fraction * 100.0);
      feedback->status = "Combined path planning failed";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    // 5. Modify trajectory to include gripper rotation
    robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                         "manipulator");
    rt.setRobotTrajectoryMsg(*current_state, trajectory);

    // Find gripper joint index
    const std::vector<std::string>& joint_names = move_group_->getJointNames();
    size_t gripper_joint_index = 0;
    bool found = false;

    for (size_t i = 0; i < joint_names.size(); i++) {
      const auto& joint = joint_names[i];
      if (joint.find("joint_6") != std::string::npos ||
          joint.find("j2n6") != std::string::npos ||
          joint.find("joint6") != std::string::npos) {
        gripper_joint_index = i;
        found = true;
        break;
      }
    }

    if (!found) {
      RCLCPP_ERROR(logger_, "Could not find gripper joint");
      feedback->status = "Could not find gripper joint";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    // Get current joint values
    std::vector<double> joint_values = move_group_->getCurrentJointValues();
    double start_angle = joint_values[gripper_joint_index];
    double target_angle = start_angle + degToRad(rotation_degrees);

    // Apply gradual rotation along the trajectory
    for (size_t i = 0; i < rt.getWayPointCount(); i++) {
      moveit::core::RobotStatePtr waypoint = rt.getWayPointPtr(i);
      double fraction = static_cast<double>(i) / (rt.getWayPointCount() - 1);
      double new_angle = start_angle + (target_angle - start_angle) * fraction;

      std::vector<double> joint_positions;
      waypoint->copyJointGroupPositions("manipulator", joint_positions);
      joint_positions[gripper_joint_index] = new_angle;
      waypoint->setJointGroupPositions("manipulator", joint_positions);
    }

    // 6. Apply time parameterization based on mode
    feedback->status = "Optimizing combined movement trajectory...";
    goal_handle->publish_feedback(feedback);

    if (!use_direct_mode) {
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      iptp.computeTimeStamps(rt, vel_scale, acc_scale);  // Using custom values
    } else {
      RCLCPP_INFO(logger_,
                  "Skipping time parameterization for direct roto-translation "
                  "at full speed");
    }

    // 7. Convert to plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    rt.getRobotTrajectoryMsg(trajectory);
    plan.trajectory_ = trajectory;

    feedback->status = "Executing combined roto-translation movement...";
    goal_handle->publish_feedback(feedback);

    auto error_code = move_group_->execute(plan);

    if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Execution failed for combined movement");
      feedback->status = "Execution failed for combined movement";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    feedback->status = "Combined roto-translation completed successfully";
    goal_handle->publish_feedback(feedback);
    return true;
  }

  // Execute movement to position with specific orientation - NO VELOCITY
  // CUSTOMIZATION REQUESTED
  bool execute_move_with_orientation(
      const geometry_msgs::msg::Point& destination_point,
      const geometry_msgs::msg::Point& orientation_target,
      const std::shared_ptr<GoalHandleKinovaAction> goal_handle,
      std::shared_ptr<KinovaAction::Feedback> feedback) {
    RCLCPP_INFO(logger_,
                "Executing move to position (%.3f, %.3f, %.3f) with "
                "orientation towards (%.3f, %.3f, %.3f)",
                destination_point.x, destination_point.y, destination_point.z,
                orientation_target.x, orientation_target.y,
                orientation_target.z);

    // Validate the destination position
    if (!validateTargetPosition(destination_point, logger_)) {
      feedback->status = "Destination position out of range";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    // 1. Get current robot state
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
    auto current_pose = move_group_->getCurrentPose().pose;

    // 2. Create target pose with new position and calculated orientation
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = destination_point;

    // Calculate the orientation to point the gripper from destination to
    // orientation_target
    target_pose.orientation = calculateGripperOrientation(
        destination_point,   // From the destination point...
        orientation_target,  // ...towards the orientation reference point
        logger_);

    RCLCPP_INFO(logger_,
                "Calculated target orientation: (%.3f, %.3f, %.3f, %.3f)",
                target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w);

    feedback->status = "Planning combined position and orientation movement...";
    goal_handle->publish_feedback(feedback);

    // 3. Create waypoints for smooth transition
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // We'll interpolate position and also rotate gradually to the target
    // orientation
    int steps = 10;

    for (int i = 1; i <= steps; i++) {
      double fraction = static_cast<double>(i) / steps;

      // Intermediate position (linear interpolation)
      geometry_msgs::msg::Pose intermediate_pose;
      intermediate_pose.position.x =
          current_pose.position.x +
          (destination_point.x - current_pose.position.x) * fraction;
      intermediate_pose.position.y =
          current_pose.position.y +
          (destination_point.y - current_pose.position.y) * fraction;
      intermediate_pose.position.z =
          current_pose.position.z +
          (destination_point.z - current_pose.position.z) * fraction;

      // For final waypoint, use target orientation exactly
      if (i == steps) {
        intermediate_pose.orientation = target_pose.orientation;
      } else {
        // For intermediate waypoints, keep current orientation
        // This approach works better with cartesian planning
        intermediate_pose.orientation = current_pose.orientation;
      }

      waypoints.push_back(intermediate_pose);
    }

    // 4. Attempt cartesian planning first
    moveit_msgs::msg::RobotTrajectory trajectory;
    double path_fraction =
        move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    bool success = false;

    if (path_fraction > 0.9) {
      // Cartesian path planning was successful
      RCLCPP_INFO(logger_, "Cartesian path planned: %.1f%% complete",
                  path_fraction * 100.0);

      // Slow down the trajectory - using default values
      robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                           "manipulator");
      rt.setRobotTrajectoryMsg(*current_state, trajectory);

      // Apply scaling to trajectory
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      iptp.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);

      // Convert back to ROS message
      rt.getRobotTrajectoryMsg(trajectory);

      // Create the plan
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      feedback->status = "Executing cartesian movement with orientation...";
      goal_handle->publish_feedback(feedback);

      auto error_code = move_group_->execute(plan);
      success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      // Cartesian planning failed, try direct IK solution
      RCLCPP_WARN(
          logger_,
          "Cartesian path planning incomplete (%.1f%%), trying direct IK",
          path_fraction * 100.0);

      // Set the target pose directly
      move_group_->setPoseTarget(target_pose);

      feedback->status = "Planning direct IK movement...";
      goal_handle->publish_feedback(feedback);

      // Plan and execute
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = static_cast<bool>(move_group_->plan(plan));

      if (!plan_success) {
        RCLCPP_ERROR(logger_, "Direct IK planning failed");
        feedback->status = "Direct IK planning failed";
        goal_handle->publish_feedback(feedback);
        return false;
      }

      // Apply velocity scaling - using default values
      robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(),
                                           "manipulator");
      rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(),
                               plan.trajectory_);

      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      iptp.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);

      rt.getRobotTrajectoryMsg(plan.trajectory_);

      feedback->status = "Executing direct IK movement...";
      goal_handle->publish_feedback(feedback);

      auto error_code = move_group_->execute(plan);
      success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);
    }

    if (!success) {
      RCLCPP_ERROR(logger_, "Execution failed for combined movement");
      feedback->status = "Execution failed for combined movement";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    feedback->status =
        "Combined position and orientation movement completed successfully";
    goal_handle->publish_feedback(feedback);
    return true;
  }
};

// MAIN FUNCTION
// ----------------------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({{"use_sim_time", true}});

  try {
    auto node = std::make_shared<KinovaActionServer>(options);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Kinova Action Server started");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Available commands:");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "1: Rotate gripper (joint 6) by specific angle - supports "
                "custom velocity");
    RCLCPP_INFO(
        rclcpp::get_logger("main"),
        "2: Move to cartesian position (maintains current orientation)");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "3: Translate along gripper Z axis - supports custom velocity "
                "and full speed mode");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "4: Set gripper orientation using a target point");
    RCLCPP_INFO(rclcpp::get_logger("main"), "5: Go to home position");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "6: Roto-translation (rotation + Z-translation) - supports "
                "custom velocity and full speed");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "7: Move to position with orientation towards a target point");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Custom velocity settings:");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "- Use velocity_scaling: -1.0 for FULL SPEED movement without "
                "time parameterization");

    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
