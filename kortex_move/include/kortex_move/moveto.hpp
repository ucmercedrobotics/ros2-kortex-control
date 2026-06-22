#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "kortex_interfaces/action/gripper_control.hpp"
#include "kortex_interfaces/action/move_to.hpp"

using MoveTo = kortex_interfaces::action::MoveTo;
using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;

using GripperControl = kortex_interfaces::action::GripperControl;
using GoalHandleGripperControl = rclcpp_action::ServerGoalHandle<GripperControl>;
using GripperCommand = control_msgs::action::GripperCommand;

// Define a class for the Action Server
class MoveToNode : public rclcpp::Node {
 public:
  MoveToNode();

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_interface_;

 private:
  rclcpp_action::Server<MoveTo>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // START MoveTo action server functionality
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveTo::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMoveTo> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle);
  void execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle);
  void send_feedback(const std::shared_ptr<GoalHandleMoveTo> goal_handle,
                     const std::string& feedback_msg);
  // END MoveTo action server functionality

  // START GripperControl action server functionality
  rclcpp_action::Server<GripperControl>::SharedPtr gripper_action_server_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;

  rclcpp_action::GoalResponse handle_gripper_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const GripperControl::Goal> goal);
  rclcpp_action::CancelResponse handle_gripper_cancel(
      const std::shared_ptr<GoalHandleGripperControl> goal_handle);
  void handle_gripper_accepted(
      const std::shared_ptr<GoalHandleGripperControl> goal_handle);
  void execute_gripper(
      const std::shared_ptr<GoalHandleGripperControl> goal_handle);
  void send_gripper_feedback(
      const std::shared_ptr<GoalHandleGripperControl> goal_handle,
      const std::string& feedback_msg);
  // END GripperControl action server functionality

  geometry_msgs::msg::Pose compute_relative_goal_pose(
      const geometry_msgs::msg::Pose& current_pose,
      const geometry_msgs::msg::Pose& goal_pose);
};
