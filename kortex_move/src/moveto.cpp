#include "kortex_move/moveto.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <future>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

MoveToNode::MoveToNode()
    : Node("move_to_action_server")  // Initialize the node
{
  // Create the action server
  action_server_ = rclcpp_action::create_server<MoveTo>(
      this,       // Node pointer
      "move_to",  // Action name, relative so it resolves under this node's
                  // namespace instead of colliding across robots
      std::bind(&MoveToNode::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),  // Goal handler
      std::bind(&MoveToNode::handle_cancel, this,
                std::placeholders::_1),  // Cancel handler
      std::bind(&MoveToNode::handle_accepted, this,
                std::placeholders::_1)  // Accepted handler
  );

  std::string move_group_ns = this->get_namespace();
  if (move_group_ns == "/") {
    move_group_ns.clear();
  }
  moveit::planning_interface::MoveGroupInterface::Options options("manipulator");
  options.move_group_namespace_ = move_group_ns;
  move_group_interface_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          std::shared_ptr<MoveToNode>(this), options);
  move_group_interface_->setPlanningPipelineId(
      "pilz_industrial_motion_planner");
  move_group_interface_->setPlannerId("PTP");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create the gripper action server
  gripper_action_server_ = rclcpp_action::create_server<GripperControl>(
      this,
      "gripper_control",  // relative, same reasoning as /move_to above
      std::bind(&MoveToNode::handle_gripper_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&MoveToNode::handle_gripper_cancel, this,
                std::placeholders::_1),
      std::bind(&MoveToNode::handle_gripper_accepted, this,
                std::placeholders::_1));

  gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "robotiq_gripper_controller/gripper_cmd");
}

// Handle incoming goal requests
rclcpp_action::GoalResponse MoveToNode::handle_goal(
    const rclcpp_action::GoalUUID &uuid,       // Goal ID (not used here)
    std::shared_ptr<const MoveTo::Goal> goal)  // Goal message
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received moveto goal request...");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // Accept the goal
}

// Handle cancel requests
rclcpp_action::CancelResponse MoveToNode::handle_cancel(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Goal cancel requested");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;  // Accept the cancel request
}

// Handle accepted goals
void MoveToNode::handle_accepted(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle) {
  // Start a new thread to execute the goal
  std::thread{std::bind(&MoveToNode::execute, this, goal_handle)}.detach();
}

// Execute the goal
void MoveToNode::execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle) {
  auto goal = goal_handle->get_goal();               // Get the goal message
  auto result = std::make_shared<MoveTo::Result>();  // Create a result message

  geometry_msgs::msg::PoseStamped eel_pose =
      move_group_interface_->getCurrentPose();
  geometry_msgs::msg::Pose target_pose;

  RCLCPP_INFO(this->get_logger(), "Current efl pose: %f, %f, %f",
              eel_pose.pose.position.x, eel_pose.pose.position.y,
              eel_pose.pose.position.z);
  RCLCPP_INFO(this->get_logger(), "Current efl orientation: %f, %f, %f, %f",
              eel_pose.pose.orientation.x, eel_pose.pose.orientation.y,
              eel_pose.pose.orientation.z, eel_pose.pose.orientation.w);
  RCLCPP_INFO(this->get_logger(), "Movement type: %s",
              goal->movement_link.c_str());

  if (strncmp(goal->movement_link.c_str(), goal->END_EFFECTOR_LINK.c_str(),
              strlen(goal->END_EFFECTOR_LINK.c_str())) == 0) {
    // TODO: add support for orientation updating as well, right now only
    // (x,y,z)
    target_pose = compute_relative_goal_pose(eel_pose.pose, goal->pose);

  } else if (strncmp(goal->movement_link.c_str(), goal->BASE_LINK.c_str(),
                     strlen(goal->BASE_LINK.c_str())) == 0) {
    // we move with respect to the base_link world
    // no transformation required as it expects input is with respect to base
    // link
    target_pose.position = goal->pose.position;
    target_pose.orientation = goal->pose.orientation;
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Bad link requested for movement: %s. Expected: %s, %s",
                 goal->movement_link.c_str(), goal->END_EFFECTOR_LINK.c_str(),
                 goal->BASE_LINK.c_str());
    send_feedback(goal_handle, "FAILED");
    return;
  }

  // move to pose wrt end effector link
  // NOTE: I tried doing this with other links as reference and it works
  //       HOWEVER, after one movement, the error builds up and fails goal
  //       tolerance the next move
  RCLCPP_INFO(this->get_logger(), "Desired efl pose: %f, %f, %f",
              target_pose.position.x, target_pose.position.y,
              target_pose.position.z);
  RCLCPP_INFO(this->get_logger(), "Desired efl orientation: %f, %f, %f, %f",
              target_pose.orientation.x, target_pose.orientation.y,
              target_pose.orientation.z, target_pose.orientation.w);

  move_group_interface_->setPoseTarget(target_pose);

  // Plan the movement to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    moveit::core::MoveItErrorCode ret =
        move_group_interface_->execute(plan);  // Execute the planned movement
    if (ret == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      result->success = true;
    } else {
      result->success = false;
      send_feedback(goal_handle, "Execution failed");  // Send failure feedback
    }
    send_feedback(goal_handle, "SUCCESS");  // Send success feedback
  } else {
    result->success = false;
    send_feedback(goal_handle, "Planning failed");  // Send failure feedback
  }

  goal_handle->succeed(result);  // Mark the goal as succeeded
}

// Send feedback to the client
void MoveToNode::send_feedback(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle,
    const std::string &feedback_msg) {
  (void)goal_handle;
  auto feedback =
      std::make_shared<MoveTo::Feedback>();  // Create a feedback message
  feedback->processing_status = feedback_msg;
  goal_handle->publish_feedback(feedback);  // Publish the feedback
  RCLCPP_INFO(this->get_logger(), "Feedback: %s",
              feedback_msg.c_str());  // Log the feedback
}

geometry_msgs::msg::Pose MoveToNode::compute_relative_goal_pose(
    const geometry_msgs::msg::Pose &current_pose,
    const geometry_msgs::msg::Pose &goal_pose) {
  geometry_msgs::msg::Pose target_pose;
  // Current orientation as a tf2::Quaternion
  tf2::Quaternion current_orientation;
  tf2::fromMsg(current_pose.orientation, current_orientation);

  // convert goals into tf objects
  tf2::Vector3 relative_translation(goal_pose.position.x, goal_pose.position.y,
                                    goal_pose.position.z);
  tf2::Quaternion relative_orientation(
      goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z,
      goal_pose.orientation.w);

  // Combine the current orientation with the relative orientation
  tf2::Quaternion target_orientation =
      current_orientation * relative_orientation;
  target_orientation.normalize();

  // Transform the relative translation into the world frame
  tf2::Matrix3x3 rotation_matrix(current_orientation);
  tf2::Vector3 transformed_translation = rotation_matrix * relative_translation;

  // Compute the target pose
  target_pose.position.x =
      current_pose.position.x + transformed_translation.x();
  target_pose.position.y =
      current_pose.position.y + transformed_translation.y();
  target_pose.position.z =
      current_pose.position.z + transformed_translation.z();
  target_pose.orientation = tf2::toMsg(target_orientation);

  return target_pose;
}

rclcpp_action::GoalResponse MoveToNode::handle_gripper_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const GripperControl::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(this->get_logger(),
              "Received gripper goal request (position: %.3f)", goal->position);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveToNode::handle_gripper_cancel(
    const std::shared_ptr<GoalHandleGripperControl> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Gripper goal cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveToNode::handle_gripper_accepted(
    const std::shared_ptr<GoalHandleGripperControl> goal_handle) {
  std::thread{std::bind(&MoveToNode::execute_gripper, this, goal_handle)}
      .detach();
}

void MoveToNode::execute_gripper(
    const std::shared_ptr<GoalHandleGripperControl> goal_handle) {
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<GripperControl::Result>();

  if (!gripper_action_client_->wait_for_action_server(
          std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Gripper controller not available after 5s");
    result->success = false;
    goal_handle->succeed(result);
    return;
  }

  auto gripper_goal = GripperCommand::Goal();
  gripper_goal.command.position = goal->position;
  gripper_goal.command.max_effort = 0.0;

  send_gripper_feedback(goal_handle, "Sending gripper command");

  // Use a promise so this thread can block until the downstream action
  // completes while the main rclcpp::spin processes the callbacks.
  std::promise<bool> done_promise;
  auto done_future = done_promise.get_future();

  auto send_goal_options =
      rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  send_goal_options.result_callback =
      [&done_promise](
          const rclcpp_action::ClientGoalHandle<GripperCommand>::WrappedResult
              &wrapped_result) {
        done_promise.set_value(wrapped_result.code ==
                               rclcpp_action::ResultCode::SUCCEEDED);
      };
  send_goal_options.goal_response_callback =
      [this](
          const rclcpp_action::ClientGoalHandle<GripperCommand>::SharedPtr
              &goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(),
                       "Gripper goal was rejected by controller");
        }
      };

  gripper_action_client_->async_send_goal(gripper_goal, send_goal_options);

  bool success = done_future.get();
  result->success = success;
  send_gripper_feedback(goal_handle, success ? "SUCCESS" : "FAILED");
  goal_handle->succeed(result);
}

void MoveToNode::send_gripper_feedback(
    const std::shared_ptr<GoalHandleGripperControl> goal_handle,
    const std::string &feedback_msg) {
  auto feedback = std::make_shared<GripperControl::Feedback>();
  feedback->processing_status = feedback_msg;
  goal_handle->publish_feedback(feedback);
  RCLCPP_INFO(this->get_logger(), "Gripper feedback: %s",
              feedback_msg.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<MoveToNode>());  // Create and spin the action server
  rclcpp::shutdown();                   // Shutdown ROS 2
  return 0;
}
