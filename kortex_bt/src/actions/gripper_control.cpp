#include "kortex_bt/actions/gripper_control.hpp"

namespace kortex_bt {

GripperControl::GripperControl(const std::string &name,
                               const BT::NodeConfig &config,
                               const BT::RosNodeParams &params)
    : BT::RosActionNode<GripperControlAction>(name, config, params) {}

BT::PortsList GripperControl::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("position")});
}

bool GripperControl::setGoal(Goal &goal) {
  double position;
  if (!getInput("position", position)) {
    RCLCPP_ERROR(logger(), "Missing position input");
    return false;
  }
  goal.position = position;
  RCLCPP_INFO(logger(), "Setting gripper position to %.2f", position);
  return true;
}

BT::NodeStatus GripperControl::onResultReceived(const WrappedResult &result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (result.result->success) {
        RCLCPP_INFO(logger(), "GripperControl action succeeded.");
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_ERROR(logger(), "GripperControl action failed.");
        return BT::NodeStatus::FAILURE;
      }
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "GripperControl action was aborted.");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(logger(), "GripperControl action was canceled.");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown result code received.");
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus GripperControl::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "%s", feedback->processing_status.c_str());
  return BT::NodeStatus::RUNNING;
}

}  // namespace kortex_bt
