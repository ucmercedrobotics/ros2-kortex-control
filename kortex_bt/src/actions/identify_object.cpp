#include "kortex_bt/actions/identify_object.hpp"

namespace kortex_bt {

IdentifyObject::IdentifyObject(const std::string &name,
                               const BT::NodeConfig &config,
                               const BT::RosNodeParams &params)
    : BT::RosActionNode<DetectObject>(name, config, params) {}

BT::PortsList IdentifyObject::providedPorts() {
  return providedBasicPorts({BT::InputPort<std::string>("objectName"),
                             BT::OutputPort<std::string>("objectColor"),
                             BT::OutputPort<double>("objectDistance")});
}

bool IdentifyObject::setGoal(Goal &goal) {
  std::string object_name;
  std::string object_color;
  double object_distance;

  if (!getInput("objectName", object_name)) {
    RCLCPP_ERROR(logger(), "Missing object name input");
    return false;
  }
  goal.target_class = object_name;

  if (getInput("objectColor", object_color)) {
    goal.colors = {object_color};
  } else {
    RCLCPP_WARN(logger(), "No object color specified, proceeding without it");
  }

  if (getInput("objectDistance", object_distance)) {
    goal.target_view_point_distance = object_distance;
  } else {
    RCLCPP_WARN(logger(), "No object distance specified, defaulting to %.2f",
                IDENTIFY_OBJECT_DEFAULT_DISTANCE);
    goal.target_view_point_distance = IDENTIFY_OBJECT_DEFAULT_DISTANCE;
  }

  RCLCPP_INFO(logger(), "Looking for object: %s", goal.target_class.c_str());
  return true;
}

BT::NodeStatus IdentifyObject::onResultReceived(const WrappedResult &result) {
  // TODO: handle object location details from result

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (result.result->success) {
        RCLCPP_INFO(logger(), "Object identified successfully.");
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_ERROR(logger(), "Object identification failed.");
        return BT::NodeStatus::FAILURE;
      }
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "IdentifyObject action was aborted.");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(logger(), "IdentifyObject action was canceled.");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown result code received.");
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus IdentifyObject::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "%s", feedback->processing_status.c_str());
  return BT::NodeStatus::RUNNING;
}

}  // namespace kortex_bt
