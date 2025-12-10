#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <kortex_interfaces/action/detect_object.hpp>

#define IDENTIFY_OBJECT_DEFAULT_DISTANCE 0.05

namespace kortex_bt {

using DetectObject = kortex_interfaces::action::DetectObject;

class IdentifyObject : public BT::RosActionNode<DetectObject> {
 public:
  IdentifyObject(const std::string &name, const BT::NodeConfig &config,
                 const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace kortex_bt
