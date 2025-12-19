#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <kortex_interfaces/action/move_to.hpp>

namespace kortex_bt {

using MoveToAction = kortex_interfaces::action::MoveTo;

class MoveTo : public BT::RosActionNode<MoveToAction> {
 public:
  MoveTo(const std::string &name, const BT::NodeConfig &config,
         const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace kortex_bt
