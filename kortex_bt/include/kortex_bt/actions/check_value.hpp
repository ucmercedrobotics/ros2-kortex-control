#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

namespace kortex_bt {

class CheckValue : public BT::ConditionNode {
 public:
  CheckValue(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace kortex_bt
