#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

namespace kortex_bt {

class AssertTrue : public BT::ConditionNode {
 public:
  AssertTrue(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace kortex_bt
