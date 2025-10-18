#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "kortex_interfaces/srv/switch_control_mode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ControllerSwitcher : public rclcpp::Node {
 public:
  ControllerSwitcher() : Node("controller_switcher") {
    position_controller_ = "joint_trajectory_controller";
    velocity_controller_ = "twist_controller";
    current_active_mode_ = "unknown";  // Initially unknown

    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    switch_controller_client_ =
        this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

    list_controllers_client_ =
        this->create_client<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers");

    auto handle_switch_request =
        [this](const std::shared_ptr<
                   kinova_action_interfaces::srv::SwitchControlMode::Request>
                   request,
               std::shared_ptr<
                   kinova_action_interfaces::srv::SwitchControlMode::Response>
                   response) -> void {
      RCLCPP_INFO(this->get_logger(), "Switch mode request received: '%s'",
                  request->mode.c_str());

      // First check the current state of controllers
      if (!update_current_mode()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to get current controller state");
        response->success = false;
        return;
      }

      // Check if we are already in the requested mode
      if (request->mode == current_active_mode_) {
        RCLCPP_INFO(
            this->get_logger(),
            "Controller for mode '%s' is already active. No switch needed.",
            request->mode.c_str());
        response->success = true;
        return;
      }

      std::vector<std::string> activate_controllers;
      std::vector<std::string> deactivate_controllers;

      if (request->mode == "velocity") {
        activate_controllers.push_back(velocity_controller_);
        deactivate_controllers.push_back(position_controller_);
      } else if (request->mode == "position") {
        activate_controllers.push_back(position_controller_);
        deactivate_controllers.push_back(velocity_controller_);
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid mode: '%s'. Use 'position' or 'velocity'.",
                     request->mode.c_str());
        response->success = false;
        return;
      }

      if (!switch_controller_client_->wait_for_service(2s)) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Service /controller_manager/switch_controller not available.");
        response->success = false;
        return;
      }

      auto switch_request = std::make_shared<
          controller_manager_msgs::srv::SwitchController::Request>();
      switch_request->activate_controllers = activate_controllers;
      switch_request->deactivate_controllers = deactivate_controllers;
      switch_request->strictness =
          controller_manager_msgs::srv::SwitchController::Request::STRICT;
      switch_request->activate_asap = true;

      RCLCPP_INFO(this->get_logger(), "Switching from '%s' to '%s'...",
                  current_active_mode_.c_str(), request->mode.c_str());

      auto future =
          switch_controller_client_->async_send_request(switch_request);
      auto status = future.wait_for(15s);

      if (status == std::future_status::ready) {
        auto result = future.get();
        if (result->ok) {
          RCLCPP_INFO(this->get_logger(),
                      "Controller switch to '%s' mode succeeded.",
                      request->mode.c_str());
          current_active_mode_ = request->mode;
          response->success = true;
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Controller switch failed (response from manager).");
          response->success = false;
        }
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Timeout while calling the switch service.");
        response->success = false;
      }
    };

    switch_mode_service_ =
        this->create_service<kinova_action_interfaces::srv::SwitchControlMode>(
            "/switch_control_mode", handle_switch_request,
            rmw_qos_profile_services_default, service_callback_group_);

    // Determine initial state
    if (!update_current_mode()) {
      RCLCPP_WARN(this->get_logger(),
                  "Could not determine initial controller state");
      current_active_mode_ = "unknown";
    }

    RCLCPP_INFO(this->get_logger(), "Switcher node ready. Current mode: '%s'.",
                current_active_mode_.c_str());
  }

 private:
  std::string position_controller_;
  std::string velocity_controller_;
  std::string current_active_mode_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
      list_controllers_client_;
  rclcpp::Service<kinova_action_interfaces::srv::SwitchControlMode>::SharedPtr
      switch_mode_service_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  bool update_current_mode() {
    if (!list_controllers_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Service /controller_manager/list_controllers not available.");
      return false;
    }

    auto request = std::make_shared<
        controller_manager_msgs::srv::ListControllers::Request>();
    auto future = list_controllers_client_->async_send_request(request);

    auto status = future.wait_for(5s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Timeout while listing controllers.");
      return false;
    }

    auto result = future.get();
    bool position_active = false;
    bool velocity_active = false;

    for (const auto& controller : result->controller) {
      if (controller.name == position_controller_ &&
          controller.state == "active") {
        position_active = true;
      }
      if (controller.name == velocity_controller_ &&
          controller.state == "active") {
        velocity_active = true;
      }
    }

    if (position_active && !velocity_active) {
      current_active_mode_ = "position";
      return true;
    } else if (velocity_active && !position_active) {
      current_active_mode_ = "velocity";
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Ambiguous controller state: pos=%d, vel=%d", position_active,
                  velocity_active);
      current_active_mode_ = "unknown";
      return false;
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto switcher_node = std::make_shared<ControllerSwitcher>();

  executor.add_node(switcher_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
