#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "kortex_interfaces/action/process_target.hpp"
#include "kortex_interfaces/msg/leaf_pose_arrays.hpp"
#include "std_msgs/msg/string.hpp"  // Required for publishing the status

class TargetManager : public rclcpp::Node {
 public:
  using ProcessTarget = kortex_interfaces::action::ProcessTarget;
  using GoalHandleProcessTarget =
      rclcpp_action::ClientGoalHandle<ProcessTarget>;

  explicit TargetManager()
      : Node("target_manager",
             rclcpp::NodeOptions()
                 .automatically_declare_parameters_from_overrides(true)) {
    // Action client to send goals to the ArmCommander
    this->action_client_ =
        rclcpp_action::create_client<ProcessTarget>(this, "process_target");

    // Subscription to receive the list of all targets
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.transient_local();
    subscription_ =
        this->create_subscription<kortex_interfaces::msg::LeafPoseArrays>(
            "/multi_target_poses", qos,
            std::bind(&TargetManager::topic_callback, this,
                      std::placeholders::_1));

    // Publisher to announce when the entire task sequence is complete
    status_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/arm_task_status", 10);

    RCLCPP_INFO(this->get_logger(),
                "Target Manager node has been initialized.");
  }

 private:
  rclcpp_action::Client<ProcessTarget>::SharedPtr action_client_;
  rclcpp::Subscription<kortex_interfaces::msg::LeafPoseArrays>::SharedPtr
      subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  // Member variables to store the list of targets and track progress
  kortex_interfaces::msg::LeafPoseArrays::SharedPtr current_target_list_;
  size_t current_target_index_ = 0;
  size_t successful_target_count_ = 0;

  // This callback saves the targets and starts the processing sequence.
  void topic_callback(
      const kortex_interfaces::msg::LeafPoseArrays::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received new multi-target poses message.");

    if (msg->poses1.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Received pose message with 0 targets. Doing nothing.");
      // Announce completion for the bridge node to hear
      auto status_msg = std_msgs::msg::String();
      status_msg.data = "COMPLETE";
      status_publisher_->publish(status_msg);

      RCLCPP_INFO(this->get_logger(),
                  "-----------------------------------------");
      current_target_list_.reset();  // Clear the stored list
      return;
    }

    this->current_target_list_ = msg;
    this->current_target_index_ = 0;
    // I didn't restart the count here to preserve history across multiple
    // messages. this->successful_target_count_ = 0;

    // Kick off the processing by sending the first goal.
    send_next_goal();
  }

  // This function sends a single goal to the arm commander based on the current
  // index.
  void send_next_goal() {
    // If we have processed all targets in the list, publish completion and
    // stop.
    if (!current_target_list_ ||
        current_target_index_ >= current_target_list_->poses1.size()) {
      RCLCPP_INFO(this->get_logger(),
                  "-----------------------------------------");
      RCLCPP_INFO(this->get_logger(), "All targets have been processed.");
      RCLCPP_INFO(this->get_logger(),
                  "Final Result: %zu out of %zu targets were successful.",
                  successful_target_count_,
                  current_target_list_->poses1.size());
      RCLCPP_INFO(this->get_logger(),
                  "-----------------------------------------");

      auto status_msg = std_msgs::msg::String();
      status_msg.data = "COMPLETE";
      status_publisher_->publish(status_msg);

      current_target_list_.reset();
      return;
    }

    size_t num_targets = current_target_list_->poses1.size();
    RCLCPP_INFO(this->get_logger(),
                "-----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Processing Target %zu of %zu",
                current_target_index_ + 1, num_targets);

    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Action server not available after waiting. Aborting all targets.");
      return;
    }

    // Prepare the goal message for the current target
    auto goal_msg = ProcessTarget::Goal();
    goal_msg.target_poses[0] =
        current_target_list_->poses1[current_target_index_];
    goal_msg.target_poses[1] =
        current_target_list_->poses2[current_target_index_];
    goal_msg.target_poses[2] =
        current_target_list_->poses3[current_target_index_];
    goal_msg.target_poses[3] =
        current_target_list_->poses4[current_target_index_];
    goal_msg.target_poses[4] =
        current_target_list_->poses5[current_target_index_];

    auto send_goal_options =
        rclcpp_action::Client<ProcessTarget>::SendGoalOptions();

    send_goal_options.feedback_callback =
        [](GoalHandleProcessTarget::SharedPtr,
           const std::shared_ptr<const ProcessTarget::Feedback> feedback) {
          RCLCPP_INFO(rclcpp::get_logger("target_manager"), "Feedback: %s",
                      feedback->status.c_str());
        };

    // The result_callback will be triggered automatically when the action
    // completes.
    send_goal_options.result_callback =
        std::bind(&TargetManager::result_callback, this, std::placeholders::_1);

    // Send the goal asynchronously. The node is now free until a result comes
    // back.
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // This callback processes the result of an action and triggers the next goal.
  void result_callback(const GoalHandleProcessTarget::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      if (result.result->success) {
        RCLCPP_INFO(this->get_logger(), "Target %zu processed successfully!",
                    current_target_index_ + 1);
        successful_target_count_++;
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Arm Commander reported failure for target %zu.",
                    current_target_index_ + 1);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Target %zu failed with status code: %d",
                   current_target_index_ + 1, static_cast<int>(result.code));
    }

    // Increment the index and call send_next_goal() to continue the sequence.
    current_target_index_++;
    send_next_goal();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetManager>();
  // Using a MultiThreadedExecutor is good practice for nodes with callbacks
  // that might take time. do not use multi-threading for now
  // rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
