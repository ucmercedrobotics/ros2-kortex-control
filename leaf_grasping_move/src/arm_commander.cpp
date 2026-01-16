#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>

#include "control_msgs/action/gripper_command.hpp"
#include "custom_interfaces/srv/get_spectrum.hpp"
#include "kortex_interfaces/action/process_target.hpp"  // The new action file
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

class ArmCommander : public rclcpp::Node {
 public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GripperCommandGoalHandle =
      rclcpp_action::ClientGoalHandle<GripperCommand>;
  using ProcessTarget = kortex_interfaces::action::ProcessTarget;
  using GoalHandleProcessTarget =
      rclcpp_action::ServerGoalHandle<ProcessTarget>;
  using GetSpectrum = custom_interfaces::srv::GetSpectrum;

  explicit ArmCommander()
      : Node("arm_commander",
             rclcpp::NodeOptions()
                 .automatically_declare_parameters_from_overrides(true)) {
    RCLCPP_INFO(this->get_logger(), "Starting initialization...");
  }

  void initialize() {
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    using moveit::planning_interface::MoveGroupInterface;
    move_group_ = std::make_shared<MoveGroupInterface>(
        std::static_pointer_cast<rclcpp::Node>(shared_from_this()),
        "manipulator");

    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    this->gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
        this, "/robotiq_gripper_controller/gripper_cmd", callback_group_);

    this->spectrum_client_ =
        this->create_client<GetSpectrum>("/get_spectrum", rmw_qos_profile_services_default, callback_group_);

    // Create the Action Server
    this->action_server_ = rclcpp_action::create_server<ProcessTarget>(
        this, "process_target",
        std::bind(&ArmCommander::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&ArmCommander::handle_cancel, this, std::placeholders::_1),
        std::bind(&ArmCommander::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(), callback_group_);

    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);

    rclcpp::sleep_for(std::chrono::seconds(2));
    addCollisionObjects();

    RCLCPP_INFO(this->get_logger(),
                "Arm Commander Action Server has been initialized and is ready "
                "for goals.");
  }

  void goToHomePosition() {
    std::vector<double> joint_goal = move_group_->getCurrentJointValues();

    joint_goal[0] = 0.0;
    joint_goal[1] = -0.785398;
    joint_goal[2] = -2.0;
    joint_goal[3] = 0.0;
    joint_goal[4] = -0.436332;
    joint_goal[5] = 1.5708;

    RCLCPP_INFO(this->get_logger(), "Planning motion to home position...");
    move_group_->setJointValueTarget(joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (static_cast<bool>(move_group_->plan(plan))) {
      RCLCPP_INFO(this->get_logger(), "Executing motion to home position...");
      move_group_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Reached home position");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
  }

  bool goToPose(const geometry_msgs::msg::Pose& pose_goal) {
    RCLCPP_INFO(this->get_logger(), "Planning motion to target pose...");
    RCLCPP_INFO(this->get_logger(), "Target pose: x=%.3f, y=%.3f, z=%.3f",
                pose_goal.position.x, pose_goal.position.y,
                pose_goal.position.z);

    if (!move_group_) {
      RCLCPP_ERROR(this->get_logger(), "Move group not initialized!");
      return false;
    }

    move_group_->setPoseTarget(pose_goal);
    move_group_->setPlanningTime(1.0);
    move_group_->setNumPlanningAttempts(10);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing planned motion...");
      if (move_group_->execute(plan)) {
        RCLCPP_INFO(this->get_logger(), "Reached target pose");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute motion");
        success = false;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Planning failed! Target pose might be unreachable");
    }
    move_group_->clearPoseTargets();
    return success;
  }

 private:
  // Action Server Callbacks
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ProcessTarget::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleProcessTarget> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<GoalHandleProcessTarget> goal_handle) {
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    std::thread{
        std::bind(&ArmCommander::execute_goal, this, std::placeholders::_1),
        goal_handle}
        .detach();
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  }

  // Goal handler that tries to reach each pose in sequence
  // reaching one pose is enough to consider the goal successful -> we break the
  // loop once a pose is reached
  void execute_goal(
      const std::shared_ptr<GoalHandleProcessTarget> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ProcessTarget::Feedback>();
    auto result = std::make_shared<ProcessTarget::Result>();

    int successful_pose_count = 0;

    for (int i = 0; i < 5; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->status = "Moving to pose " + std::to_string(i + 1);
      goal_handle->publish_feedback(feedback);

      // If a pose is reachable, perform the sequence and increment the counter
      if (goToPose(goal->target_poses[i])) {
        successful_pose_count++;

        feedback->status = "Operating gripper at pose " + std::to_string(i + 1);
        goal_handle->publish_feedback(feedback);
        operateGripper(0.8);

        // Call the spectrum service while the gripper is closed
        feedback->status = "Acquiring spectrum data...";
        goal_handle->publish_feedback(feedback);
        callGetSpectrumService();

        // close the gripper for 5 seconds
        this->get_clock()->sleep_for(std::chrono::seconds(5));
        operateGripper(0.0);
        RCLCPP_INFO(this->get_logger(),
                    "A valid pose was reached. Completing the goal.");
        break;
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to reach pose %d. Skipping.",
                    i + 1);
        continue;
      }
    }

    if (successful_pose_count > 0) {
      feedback->status = "Sequence complete. Returning to home position.";
      goal_handle->publish_feedback(feedback);
      // after finishing one target, go back to home position
      goToHomePosition();
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "No poses in this target were reachable.");
    }

    if (rclcpp::ok()) {
      //  The success flag is based on whether any pose was reached.
      result->success = (successful_pose_count > 0);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Finished processing target.");
    }
  }

  // Helper function to create a box-shaped collision object
  moveit_msgs::msg::CollisionObject createCollisionBox(
      const std::string& id, const std::string& frame_id, double dim_x,
      double dim_y, double dim_z, double pos_x, double pos_y, double pos_z) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = dim_x;
    primitive.dimensions[primitive.BOX_Y] = dim_y;
    primitive.dimensions[primitive.BOX_Z] = dim_z;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = pos_x;
    box_pose.position.y = pos_y;
    box_pose.position.z = pos_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }

  void createBoundingBoxRestrictions(
      std::vector<moveit_msgs::msg::CollisionObject>& collision_objects,
      const std::string& frame_id, const geometry_msgs::msg::Point& center,
      const geometry_msgs::msg::Vector3& dimensions, double thickness) {
    // Top wall
    collision_objects.push_back(createCollisionBox(
        "restriction_top", frame_id, dimensions.x, dimensions.y,
        thickness,  // Dimensions
        center.x, center.y,
        center.z + thickness / 2.0 + dimensions.z / 2.0));  // Position

    // Bottom wall
    collision_objects.push_back(createCollisionBox(
        "restriction_bottom", frame_id, dimensions.x, dimensions.y,
        thickness,  // Dimensions
        center.x, center.y,
        center.z - thickness / 2.0 - dimensions.z / 2.0));  // Position

    // Left wall (positive Y side)
    collision_objects.push_back(createCollisionBox(
        "restriction_left", frame_id, dimensions.x, thickness,
        dimensions.z + 2.0 * thickness,  // Dimensions
        center.x, center.y + thickness / 2.0 + dimensions.y / 2.0,
        center.z));  // Position

    // Right wall (negative Y side)
    collision_objects.push_back(createCollisionBox(
        "restriction_right", frame_id, dimensions.x, thickness,
        dimensions.z + 2.0 * thickness,  // Dimensions
        center.x, center.y - thickness / 2.0 - dimensions.y / 2.0,
        center.z));  // Position
  }

  void addCollisionObjects() {
    // ground table
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    const std::string planning_frame = move_group_->getPlanningFrame();
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group_->getPlanningFrame();
    table.id = "table";
    shape_msgs::msg::SolidPrimitive primitive_table;
    primitive_table.type = primitive_table.BOX;
    primitive_table.dimensions.resize(3);
    primitive_table.dimensions[primitive_table.BOX_X] = 1.5;
    primitive_table.dimensions[primitive_table.BOX_Y] = 1.5;
    primitive_table.dimensions[primitive_table.BOX_Z] = 0.05;
    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.z = -0.025;
    table.primitives.push_back(primitive_table);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    collision_objects.push_back(table);

    // wall on the right side of the robot
    moveit_msgs::msg::CollisionObject wall;
    wall.header.frame_id = move_group_->getPlanningFrame();
    wall.id = "wall";
    shape_msgs::msg::SolidPrimitive primitive_wall;
    primitive_wall.type = primitive_wall.BOX;
    primitive_wall.dimensions.resize(3);
    primitive_wall.dimensions[primitive_wall.BOX_X] = 1.5;
    primitive_wall.dimensions[primitive_wall.BOX_Y] = 0.05;
    primitive_wall.dimensions[primitive_wall.BOX_Z] = 1.5;
    geometry_msgs::msg::Pose wall_pose;
    wall_pose.orientation.w = 1.0;
    wall_pose.position.x = 0.0;
    wall_pose.position.y = -0.345;
    wall_pose.position.z = 0.75;
    wall.primitives.push_back(primitive_wall);
    wall.primitive_poses.push_back(wall_pose);
    wall.operation = wall.ADD;
    // collision_objects.push_back(wall);

    // Define the properties of the virtual bounding box ***to restrict the
    // workspace***
    geometry_msgs::msg::Point box_center;
    box_center.x = 0.7;
    box_center.y = 0.0;
    box_center.z = 0.65;

    geometry_msgs::msg::Vector3 box_dimensions;
    box_dimensions.x = 1.0;   // The length of the workspace
    box_dimensions.y = 0.75;  // The width of the workspace
    box_dimensions.z = 0.5;   // The height of the workspace

    const double wall_thickness = 0.1;  // virtual walls

    // Create the bounding box with a single function call
    // createBoundingBoxRestrictions(collision_objects, planning_frame, box_center,
    //                               box_dimensions, wall_thickness);

    RCLCPP_INFO(this->get_logger(), "Adding collision objects to the world");
    planning_scene_interface_->addCollisionObjects(collision_objects);
  }

  // Function to operate the gripper -> 0.8 to close, 0.0 to open
  void operateGripper(float position) {
    if (!this->gripper_action_client_->wait_for_action_server(
            std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Gripper action server not available after waiting");
      return;
    }
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 100.0;
    RCLCPP_INFO(this->get_logger(), "Sending gripper goal (Position: %.1f)",
                position);
    auto send_goal_options =
        rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](const GripperCommandGoalHandle::SharedPtr& goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(),
                         "Gripper goal was rejected by server");
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "Gripper goal accepted by server, waiting for result");
          }
        };
    auto future_goal_handle =
        gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(),
                   "Gripper goal was rejected by server or handle is null");
      return;
    }
    auto future_result = gripper_action_client_->async_get_result(goal_handle);
    auto result_wrapper = future_result.get();
    if (result_wrapper.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(),
                  "Gripper operation finished successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Gripper operation failed with status: %d",
                   static_cast<int>(result_wrapper.code));
    }
  }

  // Function to call the spectrum service
  void callGetSpectrumService() {
    if (!spectrum_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(this->get_logger(),
                  "GetSpectrum service not available after waiting");
      return;
    }

    auto request = std::make_shared<GetSpectrum::Request>();
    RCLCPP_INFO(this->get_logger(), "Calling GetSpectrum service...");

    auto future = spectrum_client_->async_send_request(request);
    auto result = future.get();

    RCLCPP_INFO(this->get_logger(),
                "GetSpectrum service returned %zu wavelengths and %zu spectrum values",
                result->wavelengths.size(), result->spectrum.size());

    // Save spectrum data to the latest results directory
    if (!result->wavelengths.empty() && !result->spectrum.empty()) {
      saveSpectrumData(result->wavelengths, result->spectrum);
    }
  }

  // Find the latest results directory matching the Python node's structure
  std::string findLatestResultsDir() {
    namespace fs = std::filesystem;

    // Get today's date in MM-DD-YYYY format (matching Python's strftime("%m-%d-%Y"))
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream date_ss;
    date_ss << std::put_time(std::localtime(&time_t_now), "%m-%d-%Y");
    std::string date_str = date_ss.str();

    fs::path base_dir = "runs/results";
    fs::path date_dir = base_dir / date_str;

    if (!fs::exists(date_dir)) {
      RCLCPP_WARN(this->get_logger(), "Date directory does not exist: %s",
                  date_dir.string().c_str());
      return "";
    }

    // Find the latest resultsN directory
    int latest_run = 0;
    for (const auto& entry : fs::directory_iterator(date_dir)) {
      if (entry.is_directory()) {
        std::string dir_name = entry.path().filename().string();
        if (dir_name.rfind("results", 0) == 0) {  // starts with "results"
          try {
            int run_num = std::stoi(dir_name.substr(7));  // extract number after "results"
            if (run_num > latest_run) {
              latest_run = run_num;
            }
          } catch (...) {
            continue;
          }
        }
      }
    }

    if (latest_run == 0) {
      RCLCPP_WARN(this->get_logger(), "No results directories found in: %s",
                  date_dir.string().c_str());
      return "";
    }

    fs::path latest_dir = date_dir / ("results" + std::to_string(latest_run));
    return latest_dir.string();
  }

  // Function to save spectrum data to a CSV file
  void saveSpectrumData(const std::vector<uint16_t>& wavelengths,
                        const std::vector<double>& spectrum) {
    std::string save_dir = findLatestResultsDir();

    if (save_dir.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not find results directory. Spectrum data not saved.");
      return;
    }

    // Generate timestamp for unique filename
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream ts_ss;
    ts_ss << std::put_time(std::localtime(&time_t_now), "%H%M%S");

    std::string filename = save_dir + "/spectrum_" + ts_ss.str() + ".csv";

    std::ofstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s",
                   filename.c_str());
      return;
    }

    // Write header
    file << "wavelength,spectrum\n";

    // Write data
    size_t num_values = std::min(wavelengths.size(), spectrum.size());
    for (size_t i = 0; i < num_values; ++i) {
      file << wavelengths[i] << "," << std::fixed << std::setprecision(6)
           << spectrum[i] << "\n";
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Spectrum data saved to: %s", filename.c_str());
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
      planning_scene_interface_;
  rclcpp_action::Server<ProcessTarget>::SharedPtr action_server_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  rclcpp::Client<GetSpectrum>::SharedPtr spectrum_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto arm_commander_node = std::make_shared<ArmCommander>();
  arm_commander_node->initialize();

  // no multi-threaded executor for now
  rclcpp::executors::MultiThreadedExecutor executor;
  // rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(arm_commander_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
