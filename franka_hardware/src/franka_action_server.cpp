#include "franka_hardware/franka_action_server.hpp"

namespace franka_hardware {

ActionServer::ActionServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot)
    : rclcpp::Node("action_server", options), robot_(std::move(robot)) {
  error_recovery_action_server_ = rclcpp_action::create_server<franka_msgs::action::ErrorRecovery>(
      this, "~/error_recovery",
      [this](auto /*uuid*/, auto /*goal*/) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto& /*goal_handle*/) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto& goal_handle) { errorRecoveryAction(goal_handle); });

  RCLCPP_INFO(get_logger(), "Action server started");
}

auto ActionServer::errorRecoveryAction(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>>&
        goal_handle) -> void {
  std::thread{[this, goal_handle]() {
    auto result = std::make_shared<franka_msgs::action::ErrorRecovery::Result>();
    try {
      robot_->automaticErrorRecovery();
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Automatic recovery succeeded");
    } catch (const franka::CommandException& command_exception) {
      RCLCPP_ERROR(this->get_logger(),
                   "Command exception thrown during automatic error recovery %s",
                   command_exception.what());
      goal_handle->abort(result);
    } catch (const franka::NetworkException& network_exception) {
      RCLCPP_ERROR(this->get_logger(), "Network exception thrown automatic error recovery %s",
                   network_exception.what());
      goal_handle->abort(result);
    }
  }}.detach();
}

}  // namespace franka_hardware
