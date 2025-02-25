#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/time.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <franka/robot_state.h>
#include <controller_interface/controller_interface.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

#include "franka_example_controllers/visibility_control.h"

namespace franka_example_controllers {
class CartesianImpedanceExampleController : public controller_interface::ControllerInterface {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Config methods

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  // ROS2 lifecycle related methods
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Main real-time method
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  bool k_elbow_activated{true};

  franka_msgs::msg::FrankaRobotState robot_state_;
  franka_msgs::msg::FrankaRobotState init_robot_state_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  std::string arm_id_;
  int num_joints{7};

  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_j_d);

  // Classic cartesian controller
  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_equilibrium_pose_;

  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace franka_example_controllers
