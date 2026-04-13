// Copyright 2026 OpenArm Contributors
// SPDX-License-Identifier: Apache-2.0
//
// Variable impedance controller with KDL-based gravity/friction/Coriolis
// compensation for OpenArm V10 bimanual robot.
//
// Claims: effort, stiffness, damping command interfaces.
// Leaves:  position, velocity for JointTrajectoryController.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>

namespace openarm_compliance_controller {

class ComplianceController : public controller_interface::ControllerInterface {
 public:
  ComplianceController() = default;

  // --- Lifecycle ---
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // --- Configuration ---
  std::vector<std::string> joint_names_;
  size_t num_joints_ = 7;
  std::string root_link_;
  std::string tip_link_;

  // --- KDL dynamics ---
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
  bool kdl_initialized_ = false;

  // --- Friction model: tau_f = Fc*tanh(0.1*k*dq) + Fv*dq + Fo ---
  std::vector<double> Fc_, k_coeff_, Fv_, Fo_;

  // --- Calibration scale factors for tau_ff ---
  std::vector<double> tau_ff_scale_;

  // --- Impedance parameters ---
  std::vector<double> kp_current_, kd_current_;   // current (rate-limited)
  std::vector<double> kp_desired_, kd_desired_;    // target from topic
  std::vector<double> kp_default_, kd_default_;    // startup defaults
  std::vector<double> kp_min_, kp_max_;
  std::vector<double> kd_min_, kd_max_;
  double delta_kp_max_ = 2.0;
  double delta_kd_max_ = 0.1;

  // --- RT-safe communication ---
  // Buffer stores [kp_0..kp_6, kd_0..kd_6] = 14 doubles
  realtime_tools::RealtimeBuffer<std::vector<double>> rt_impedance_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      impedance_sub_;

  // --- Diagnostic publishers ---
  std::shared_ptr<
      realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
      tau_ff_pub_;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
      gains_pub_;  // publishes [kp_current..., kd_current...]

  // --- Helpers ---
  void compute_gravity(const KDL::JntArray& q, KDL::JntArray& tau_g);
  void compute_coriolis(const KDL::JntArray& q, const KDL::JntArray& qdot,
                        KDL::JntArray& tau_c);
  void compute_friction(const KDL::JntArray& qdot,
                        std::vector<double>& tau_f);
};

}  // namespace openarm_compliance_controller
