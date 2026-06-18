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
#include <mutex>
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
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/segment.hpp>

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

  // --- Gripper integration (Task 2.5) ---
  // Gripper sits beyond the KDL tip link — no dynamics, just Kp/Kd passthrough.
  std::string gripper_joint_name_;
  bool has_gripper_ = false;
  double gripper_kp_current_ = 2.0;
  double gripper_kd_current_ = 0.5;
  double gripper_kp_desired_ = 2.0;
  double gripper_kd_desired_ = 0.5;
  double gripper_kp_default_ = 2.0;
  double gripper_kd_default_ = 0.5;
  double gripper_kp_max_ = 10.0;
  double gripper_kd_max_ = 2.0;

  // --- KDL dynamics ---
  KDL::Chain chain_;                             // original URDF chain (immutable after configure)
  KDL::Chain active_chain_;                      // chain used by solver (may include payload)
  std::unique_ptr<KDL::ChainDynParam> dyn_solver_; // references active_chain_ (NOT a copy!)
  bool kdl_initialized_ = false;

  // --- Payload compensation ---
  // Subscriber receives [mass_kg, cog_x, cog_y, cog_z] via ~/set_payload topic.
  // Solver rebuild happens in the subscriber callback (non-RT thread),
  // NOT in the 100 Hz update() loop, to avoid memory allocation in the RT path.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr payload_sub_;
  double active_payload_mass_ = 0.0;             // last-applied payload mass
  std::mutex chain_mutex_;                       // protects solver rebuild vs RT reads

  /// Rebuild the dynamics solver with payload added to the last segment.
  /// Called from subscriber callback (non-RT thread).
  void rebuild_dynamics_with_payload(double mass, const KDL::Vector& cog);

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
  // With gripper: [kp_0..kp_6, kd_0..kd_6, grip_kp, grip_kd] = 16 doubles
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

  // --- External force estimation ---
  // tau_ext = tau_motor (from HW effort state) - tau_ff (from model)
  // Low-pass filtered to remove high-frequency noise.
  std::shared_ptr<
      realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
      ext_force_pub_;  // publishes ~/external_force
  std::vector<double> tau_ext_filtered_;  // low-pass filtered external torque
  double ext_force_alpha_ = 0.05;         // filter coefficient (from YAML)

  // --- Helpers ---
  void compute_gravity(const KDL::JntArray& q, KDL::JntArray& tau_g);
  void compute_coriolis(const KDL::JntArray& q, const KDL::JntArray& qdot,
                        KDL::JntArray& tau_c);
  void compute_friction(const KDL::JntArray& qdot,
                        std::vector<double>& tau_f);
};

}  // namespace openarm_compliance_controller
