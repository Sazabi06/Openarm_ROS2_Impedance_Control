// Copyright 2026 OpenArm Contributors
// SPDX-License-Identifier: Apache-2.0

#include "openarm_compliance_controller/compliance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#include "pluginlib/class_list_macros.hpp"

namespace openarm_compliance_controller {

// ============================================================
// on_init — declare all parameters
// ============================================================
controller_interface::CallbackReturn ComplianceController::on_init() {
  try {
    // Joint names
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});

    // KDL chain endpoints
    auto_declare<std::string>("root_link", "openarm_body_link0");
    auto_declare<std::string>("tip_link", "openarm_right_hand");

    // Calibration scale factors
    auto_declare<std::vector<double>>(
        "tau_ff_scale", {1.0, 0.96, 1.0, 0.67, 1.0, 1.0, 1.0});

    // Friction model
    auto_declare<std::vector<double>>(
        "friction.Fc", {0.306, 0.306, 0.40, 0.166, 0.050, 0.093, 0.172});
    auto_declare<std::vector<double>>(
        "friction.k", {28.417, 28.417, 29.065, 130.038, 151.771, 242.287, 7.888});
    auto_declare<std::vector<double>>(
        "friction.Fv", {0.063, 0.063, 0.604, 0.813, 0.029, 0.072, 0.084});
    auto_declare<std::vector<double>>(
        "friction.Fo", {0.088, 0.088, 0.008, -0.058, 0.005, 0.009, -0.059});

    // Impedance bounds
    auto_declare<std::vector<double>>(
        "kp_default", {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0});
    auto_declare<std::vector<double>>(
        "kd_default", {2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5});
    auto_declare<std::vector<double>>(
        "kp_max", {150.0, 150.0, 150.0, 120.0, 30.0, 30.0, 30.0});
    auto_declare<std::vector<double>>(
        "kd_max", {5.0, 5.0, 5.0, 5.0, 2.0, 2.0, 2.0});
    auto_declare<std::vector<double>>(
        "kp_min", {15.0, 15.0, 15.0, 12.0, 3.0, 3.0, 3.0});
    auto_declare<std::vector<double>>(
        "kd_min", {0.5, 0.5, 0.4, 0.4, 0.15, 0.12, 0.1});

    // Rate limiting
    auto_declare<double>("delta_kp_max", 2.0);
    auto_declare<double>("delta_kd_max", 0.1);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// command_interface_configuration — claim effort + stiffness + damping
// ============================================================
controller_interface::InterfaceConfiguration
ComplianceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    conf.names.push_back(joint + "/stiffness");
    conf.names.push_back(joint + "/damping");
  }
  return conf;
}

// ============================================================
// state_interface_configuration — read position + velocity
// ============================================================
controller_interface::InterfaceConfiguration
ComplianceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

// ============================================================
// on_configure — load parameters + build KDL chain + create pub/sub
// ============================================================
controller_interface::CallbackReturn ComplianceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Load joint names
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified!");
    return controller_interface::CallbackReturn::ERROR;
  }
  num_joints_ = joint_names_.size();

  // Load chain endpoints
  root_link_ = get_node()->get_parameter("root_link").as_string();
  tip_link_ = get_node()->get_parameter("tip_link").as_string();

  // Load calibration scale factors
  tau_ff_scale_ = get_node()->get_parameter("tau_ff_scale").as_double_array();

  // Load friction parameters
  Fc_ = get_node()->get_parameter("friction.Fc").as_double_array();
  k_coeff_ = get_node()->get_parameter("friction.k").as_double_array();
  Fv_ = get_node()->get_parameter("friction.Fv").as_double_array();
  Fo_ = get_node()->get_parameter("friction.Fo").as_double_array();

  // Load impedance bounds
  kp_default_ = get_node()->get_parameter("kp_default").as_double_array();
  kd_default_ = get_node()->get_parameter("kd_default").as_double_array();
  kp_max_ = get_node()->get_parameter("kp_max").as_double_array();
  kd_max_ = get_node()->get_parameter("kd_max").as_double_array();
  kp_min_ = get_node()->get_parameter("kp_min").as_double_array();
  kd_min_ = get_node()->get_parameter("kd_min").as_double_array();
  delta_kp_max_ = get_node()->get_parameter("delta_kp_max").as_double();
  delta_kd_max_ = get_node()->get_parameter("delta_kd_max").as_double();

  // Validate array sizes
  auto validate_size = [&](const std::vector<double>& v, const std::string& name) {
    if (v.size() != num_joints_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "%s has %zu elements, expected %zu", name.c_str(), v.size(), num_joints_);
      return false;
    }
    return true;
  };
  if (!validate_size(tau_ff_scale_, "tau_ff_scale") ||
      !validate_size(Fc_, "friction.Fc") ||
      !validate_size(kp_default_, "kp_default") ||
      !validate_size(kp_max_, "kp_max") ||
      !validate_size(kp_min_, "kp_min")) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize current impedance to defaults
  kp_current_ = kp_default_;
  kd_current_ = kd_default_;
  kp_desired_ = kp_default_;
  kd_desired_ = kd_default_;

  // Build KDL chain from robot_description
  // Use a separate node + executor to fetch /robot_description (TRANSIENT_LOCAL)
  // This avoids executor conflict with the controller_manager's executor.
  std::string robot_description;
  {
    auto temp_node = std::make_shared<rclcpp::Node>(
        "_urdf_loader_" + std::to_string(reinterpret_cast<uintptr_t>(this)));

    auto sub = temp_node->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local(),
        [&robot_description](const std_msgs::msg::String::SharedPtr msg) {
          robot_description = msg->data;
        });

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(temp_node);

    auto start = std::chrono::steady_clock::now();
    while (robot_description.empty()) {
      exec.spin_some(std::chrono::milliseconds(10));
      if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Timeout waiting for /robot_description");
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }  // temp_node and exec go out of scope

  RCLCPP_INFO(get_node()->get_logger(),
              "Received robot_description (%zu bytes)", robot_description.size());

  // Parse URDF into KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_description, tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build KDL tree from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Extract chain
  if (!tree.getChain(root_link_, tip_link_, chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to extract KDL chain: %s -> %s",
                 root_link_.c_str(), tip_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (chain_.getNrOfJoints() == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "KDL chain has 0 joints!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Create dynamics solver
  KDL::Vector gravity(0.0, 0.0, -9.81);
  dyn_solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity);
  kdl_initialized_ = true;

  RCLCPP_INFO(get_node()->get_logger(),
              "KDL chain initialized: %u joints, %u segments (%s -> %s)",
              chain_.getNrOfJoints(), chain_.getNrOfSegments(),
              root_link_.c_str(), tip_link_.c_str());

  // Log chain mass info
  double total_mass = 0.0;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    auto seg = chain_.getSegment(i);
    double m = seg.getInertia().getMass();
    total_mass += m;
    if (m > 0.001) {
      auto cog = seg.getInertia().getCOG();
      RCLCPP_INFO(get_node()->get_logger(),
                  "  Segment '%s': mass=%.4f kg, CoG=(%.4f, %.4f, %.4f)",
                  seg.getName().c_str(), m, cog.x(), cog.y(), cog.z());
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "  Total chain mass: %.4f kg", total_mass);

  // Create impedance params subscriber (non-RT thread)
  impedance_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/impedance_params", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() == 2 * num_joints_) {
          rt_impedance_buffer_.writeFromNonRT(msg->data);
        } else {
          RCLCPP_WARN(get_node()->get_logger(),
                      "impedance_params: expected %zu values, got %zu",
                      2 * num_joints_, msg->data.size());
        }
      });

  // Initialize RT buffer with defaults
  std::vector<double> default_cmd;
  default_cmd.insert(default_cmd.end(), kp_default_.begin(), kp_default_.end());
  default_cmd.insert(default_cmd.end(), kd_default_.begin(), kd_default_.end());
  rt_impedance_buffer_.initRT(default_cmd);

  // Create tau_ff publisher
  tau_ff_pub_ = std::make_shared<
      realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/tau_ff", 10));

  // Create gains publisher (publishes actual clamped Kp/Kd for verification)
  gains_pub_ = std::make_shared<
      realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/gains", 10));

  RCLCPP_INFO(get_node()->get_logger(),
              "Compliance controller configured. Scale factors:");
  for (size_t i = 0; i < num_joints_; ++i) {
    RCLCPP_INFO(get_node()->get_logger(),
                "  J%zu: scale=%.2f, kp=[%.1f, %.1f], kd=[%.2f, %.2f]",
                i + 1, tau_ff_scale_[i], kp_min_[i], kp_max_[i],
                kd_min_[i], kd_max_[i]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// on_activate — init Kp/Kd to defaults, zero tau_ff
// ============================================================
controller_interface::CallbackReturn ComplianceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Reset to defaults
  kp_current_ = kp_default_;
  kd_current_ = kd_default_;
  kp_desired_ = kp_default_;
  kd_desired_ = kd_default_;

  // Write initial values to command interfaces
  // Interface order per joint: effort(0), stiffness(1), damping(2)
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces_[i * 3 + 0].set_value(0.0);            // tau_ff = 0
    command_interfaces_[i * 3 + 1].set_value(kp_current_[i]); // stiffness
    command_interfaces_[i * 3 + 2].set_value(kd_current_[i]); // damping
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Compliance controller activated with default gains");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate — restore high-stiffness defaults + zero tau_ff
// ============================================================
controller_interface::CallbackReturn ComplianceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Restore high stiffness and zero feedforward
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces_[i * 3 + 0].set_value(0.0);             // tau_ff = 0
    command_interfaces_[i * 3 + 1].set_value(kp_default_[i]);  // restore stiffness
    command_interfaces_[i * 3 + 2].set_value(kd_default_[i]);  // restore damping
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Compliance controller deactivated, restored default gains");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// update — 100 Hz main loop
// ============================================================
controller_interface::return_type ComplianceController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

  if (!kdl_initialized_) {
    return controller_interface::return_type::OK;
  }

  const unsigned int nj = chain_.getNrOfJoints();

  // ------ 1. Read state interfaces ------
  KDL::JntArray q(nj), qdot(nj);
  for (size_t i = 0; i < num_joints_ && i < nj; ++i) {
    // State interface order per joint: position(0), velocity(1)
    q(i) = state_interfaces_[i * 2 + 0].get_value();
    qdot(i) = state_interfaces_[i * 2 + 1].get_value();
  }

  // ------ 2. Compute tau_ff = scale*(gravity + coriolis) + friction ------
  KDL::JntArray tau_gravity(nj), tau_coriolis(nj);
  std::vector<double> tau_friction(num_joints_, 0.0);

  compute_gravity(q, tau_gravity);
  compute_coriolis(q, qdot, tau_coriolis);
  compute_friction(qdot, tau_friction);

  std::vector<double> tau_ff(num_joints_, 0.0);
  for (size_t i = 0; i < num_joints_ && i < nj; ++i) {
    tau_ff[i] = tau_ff_scale_[i] * (tau_gravity(i) + tau_coriolis(i))
                + tau_friction[i];
  }

  // ------ 3. Read desired Kp/Kd from RT buffer ------
  const auto* cmd = rt_impedance_buffer_.readFromRT();
  if (cmd && cmd->size() == 2 * num_joints_) {
    for (size_t i = 0; i < num_joints_; ++i) {
      kp_desired_[i] = (*cmd)[i];
      kd_desired_[i] = (*cmd)[i + num_joints_];
    }
  }

  // ------ 4. Rate-limit Kp/Kd changes ------
  for (size_t i = 0; i < num_joints_; ++i) {
    double kp_delta = kp_desired_[i] - kp_current_[i];
    kp_delta = std::clamp(kp_delta, -delta_kp_max_, delta_kp_max_);
    kp_current_[i] += kp_delta;

    double kd_delta = kd_desired_[i] - kd_current_[i];
    kd_delta = std::clamp(kd_delta, -delta_kd_max_, delta_kd_max_);
    kd_current_[i] += kd_delta;
  }

  // ------ 5. Clamp to [min, max] ------
  for (size_t i = 0; i < num_joints_; ++i) {
    kp_current_[i] = std::clamp(kp_current_[i], kp_min_[i], kp_max_[i]);
    kd_current_[i] = std::clamp(kd_current_[i], kd_min_[i], kd_max_[i]);
  }

  // ------ 6. Write to command interfaces ------
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces_[i * 3 + 0].set_value(tau_ff[i]);       // effort
    command_interfaces_[i * 3 + 1].set_value(kp_current_[i]);  // stiffness
    command_interfaces_[i * 3 + 2].set_value(kd_current_[i]);  // damping
  }

  // ------ 7. Publish tau_ff for diagnostics (non-blocking) ------
  if (tau_ff_pub_ && tau_ff_pub_->trylock()) {
    tau_ff_pub_->msg_.data = tau_ff;
    tau_ff_pub_->unlockAndPublish();
  }

  // ------ 8. Publish actual gains (clamped values) ------
  if (gains_pub_ && gains_pub_->trylock()) {
    gains_pub_->msg_.data.clear();
    gains_pub_->msg_.data.insert(
        gains_pub_->msg_.data.end(), kp_current_.begin(), kp_current_.end());
    gains_pub_->msg_.data.insert(
        gains_pub_->msg_.data.end(), kd_current_.begin(), kd_current_.end());
    gains_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

// ============================================================
// Helpers
// ============================================================

void ComplianceController::compute_gravity(
    const KDL::JntArray& q, KDL::JntArray& tau_g) {
  dyn_solver_->JntToGravity(q, tau_g);
}

void ComplianceController::compute_coriolis(
    const KDL::JntArray& q, const KDL::JntArray& qdot,
    KDL::JntArray& tau_c) {
  dyn_solver_->JntToCoriolis(q, qdot, tau_c);
}

void ComplianceController::compute_friction(
    const KDL::JntArray& qdot, std::vector<double>& tau_f) {
  // Friction model: amp * Fc * tanh(coef * k * dq) + Fv * dq + Fo
  // Matches openarm_teleop/control.cpp::ComputeFriction
  constexpr double amp = 1.0;
  constexpr double coef = 0.1;
  for (size_t i = 0; i < num_joints_; ++i) {
    double dq = qdot(i);
    tau_f[i] = amp * Fc_[i] * std::tanh(coef * k_coeff_[i] * dq)
               + Fv_[i] * dq
               + Fo_[i];
  }
}

}  // namespace openarm_compliance_controller

PLUGINLIB_EXPORT_CLASS(openarm_compliance_controller::ComplianceController,
                       controller_interface::ControllerInterface)
