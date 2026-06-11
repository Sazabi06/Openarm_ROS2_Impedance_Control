#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <openarm_msgs/srv/set_payload.hpp>
#include <openarm_msgs/srv/set_stiffness.hpp>
#include <openarm_msgs/msg/stress_test_status.hpp>
#include <std_msgs/msg/bool.hpp>

// Pinocchio for real-time inverse dynamics
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "openarm_hw_control/low_pass_filter.hpp"

namespace openarm_hw_control
{
struct PayloadCmd {
    double mass_kg = 0.0;
    double lpf_tau = 0.5;
};

struct StiffnessCmd {
    double kp_cartesian_z = 800.0;
    double kd_cartesian_z = 40.0;
};

class ImpedanceController : public controller_interface::ControllerInterface
{
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
        // ROS Services
    rclcpp::Service<openarm_msgs::srv::SetPayload>::SharedPtr payload_srv_;
    rclcpp::Service<openarm_msgs::srv::SetStiffness>::SharedPtr stiffness_srv_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    
    // Callbacks
    void set_payload_cb(const std::shared_ptr<openarm_msgs::srv::SetPayload::Request> req,
                        std::shared_ptr<openarm_msgs::srv::SetPayload::Response> res);
    void set_stiffness_cb(const std::shared_ptr<openarm_msgs::srv::SetStiffness::Request> req,
                          std::shared_ptr<openarm_msgs::srv::SetStiffness::Response> res);

    std::vector<std::string> joint_names_;

    // Real-time safe buffers (Non-blocking inter-thread comms)
    realtime_tools::RealtimeBuffer<PayloadCmd> rt_payload_buffer_;
    realtime_tools::RealtimeBuffer<StiffnessCmd> rt_stiffness_buffer_;
    realtime_tools::RealtimeBuffer<bool> rt_estop_buffer_;
    std::shared_ptr<realtime_tools::RealtimePublisher<openarm_msgs::msg::StressTestStatus>> status_pub_;

    // Mass injection smooth filter
    LowPassFilter payload_lpf_;

    // Pinocchio Dynamics
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::JointIndex link7_id_;
    double original_link7_mass_ = 0.0;
    Eigen::VectorXd q_, v_, a_;
    Eigen::VectorXd v_zero_;        // preallocated (RT loop must not allocate)
    std::array<int, 7> q_idx_{};    // arm joint i -> Pinocchio configuration index
    std::array<int, 7> v_idx_{};    // arm joint i -> Pinocchio velocity index

    // Control state
    bool in_estop_ = false;
};

} // namespace openarm_hw_control
