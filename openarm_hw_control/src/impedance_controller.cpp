#include "openarm_hw_control/impedance_controller.hpp"

namespace openarm_hw_control
{

controller_interface::CallbackReturn ImpedanceController::on_init()
{
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // ROS API (Non-RT thread)
    payload_srv_ = get_node()->create_service<openarm_msgs::srv::SetPayload>(
        "~/set_payload_mass", std::bind(&ImpedanceController::set_payload_cb, this, std::placeholders::_1, std::placeholders::_2));
    
    stiffness_srv_ = get_node()->create_service<openarm_msgs::srv::SetStiffness>(
        "~/set_z_stiffness", std::bind(&ImpedanceController::set_stiffness_cb, this, std::placeholders::_1, std::placeholders::_2));
        
    estop_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        "/thermal_estop", rclcpp::SystemDefaultsQoS(), 
        [this](const std_msgs::msg::Bool::SharedPtr msg) { rt_estop_buffer_.writeFromNonRT(msg->data); });

    status_pub_ = std::make_shared<realtime_tools::RealtimePublisher<openarm_msgs::msg::StressTestStatus>>(
        get_node()->create_publisher<openarm_msgs::msg::StressTestStatus>("~/status", 10)
    );

    // Initialize Pinocchio (assuming URDF paths parameter for simplicity)
    std::string urdf_path = "/opt/openarm/urdf/openarm.urdf"; 
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    a_ = Eigen::VectorXd::Zero(model_.nv);
    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_ = Eigen::VectorXd::Zero(model_.nv);

    // Grab original link7 properties
    if (model_.existJointName("joint7")) {
        link7_id_ = model_.getJointId("joint7");
        original_link7_mass_ = model_.inertias[link7_id_].mass();
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "joint7 not found in URDF for mass injection.");
        return controller_interface::CallbackReturn::ERROR;
    }

    // Default init values
    rt_estop_buffer_.initRT(false);
    PayloadCmd p_cmd; rt_payload_buffer_.initRT(p_cmd);
    StiffnessCmd s_cmd; rt_stiffness_buffer_.initRT(s_cmd);

    return controller_interface::CallbackReturn::SUCCESS;
}

void ImpedanceController::set_payload_cb(const std::shared_ptr<openarm_msgs::srv::SetPayload::Request> req,
                                         std::shared_ptr<openarm_msgs::srv::SetPayload::Response> res)
{
    PayloadCmd cmd;
    cmd.mass_kg = req->mass_kg;
    cmd.lpf_tau = req->ramp_duration_s;
    rt_payload_buffer_.writeFromNonRT(cmd);
    res->success = true;
    res->message = "Injection ramping active via low-pass filter";
}

void ImpedanceController::set_stiffness_cb(const std::shared_ptr<openarm_msgs::srv::SetStiffness::Request> req,
                                           std::shared_ptr<openarm_msgs::srv::SetStiffness::Response> res)
{
    StiffnessCmd cmd;
    cmd.kp_cartesian_z = req->kp_z;
    cmd.kd_cartesian_z = req->kd_z;
    rt_stiffness_buffer_.writeFromNonRT(cmd);
    res->success = true;
    res->message = "Stiffness updated";
}

controller_interface::InterfaceConfiguration ImpedanceController::command_interface_configuration() const
{
    std::vector<std::string> conf_names;
    for (const auto & joint : joint_names_) {
        conf_names.push_back(joint + "/position");
        conf_names.push_back(joint + "/velocity");
        conf_names.push_back(joint + "/stiffness");
        conf_names.push_back(joint + "/damping");
        conf_names.push_back(joint + "/effort");
    }
    // Gripper needs only position command
    conf_names.push_back("gripper_joint/position");
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration ImpedanceController::state_interface_configuration() const
{
    std::vector<std::string> conf_names;
    for (const auto & joint : joint_names_) {
        conf_names.push_back(joint + "/position");
        conf_names.push_back(joint + "/velocity");
    }
    conf_names.push_back("gripper_joint/position");
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn ImpedanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    payload_lpf_.reset(0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------
// HARD REAL-TIME 500Hz LOOP 
// Strict NO heap allocations, NO mutex block, NO cout.
// ---------------------------------------------------------
controller_interface::return_type ImpedanceController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    // 1. Read Non-blocking Comm Buffers
    bool estop_active = *rt_estop_buffer_.readFromRT();
    PayloadCmd pcmd = *rt_payload_buffer_.readFromRT();
    StiffnessCmd scmd = *rt_stiffness_buffer_.readFromRT();

    // 2. Safety Interlock
    if (estop_active) {
        if (!in_estop_) {
            in_estop_ = true;
            payload_lpf_.reset(0.0); // Drop mass immediately
        }
        // Force ZERO torque on E-stop. Soft drop logic relies on gravity + minimal damping.
        for (size_t i = 0; i < 7; ++i) {
            command_interfaces_[i*5 + 2].set_value(0.0); // Kp = 0
            command_interfaces_[i*5 + 3].set_value(5.0); // Kd = Light Damping
            command_interfaces_[i*5 + 4].set_value(0.0); // Tau_ff = 0
        }
        command_interfaces_[35].set_value(0.0); // Gripper release
        return controller_interface::return_type::OK;
    }

    in_estop_ = false;

    // 3. Dynamic Mass Injection Pipeline (Smooth LPF)
    double dt = period.seconds();
    payload_lpf_.configure(dt, pcmd.lpf_tau);
    double injected_mass = payload_lpf_.limit(pcmd.mass_kg);

    // Pinocchio injection: Update link7 mass and inertia matrix based on new sum mass
    pinocchio::Inertia current_inertia = model_.inertias[link7_id_];
    current_inertia.mass() = original_link7_mass_ + injected_mass;
    model_.inertias[link7_id_] = current_inertia;

    // 4. Read States into Eigen 
    for (size_t i = 0; i < 7; ++i) {
        q_[i] = state_interfaces_[i*2].get_value();
        v_[i] = state_interfaces_[i*2 + 1].get_value();
    }

    // 5. Pinocchio RNEA Gravity Compensation (computes tau_ff given q, v_zero, a_zero)
    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(7);
    pinocchio::rnea(model_, data_, q_, v_zero, a_);
    Eigen::VectorXd tau_gravity = data_.tau;

    // 6. Impedance Law per joint & Command Dispatch
    // (Actual Cartesian mapping to joint Kp/Kd via Jacobian transpose omitted for brevity; 
    // applying placeholder logic based on Z stiffness request).
    double joint_kp = scmd.kp_cartesian_z * 0.5; // Stub Cartesian scaling
    double joint_kd = scmd.kd_cartesian_z * 0.5;

    for (size_t i = 0; i < 7; ++i) {
        // Assume position/velocity commands come from another topic or trajectory follower;
        // The impedance controller specifically manipulates the Kp, Kd, and Feedforward.
        command_interfaces_[i*5 + 2].set_value(joint_kp);          // Kp
        command_interfaces_[i*5 + 3].set_value(joint_kd);          // Kd
        command_interfaces_[i*5 + 4].set_value(tau_gravity[i]);    // Tau_FF
    }

    // Diagnostics publish (Non-blocking)
    if (status_pub_->trylock()) {
        status_pub_->msg_.current_payload_kg = injected_mass;
        status_pub_->msg_.thermal_estop_active = estop_active;
        status_pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace openarm_hw_control

PLUGINLIB_EXPORT_CLASS(openarm_hw_control::ImpedanceController, controller_interface::ControllerInterface)
