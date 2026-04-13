#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class ThermalWatchdog : public rclcpp::Node {
public:
    ThermalWatchdog() : Node("thermal_watchdog") {
        estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/thermal_estop", 10);
        warning_pub_ = this->create_publisher<std_msgs::msg::Bool>("/thermal_warning", 10);
        
        temp_sub_left_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/left_joint_temperatures", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                check_temperatures(msg, 0);
            });
        temp_sub_right_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/right_joint_temperatures", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                check_temperatures(msg, 8); // Offset 8 for right arm joints
            });
            
        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/thermal_watchdog/reset_estop",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                handle_reset_request(request, response);
            });
            
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() { log_cooldown_status(); }
        );
            
        RCLCPP_INFO(this->get_logger(), "75C Thermal Watchdog with auto-reset initialized.");
    }

private:
    void check_temperatures(const std_msgs::msg::Float64MultiArray::SharedPtr msg, int offset) {
        // Update local array
        for (size_t i = 0; i < msg->data.size() && (offset + i) < joint_temps_.size(); ++i) {
            joint_temps_[offset + i] = msg->data[i];
        }

        bool estop_condition = false;
        bool warning_condition = false;
        double max_t = 0.0;
        int max_t_idx = -1;
        
        // Find global max
        for (size_t i = 0; i < joint_temps_.size(); ++i) {
            if (joint_temps_[i] > max_t) {
                max_t = joint_temps_[i];
                max_t_idx = i;
            }
        }
        
        if (max_t >= 75.0) {
            estop_condition = true;
        } else if (max_t >= 65.0) {
            warning_condition = true;
        }

        // Hysteresis logic
        bool all_below_60 = (max_t < 60.0);

        // ESTOP LOGIC
        if (estop_condition && !is_estopped_) {
            RCLCPP_FATAL(this->get_logger(), "THERMAL ESTOP TRIGGERED! Max Temp: %.1fC on channel %d. Commencing soft drop.", max_t, max_t_idx);
            is_estopped_ = true;
            estop_timestamp_ = this->now();
            cooldown_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type()); // Reset cooldown
        }
        
        // AUTO RESET LOGIC
        if (is_estopped_) {
            if (all_below_60) {
                if (cooldown_start_time_.nanoseconds() == 0) {
                    cooldown_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "All motors below 60C. Starting 30s cooldown timer.");
                } else if ((this->now() - cooldown_start_time_).seconds() >= 30.0) {
                    RCLCPP_INFO(this->get_logger(), "30s cooldown complete. Auto-resetting E-stop.");
                    is_estopped_ = false;
                    cooldown_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
                }
            } else {
                cooldown_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
            }
        }

        // WARNING LOGIC
        if (warning_condition && !is_warning_) {
            RCLCPP_WARN(this->get_logger(), "THERMAL WARNING: Temp %.1fC on channel %d. Emitting warning.", max_t, max_t_idx);
            is_warning_ = true;
        } else if (is_warning_ && all_below_60) {
            RCLCPP_INFO(this->get_logger(), "Thermal warning cleared.");
            is_warning_ = false;
        }
        
        // PUBLISH STATES
        std_msgs::msg::Bool estop_msg;
        estop_msg.data = is_estopped_;
        estop_pub_->publish(estop_msg);
        
        std_msgs::msg::Bool warning_msg;
        warning_msg.data = is_warning_;
        warning_pub_->publish(warning_msg);
        
        current_max_t_ = max_t;
    }

    void handle_reset_request(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (!is_estopped_) {
            res->success = true;
            res->message = "System is not currently E-stopped.";
            RCLCPP_INFO(this->get_logger(), "Manual reset requested, but system is not E-stopped.");
            return;
        }

        if (current_max_t_ < 65.0) {
            is_estopped_ = false;
            cooldown_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
            res->success = true;
            res->message = "E-stop manually cleared.";
            RCLCPP_INFO(this->get_logger(), "E-stop manually cleared via service.");
        } else {
            res->success = false;
            char msg[100];
            snprintf(msg, sizeof(msg), "Cannot clear E-stop. Max temp is %.1fC (must be < 65C)", current_max_t_);
            res->message = msg;
            RCLCPP_WARN(this->get_logger(), "Manual reset rejected: %s", msg);
        }
    }
    
    void log_cooldown_status() {
        if (is_estopped_) {
            if (cooldown_start_time_.nanoseconds() > 0) {
                double elapsed = (this->now() - cooldown_start_time_).seconds();
                RCLCPP_INFO(this->get_logger(), "Cooling down: max temp = %.1fC, time below 60C = %.1fs/30s", current_max_t_, elapsed);
            } else {
                RCLCPP_INFO(this->get_logger(), "E-stopped: max temp = %.1fC. Waiting to reach 60C for cooldown.", current_max_t_);
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr warning_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr temp_sub_left_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr temp_sub_right_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<double> joint_temps_ = std::vector<double>(16, 0.0);
    bool is_estopped_ = false;
    bool is_warning_ = false;
    double current_max_t_ = 0.0;
    
    rclcpp::Time estop_timestamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time cooldown_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThermalWatchdog>());
    rclcpp::shutdown();
    return 0;
}
