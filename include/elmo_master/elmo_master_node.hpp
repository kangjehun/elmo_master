#ifndef ELMO_MASTER_NODE_HPP
#define ELMO_MASTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

class ElmoMasterNode : public rclcpp::Node {
public:
    ElmoMasterNode();
private:
    // methods
    void init_params();
    // service callbacks
    void handle_estop(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_reset(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_start(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);
    // subscriber callback
    void target_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg); 
    // publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_velocity_pub_;
    // subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_velocity_sub_;
    // services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    // parameters
    std::string type_;
    // variables
    float current_velocity_ = 0.0;
    float target_velocity_  = 0.0;
};

#endif // ELMO_MASTER_NODE_HPP