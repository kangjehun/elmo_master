#include "elmo_master/elmo_master_node.hpp"

ElmoMasterNode::ElmoMasterNode()
    : Node("elmo_master") {
    // Declare and get the parameters
    init_params();
    
    // Initialize services
    estop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo_estop", 
        std::bind(&ElmoMasterNode::handle_estop, this, std::placeholders::_1, std::placeholders::_2));
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo_reset", 
        std::bind(&ElmoMasterNode::handle_reset, this, std::placeholders::_1, std::placeholders::_2));
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo_start",
        std::bind(&ElmoMasterNode::handle_start, this, std::placeholders::_1, std::placeholders::_2));
    // Initialize publishers
    current_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("current_velocity", 10);
    // Initialize subscribers
    target_velocity_sub_  = this->create_subscription<std_msgs::msg::Float32>("target_velocity", 10,
        std::bind(&ElmoMasterNode::target_velocity_callback, this, std::placeholders::_1));
    // Log that the node is up and running
    RCLCPP_INFO(this->get_logger(), "ElmoMasterNode started...");
    RCLCPP_INFO(this->get_logger(), "ElmoMasterNode running in %s mode", type_.c_str());
}

void ElmoMasterNode::init_params() {
    // Get the parameters from the parameter server
    this->declare_parameter<std::string>("type", "steering");
    this->get_parameter("type", type_);
    if (type_ != "throttle" && type_ != "steering"){
        RCLCPP_WARN(this->get_logger(), 
                    "Invalid type parameter '%s', Defaulting to steering.", type_.c_str());
        type_ = "steering";
    }
}

void ElmoMasterNode::handle_estop(const std_srvs::srv::Trigger::Request::SharedPtr ,
                                  std_srvs::srv::Trigger::Response::SharedPtr response) {
    RCLCPP_WARN(this->get_logger(), "ESTOP Triggerd");
    // TODO : Handle emergency stop logic
    //
    response->success = false;
    response->message = "Emergency Stop is not Implemented Yet.";
}

void ElmoMasterNode::handle_reset(const std_srvs::srv::Trigger::Request::SharedPtr ,
                                  std_srvs::srv::Trigger::Response::SharedPtr response) {
    RCLCPP_WARN(this->get_logger(), "RESET Triggered");
    // TODO : Handle reset logic
    //
    response->success = false;
    response->message = "Reset is not Implemented Yet.";
}

void ElmoMasterNode::handle_start(const std_srvs::srv::Trigger::Request::SharedPtr ,
                                  std_srvs::srv::Trigger::Response::SharedPtr response) {
    RCLCPP_WARN(this->get_logger(), "START Triggered");
    // TODO : Handle start logic
    //
    response->success = false;
    response->message = "Start is not Implemented Yet.";                                    
}

void ElmoMasterNode::target_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    target_velocity_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Target Velocity Received: %f", target_velocity_);
    // TODO : Implement velocity control logic
    // For now, just simulate publishing the current velocity
    current_velocity_ = target_velocity_;
    auto current_velocity_msg = std_msgs::msg::Float32();
    current_velocity_msg.data = current_velocity_;
    current_velocity_pub_->publish(current_velocity_msg);
}