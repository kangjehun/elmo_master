#include "elmo_master/elmo_master_node.hpp"

ElmoMasterNode::ElmoMasterNode()
    : Node("elmo_master"), current_state_(FSMState::INIT), transition_error_flag_(false)
{
    // Declare and get the parameters
    init_params();
    // Log that the node is up and running
    RCLCPP_INFO(this->get_logger(), "ElmoMasterNode started...");
    RCLCPP_INFO(this->get_logger(), "[%s][Master] type      : %s", can_interface_.c_str(), type_.c_str());
    RCLCPP_INFO(this->get_logger(), "[%s][Master] interface : %s", can_interface_.c_str(), can_interface_.c_str());
    // Initialize the state machine
    current_state_ = FSMState::INIT;
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Current State : %s", can_interface_.c_str(), state_to_string(current_state_).c_str());
    transit_to_preop();
    // Initialize services
    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo_estop",
        std::bind(&ElmoMasterNode::handle_stop, this, std::placeholders::_1, std::placeholders::_2));
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo_reset",
        std::bind(&ElmoMasterNode::handle_reset, this, std::placeholders::_1, std::placeholders::_2));
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo_start",
        std::bind(&ElmoMasterNode::handle_start, this, std::placeholders::_1, std::placeholders::_2));
    // Initialize publishers
    current_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("current_velocity", 10);
    // Initialize subscribers
    target_velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>("target_velocity", 10,
                            std::bind(&ElmoMasterNode::target_velocity_callback, this, std::placeholders::_1));
}

ElmoMasterNode::~ElmoMasterNode() {
    RCLCPP_INFO(this->get_logger(), "[%s][Master] ElmoMasterNode is being destroyed.", can_interface_.c_str());
}

void ElmoMasterNode::siginit_exit() {
    RCLCPP_WARN(this->get_logger(), "SIGINT (ctrl+C) received. Shutting down...");
    transit_to_exit();
}

std::string ElmoMasterNode::state_to_string(FSMState state) {
    switch (state) 
    {
        case FSMState::INIT:
            return "INIT";
        case FSMState::PREOP:
            return "PREOP";
        case FSMState::OP:
            return "OP";
        case FSMState::STOP:
            return "STOP";
        case FSMState::EXIT:
            return "EXIT";
        default:
            return "UNKNOWN";
    }
}

void ElmoMasterNode::transit_to_preop()
{
    if (current_state_ != FSMState::INIT)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Cannot transition to PREOP from %s", 
                     can_interface_.c_str(), state_to_string(current_state_).c_str());
        transition_error_flag_ = true;
        transit_to_exit();
    }
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to PREOP State...", can_interface_.c_str());
    // [TODO] Implement pre-operational state behavior, read device state, PDO mapping, etc
    // ...
    current_state_ = FSMState::PREOP;
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Current State: %s", 
                can_interface_.c_str(), state_to_string(current_state_).c_str());
}

void ElmoMasterNode::transit_to_op()
{
    if (current_state_ == FSMState::PREOP)
    {
        RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to OP State...", can_interface_.c_str());
        // [TODO] Implement OP state behavior, such as enabling motor control
        // ...
        current_state_ = FSMState::OP;
        RCLCPP_INFO(this->get_logger(), "[%s][Master] Current State: %s", 
                can_interface_.c_str(), state_to_string(current_state_).c_str());
    }
    else if (current_state_ == FSMState::STOP)
    {
        RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to OP State...", can_interface_.c_str());
        // [TODO] Implement OP state behavior, such as recovering motor control
        // ...
        current_state_ = FSMState::OP;
        RCLCPP_INFO(this->get_logger(), "[%s][Master] Current State: %s", 
                can_interface_.c_str(), state_to_string(current_state_).c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Cannot transition to OP from %s", 
                     can_interface_.c_str(), state_to_string(current_state_).c_str());
        transition_error_flag_ = true;
        transit_to_exit();
    }
}

void ElmoMasterNode::transit_to_stop()
{
    if (current_state_ != FSMState::OP)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Cannot transition to STOP from %s", 
                     can_interface_.c_str(), state_to_string(current_state_).c_str());
        transition_error_flag_ = true;
        transit_to_exit();
    }
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to STOP State...", can_interface_.c_str());
    // [TODO] Implement STOP state behavior, such as quick-stop the device
    // ...
    current_state_ = FSMState::STOP;
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Current State: %s", 
                can_interface_.c_str(), state_to_string(current_state_).c_str());
}

void ElmoMasterNode::transit_to_init()
{
    if (current_state_ != FSMState::STOP) 
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Cannot transition to INIT from %s", 
                     can_interface_.c_str(), state_to_string(current_state_).c_str());
        transition_error_flag_ = true;
        transit_to_exit();
    }
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to INIT state...", can_interface_.c_str());
    // [TODO] Implement INIT state behavior, Reset devices, parameters, etc., and transition to PREOP
    // ...
    current_state_ = FSMState::INIT;
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Current State: %s", 
                can_interface_.c_str(), state_to_string(current_state_).c_str());
    transit_to_preop();
}

void ElmoMasterNode::transit_to_exit()
{
    if (current_state_ != FSMState::OP && !transition_error_flag_) 
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Cannot transition to EXIT from %s", 
                     can_interface_.c_str(), state_to_string(current_state_).c_str());
        transition_error_flag_ = true;
    }
    if (current_state_ != FSMState::INIT)
    {
        // [TODO] Implement EXIT state behavior, shutdown devices, stop SYNC messages, etc
        // ...
    }
    if (transition_error_flag_)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Transition to EXIT state due to transition error...", can_interface_.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to EXIT state...", can_interface_.c_str());
    }
    current_state_ = FSMState::EXIT;
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Current State: %s", 
                can_interface_.c_str(), state_to_string(current_state_).c_str());
    rclcpp::shutdown();
}

bool ElmoMasterNode::send_can_message(uint32_t can_id, uint8_t can_dlc, const uint8_t data[8])
{
    // Open a SocketCAN socket
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Error while opening CAN socket", can_interface_.c_str());
        return false;
    }
    // Set up the CAN interface
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Error retrieving CAN interface index", can_interface_.c_str());
        close(sock);
        return false;
    }
    // Set up the CAN address structure
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    // Bind the socket to the CAN interface
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Error in CAN socket bind", can_interface_.c_str());
        close(sock);
        return false;
    }
    // Create the CAN frame
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = can_dlc;
    memcpy(frame.data, data, can_dlc);
    // Send the CAN frame
    if (write(sock, &frame, sizeof(frame)) != sizeof(frame))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Error sending CAN frame", can_interface_.c_str());
        close(sock);
        return false;
    }
    // Log success with formatted CAN message
    char can_data_str[17]; // For storing the 8 bytes of data as a hex string
    snprintf(can_data_str, sizeof(can_data_str), "%02X%02X%02X%02X%02X%02X%02X%02X",
             frame.data[0], frame.data[1], frame.data[2], frame.data[3],
             frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    RCLCPP_INFO(this->get_logger(), "[%s][Master] %03X#%s", can_interface_.c_str(), frame.can_id, can_data_str);
    // Close the socket
    close(sock);
    return true;
}

void ElmoMasterNode::init_params()
{
    // Get the parameters from the parameter server
    this->declare_parameter<std::string>("type", "combined");
    this->declare_parameter<std::string>("can_interface", "can0");

    if (!this->get_parameter("can_interface", can_interface_))
    {
        RCLCPP_ERROR(this->get_logger(), "[-][Master] Failed to retrieve parameter 'can_interface'.");
        rclcpp::shutdown();
    }
    if (can_interface_ != "can0" && can_interface_ != "can1")
    {
        RCLCPP_ERROR(this->get_logger(), "[-][Master] CAN interface '%s' is not supported.", can_interface_.c_str());
        rclcpp::shutdown();
    }
    if (!this->get_parameter("type", type_)) 
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to retrieve parameter 'type'.", can_interface_.c_str());
        rclcpp::shutdown();
    }
    if (type_ != "combined")
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Mode '%s' is not implemented yet.", 
                     can_interface_.c_str(), type_.c_str());
        rclcpp::shutdown();
    }
}

void ElmoMasterNode::handle_stop(const std_srvs::srv::Trigger::Request::SharedPtr,
                                  std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "[%s][Master] ESTOP Triggerd", can_interface_.c_str());
    // [TODO] Handle emergency stop logic
    //
    response->success = false;
    response->message = "Emergency Stop is not Implemented Yet.";
}

void ElmoMasterNode::handle_reset(const std_srvs::srv::Trigger::Request::SharedPtr,
                                  std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "[%s][Master] RESET Triggered", can_interface_.c_str());
    // [TODO] Handle reset logic
    uint8_t data[8] = {0x81, 0x00}; // NMT command: Reset all nodes, Node ID 0x00 (all nodes)
    // Use the new send_can_message function
    if (send_can_message(0x000, 2, data))
    {
        response->success = true;
        response->message = "Reset all nodes successfully";
    }
    else
    {
        response->success = false;
        response->message = "Failed to send CAN message";
    }
}

void ElmoMasterNode::handle_start(const std_srvs::srv::Trigger::Request::SharedPtr,
                                  std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "[%s][Master] START Triggered", can_interface_.c_str());
    // [TODO] Handle start logic
    //
    response->success = false;
    response->message = "Start is not Implemented Yet.";
}

void ElmoMasterNode::target_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    target_velocity_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Target Velocity Received: %f", can_interface_.c_str(), target_velocity_);
    // [TODO] Implement velocity control logic
    // For now, just simulate publishing the current velocity
    current_velocity_ = target_velocity_;
    auto current_velocity_msg = std_msgs::msg::Float32();
    current_velocity_msg.data = current_velocity_;
    current_velocity_pub_->publish(current_velocity_msg);
}