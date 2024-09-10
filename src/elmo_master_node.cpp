#include "elmo_master/elmo_master_node.hpp"

ElmoMasterNode::ElmoMasterNode()
    : Node("elmo_master"), current_state_(FSMState::INIT), transition_error_(false)
{
    // Declare and get the parameters
    init_params();
    current_state_ = FSMState::INIT;
    transit_to_preop();
    // Initialize services
    estop_service_ = this->create_service<std_srvs::srv::Trigger>(
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
    // Log that the node is up and running
    RCLCPP_INFO(this->get_logger(), "ElmoMasterNode started...");
    RCLCPP_INFO(this->get_logger(), "[Master] type  : %s", type_.c_str());
    RCLCPP_INFO(this->get_logger(), "[Master] state : %d", static_cast<int>(current_state_));
}

std::string ElmoMasterNode::state_to_string(FSMState state) {
    switch (state) {
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
        RCLCPP_ERROR(this->get_logger(), "[Master] Cannot transition to PREOP from %s", state_to_string(current_state_).c_str());
        transition_error_ = true;
        transit_to_exit();
    }
    RCLCPP_INFO(this->get_logger(), "[Master] Transitioning to PREOP State...");
    // [TODO] Implement pre-operational state behavior, read device state, PDO mapping, etc
    current_state_ = FSMState::PREOP;
}

void ElmoMasterNode::transit_to_op()
{
    if (current_state_ == FSMState::PREOP)
    {
        RCLCPP_INFO(this->get_logger(), "[Master] Transitioning to OP State...");
        // [TODO] Implement OP state behavior, such as enabling motor control
        current_state_ = FSMState::OP;
    }
    else if (current_state_ == FSMState::STOP)
    {
        RCLCPP_INFO(this->get_logger(), "[Master] Transitioning to OP State...");
        // [TODO] Implement OP state behavior, such as recovering motor control
        current_state_ = FSMState::OP;
    }
    else if (current_state_ == FSMState::INIT)
    {
        RCLCPP_ERROR(this->get_logger(), "[Master] Cannot transition to OP from %s", state_to_string(current_state_).c_str());
        rclcpp::shutdown();
    } else 
    {
        RCLCPP_ERROR(this->get_logger(), "[Master] Cannot transition to OP from %s", state_to_string(current_state_).c_str());
        transition_error_ = true;
        transit_to_exit();
    }
}

void ElmoMasterNode::transit_to_stop()
{
    if (current_state_ != FSMState::OP)
    {
        RCLCPP_ERROR(this->get_logger(), "[Master] Cannot transition to STOP from %s", state_to_string(current_state_).c_str());
        if (current_state_ == FSMState::INIT)
        {
            rclcpp::shutdown();
        }
        transition_error_ = true;
        transit_to_exit();
    }
    RCLCPP_INFO(this->get_logger(), "[Master] Transitioning to STOP State...");
    // [TODO] Implement STOP state behavior, such as quick-stop the device
    current_state_ = FSMState::STOP;
}

void ElmoMasterNode::transit_to_exit()
{
    if (current_state_ != FSMState::OP) 
    {
        if (!transition_error_)
        {
            RCLCPP_ERROR(this->get_logger(), "[Master] Cannot transition to EXIT from %s", state_to_string(current_state_).c_str());
        }
        if (current_state_ == FSMState::INIT)
        {
            assert(!transition_error_ && "transition_error_ cannot be true in INIT state. Check the transition logic again");
            rclcpp::shutdown();
        }
    } else 
    {
        RCLCPP_INFO(this->get_logger(), "[Master] Transitioning to EXIT State...");
    }
    // [TODO] Implement EXIT state behavior, shutdown devices, stop SYNC messages, etc
    current_state_ = FSMState::EXIT;
    rclcpp::shutdown();
}

void ElmoMasterNode::transit_to_init()
{
    if (current_state_ != FSMState::STOP) 
    {
        RCLCPP_ERROR(this->get_logger(), "[Master] Cannot transition to INIT from %s", state_to_string(current_state_).c_str());
        if (current_state_ == FSMState::INIT)
        {
            rclcpp::shutdown();
        }
        transition_error_ = true;
        transit_to_exit();
    }
    RCLCPP_INFO(this->get_logger(), "[Master] Transitioning to INIT state...");
    // [TODO] Implement INIT state behavior, Reset devices, parameters, etc., and transition to PREOP
    current_state_ = FSMState::INIT;
    transit_to_preop();
}

bool ElmoMasterNode::send_can_message(uint32_t can_id, uint8_t can_dlc, const uint8_t data[8])
{
    // Open a SocketCAN socket
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while opening CAN socket");
        return false;
    }
    // Set up the CAN interface
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error retrieving CAN interface index for %s", can_interface_.c_str());
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
        RCLCPP_ERROR(this->get_logger(), "Error in CAN socket bind on %s", can_interface_.c_str());
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
        RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame on %s", can_interface_.c_str());
        close(sock);
        return false;
    }
    // Log success with formatted CAN message
    char can_data_str[17]; // For storing the 8 bytes of data as a hex string
    snprintf(can_data_str, sizeof(can_data_str), "%02X%02X%02X%02X%02X%02X%02X%02X",
             frame.data[0], frame.data[1], frame.data[2], frame.data[3],
             frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    RCLCPP_INFO(this->get_logger(), "[%s] %03X#%s", can_interface_.c_str(), frame.can_id, can_data_str);
    // Close the socket
    close(sock);
    return true;
}

void ElmoMasterNode::init_params()
{
    // Get the parameters from the parameter server
    this->declare_parameter<std::string>("type", "combined");
    this->declare_parameter<std::string>("can_interface", "can0");

    this->get_parameter("type", type_);
    if (type_ == "throttle" || type_ == "steering")
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Mode '%s' is not implemented yet. Exiting...", type_.c_str());
        rclcpp::shutdown();
    }
    else if (type_ != "combined")
    {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid type parameter '%s', Defaulting to combined.", type_.c_str());
        type_ = "combined";
    }
    this->get_parameter("can_interface", can_interface_);
}

void ElmoMasterNode::handle_stop(const std_srvs::srv::Trigger::Request::SharedPtr,
                                  std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "ESTOP Triggerd");
    // [TODO] Handle emergency stop logic
    //
    response->success = false;
    response->message = "Emergency Stop is not Implemented Yet.";
}

void ElmoMasterNode::handle_reset(const std_srvs::srv::Trigger::Request::SharedPtr,
                                  std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "RESET Triggered");
    // [TODO] Handle reset logic
    uint8_t data[8] = {0x81, 0x00}; // NMT command: Reset all noes, Node ID 0x00 (all nodes)
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
    RCLCPP_WARN(this->get_logger(), "START Triggered");
    // [TODO] Handle start logic
    //
    response->success = false;
    response->message = "Start is not Implemented Yet.";
}

void ElmoMasterNode::target_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    target_velocity_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Target Velocity Received: %f", target_velocity_);
    // [TODO] Implement velocity control logic
    // For now, just simulate publishing the current velocity
    current_velocity_ = target_velocity_;
    auto current_velocity_msg = std_msgs::msg::Float32();
    current_velocity_msg.data = current_velocity_;
    current_velocity_pub_->publish(current_velocity_msg);
}