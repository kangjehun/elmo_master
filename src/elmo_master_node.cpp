#include "elmo_master/elmo_master_node.hpp"
// yaml
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
// socketCAN
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>

ElmoMasterNode::ElmoMasterNode()
    : Node("elmo_master"), current_state_(FSMState::SIBAL), sync_action_(SYNCAction::DISABLE_SYNC), sync_count_(0), can_socket_(-1) 
{
    RCLCPP_WARN(this->get_logger(), "[canX][Master] ElmoMasterNode started...");
    // Initialize callback groups
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sync_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscribe_position_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscribe_velocity_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Init
    init_params();
    init_can_devices(can_config_file_);
    init_can_master_socket();
    init_sync_timer();
    // Services
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo/start",
        std::bind(&ElmoMasterNode::handle_start, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_cb_group_);
    exit_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo/exit",
        std::bind(&ElmoMasterNode::handle_exit, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_cb_group_);
    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo/stop",
        std::bind(&ElmoMasterNode::handle_stop, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_cb_group_);
    recover_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo/recover",
        std::bind(&ElmoMasterNode::handle_recover, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_cb_group_);
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "elmo/reset",
        std::bind(&ElmoMasterNode::handle_reset, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_cb_group_);
    // Publishers
    actual_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("elmo/actual_velocity", 10);
    actual_position_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("elmo/actual_position", 10);
    elmo_status_pub_ = this->create_publisher<std_msgs::msg::String>("elmo/status", 10);
    // Subscribers
    rclcpp::SubscriptionOptions position_sub_options;
    rclcpp::SubscriptionOptions velocity_sub_options;
    position_sub_options.callback_group = subscribe_position_cb_group_;
    velocity_sub_options.callback_group = subscribe_velocity_cb_group_;
    target_velocity_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("elmo/target_velocity", 10,
        std::bind(&ElmoMasterNode::target_velocity_callback, this, std::placeholders::_1),
        velocity_sub_options);
    target_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("elmo/target_position", 10,
        std::bind(&ElmoMasterNode::target_position_callback, this, std::placeholders::_1),
        position_sub_options);
}

ElmoMasterNode::~ElmoMasterNode()
{
    if (can_socket_ >= 0)
    {
        close(can_socket_);
        RCLCPP_INFO(this->get_logger(), "[%s][Master] Socket closed", can_interface_.c_str());
    }
    RCLCPP_WARN(this->get_logger(), "[%s][Master] ElmoMasterNode is being destroyed", can_interface_.c_str());
}

void ElmoMasterNode::siginit_exit()
{
    RCLCPP_WARN(this->get_logger(), "SIGINT (ctrl+C) received. Shutting down...");
    transit_to_exit();
}

void ElmoMasterNode::init_params()
{
    // Get the parameters from the parameter server
    this->declare_parameter<std::string>("type", "combined");
    this->declare_parameter<std::string>("can_interface", "can0");
    this->declare_parameter<std::string>("can_config_file", "config/elmo_can0.yaml");
    this->declare_parameter<int32_t>("resolution_position", 33554432); // 2^25
    this->declare_parameter<int32_t>("resolution_velocity", 33554432); // 2^25
    this->declare_parameter<float>("max_position", M_PI);; // pi
    this->declare_parameter<float>("min_position", -M_PI); // -pi
    this->declare_parameter<float>("max_velocity", 1.0f); // 1[m/s] for safety
    this->declare_parameter<float>("wheel_radius", 0.1f); // 0.1[m] for test motor
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
    if (!this->get_parameter("can_config_file", can_config_file_))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to retrieve parameter 'can_config_file'.",
                     can_interface_.c_str());
        rclcpp::shutdown();
    }
    if (!this->get_parameter("resolution_position", resolution_position_))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to retrieve parameter 'resolution_position'.", 
            can_interface_.c_str());
        rclcpp::shutdown();
    }

    if (!this->get_parameter("resolution_velocity", resolution_velocity_))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to retrieve parameter 'resolution_velocity'.", 
            can_interface_.c_str());
        rclcpp::shutdown();
    }

    if (!this->get_parameter("max_position", max_position_))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to retrieve parameter 'max_position'.", can_interface_.c_str());
        rclcpp::shutdown();
    }

    if (!this->get_parameter("min_position", min_position_))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to retrieve parameter 'min_position'.", can_interface_.c_str());
        rclcpp::shutdown();
    }

    if (!this->get_parameter("max_velocity", max_velocity_))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to retrieve parameter 'max_velocity'.", can_interface_.c_str());
        rclcpp::shutdown();
    }

    if (!this->get_parameter("wheel_radius", wheel_radius_))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to retrieve parameter 'min_velocity'.", can_interface_.c_str());
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "[%s][Master] - type      : %s", can_interface_.c_str(), type_.c_str());
    RCLCPP_INFO(this->get_logger(), "[%s][Master] - interface : %s", can_interface_.c_str(), can_interface_.c_str());
    RCLCPP_INFO(this->get_logger(), "[%s][Master] - resolution_position: %d", can_interface_.c_str(), resolution_position_);
    RCLCPP_INFO(this->get_logger(), "[%s][Master] - resolution_velocity: %d", can_interface_.c_str(), resolution_velocity_);
    RCLCPP_INFO(this->get_logger(), "[%s][Master] - max_position: %f", can_interface_.c_str(), max_position_);
    RCLCPP_INFO(this->get_logger(), "[%s][Master] - min_position: %f", can_interface_.c_str(), min_position_);
    RCLCPP_INFO(this->get_logger(), "[%s][Master] - max_velocity: %f", can_interface_.c_str(), max_velocity_);
    RCLCPP_INFO(this->get_logger(), "[%s][Master] - wheel_radius: %f", can_interface_.c_str(), wheel_radius_);
}

void ElmoMasterNode::init_can_devices(const std::string &config_file)
{
    try
    {
        // resolve absolute path
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("elmo_master");
        std::string full_path = package_share_directory + "/" + config_file;

        // load config file
        YAML::Node config = YAML::LoadFile(full_path);
        // extract CAN master info
        const YAML::Node &master = config["master"];
        sync_interval_ms_ = master["sync_interval_ms"].as<int>();
        // extract CAN slaves (devices) info
        const YAML::Node &devices = config["nodes"];
        for (const auto &device : devices)
        {
            CANDevice can_device;
            can_device.id = device["node_id"].as<int>();
            can_device.name = device["node_name"].as<std::string>();
            can_device.type = device["node_type"].as<std::string>();
            can_device.status = DeviceStatus::NOT_READY_TO_SWITCH_ON;
            can_device.set_point = true;
            can_device.target_velocity.velocity_uu = 0;
            can_device.target_position.position_uu = 0;
            can_device.actual_velocity.velocity_uu = 0;
            can_device.actual_position.position_uu = 0;
            can_devices_.push_back(can_device);
        }
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Loaded %zu CAN devices from config file %s",
                    can_interface_.c_str(), can_devices_.size(), config_file.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to load CAN devices info: %s",
                     can_interface_.c_str(), e.what());
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(this->get_logger(), "[%s][Master] - master      sync: %dms", can_interface_.c_str(), sync_interval_ms_);
    for (const auto &device : can_devices_)
    {
        RCLCPP_INFO(this->get_logger(), "[%s][Master] - nodeID: %02d, name: %s, type: %s",
                    can_interface_.c_str(),
                    device.id, device.name.c_str(), device.type.c_str());
    }
    return;
}

void ElmoMasterNode::init_can_master_socket()
{
    // Open a SocketCAN socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] socket open failed", can_interface_.c_str());
        rclcpp::shutdown();
        return;
    }
    // Set up the CAN interface index
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] socket index failed", can_interface_.c_str());
        close(can_socket_);
        rclcpp::shutdown();
        return;
    }
    // Set up the CAN address structure
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    // Bind the socket to the CAN interface
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] socket bind failed", can_interface_.c_str());
        close(can_socket_);
        rclcpp::shutdown();
        return;
    }
    RCLCPP_WARN(this->get_logger(), "[%s][Master] Socket opened", can_interface_.c_str());
    // Get and print the receive buffer size
    int recv_buf_size = 0;
    socklen_t optlen = sizeof(recv_buf_size);
    if (getsockopt(can_socket_, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, &optlen) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] socket getsockopt failed", can_interface_.c_str());
        close(can_socket_);
        rclcpp::shutdown();
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "[%s][Master] - receive buffer size: %d", can_interface_.c_str(), recv_buf_size);
    }
}

void ElmoMasterNode::init_sync_timer()
{
    sync_start_time_ = this->now();
    sync_timer_ = this->create_wall_timer(std::chrono::milliseconds(sync_interval_ms_),
                                          std::bind(&ElmoMasterNode::sync_callback, this),
                                          sync_cb_group_);
    RCLCPP_WARN(this->get_logger(), "[%s][Master] Sync timer initialized with %dms interval",
                can_interface_.c_str(), sync_interval_ms_);
}

bool ElmoMasterNode::transit_to_ready()
{
    std::unique_lock<std::mutex> lock(sync_mutex_);
    if (current_state_ != FSMState::INIT)
    {
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Cannot transition to READY from %s",
                    can_interface_.c_str(), state_to_string(current_state_).c_str());
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State : %s", 
                    can_interface_.c_str(), state_to_string(current_state_).c_str());
        lock.unlock();
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to READY State...", can_interface_.c_str());
    // Set all nodes to operational
    CANMessage set_operational_msg = {0x000, 2, {0x01, 0x00}, COBType::NMT, "(NMT) Set All Nodes to Operational"};
    if (!send_can_message(set_operational_msg, 0))
    {
        lock.unlock();
        transit_to_exit(true);
        return false;
    }
    // for (const auto &device : can_devices_)
    // {
    //     uint8_t node_id = device.id;
    //     CANMessage set_operational_msg = {0x000, 2, {0x01, node_id}, COBType::NMT, "(NMT) Set Node to Operational"};
    //     if (!send_can_message(set_operational_msg, node_id))
    //     {
    //         lock.unlock();
    //         transit_to_exit(true);
    //         return false;
    //     }
    // }
    RCLCPP_WARN(this->get_logger(), "[%s][Master] SYNC enabled", can_interface_.c_str());
    sync_start_time_ = this->now();
    sync_action_ = SYNCAction::ENABLE_SYNC;
    sync_cv_.wait(lock, [this]() { return sync_action_ == SYNCAction::ENABLE_SYNC && sync_count_ == 1; });
    sync_action_ = SYNCAction::PDO_CONTROL;
    sync_cv_.wait(lock, [this]() { return sync_action_ == SYNCAction::PDO_CONTROL; });
    // Transit to READY state
    current_state_ = FSMState::READY;
    RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State: %s",
                can_interface_.c_str(), state_to_string(current_state_).c_str());
    lock.unlock();
    // Transit to OP state automatically
    transit_to_op();
    return true;
}

bool ElmoMasterNode::transit_to_op()
{   
    std::unique_lock<std::mutex> lock(sync_mutex_);
    if (current_state_ == FSMState::READY || current_state_ == FSMState::STOP)
    {
        RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to OP State...", can_interface_.c_str());
        sync_action_ = SYNCAction::RESET_AND_ENABLE_DEVICE;
        sync_cv_.wait(lock,
        [this]()
        {
            for (const auto &device : can_devices_)
            {
                if (device.status != DeviceStatus::OPERATION_ENABLED)
                {
                    return false;
                }
            }
            return true;
        });
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Cannot transition to OP from %s",
                    can_interface_.c_str(), state_to_string(current_state_).c_str());
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State : %s", 
                    can_interface_.c_str(), state_to_string(current_state_).c_str());
        lock.unlock();             
        return false;
    }
    current_state_ = FSMState::OP;
    sync_action_ = SYNCAction::PDO_CONTROL;
    sync_cv_.wait(lock, [this]() { return sync_action_ == SYNCAction::PDO_CONTROL; });
    RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State: %s",
                can_interface_.c_str(), state_to_string(current_state_).c_str());
    lock.unlock();
    return true;
}

bool ElmoMasterNode::transit_to_stop()
{
    std::unique_lock<std::mutex> lock(sync_mutex_);
    if (current_state_ != FSMState::OP)
    {
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Cannot transition to STOP from %s",
                     can_interface_.c_str(), state_to_string(current_state_).c_str());
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State : %s", 
                    can_interface_.c_str(), state_to_string(current_state_).c_str());
        lock.unlock();
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to STOP State...", can_interface_.c_str());
    sync_action_ = SYNCAction::STOP_DEVICE;
    sync_cv_.wait(lock, 
    [this]()
    {
        for (const auto& device : can_devices_)
        {
            if (device.status != DeviceStatus::SWITCH_ON_DISABLED)
            {
                return false;
            }
        }
        return true;
    });
    // Transit to STOP state
    current_state_ = FSMState::STOP;
    RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State: %s",
                can_interface_.c_str(), state_to_string(current_state_).c_str());
    lock.unlock();
    return true;
}

bool ElmoMasterNode::transit_to_init(bool first_init)
{
    std::unique_lock<std::mutex> lock(sync_mutex_);
    if (first_init)
    {
        if (current_state_ != FSMState::SIBAL)
        {
            RCLCPP_WARN(this->get_logger(), "[%s][Master] Cannot transition to INIT from %s",
                         can_interface_.c_str(), state_to_string(current_state_).c_str());
            RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State : %s", 
                        can_interface_.c_str(), state_to_string(current_state_).c_str());
            lock.unlock();
            return false;
        }
    }
    else
    {
        if (current_state_ != FSMState::STOP)
        {
            RCLCPP_WARN(this->get_logger(), "[%s][Master] Cannot transition to INIT from %s",
                         can_interface_.c_str(), state_to_string(current_state_).c_str());
            RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State : %s", 
                        can_interface_.c_str(), state_to_string(current_state_).c_str());
            lock.unlock();
            return false;
        }
    }
    RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to INIT state...", can_interface_.c_str());
    // Reset all nodes
    sync_action_ = SYNCAction::DISABLE_SYNC;
    sync_cv_.wait(lock, [this]() { return sync_action_ == SYNCAction::DISABLE_SYNC && sync_count_ == 0; });
    for (const auto &device : can_devices_)
    {
        uint8_t node_id = device.id;
        // create NMT Reset message
        CANMessage reset_msg = {0x000, 2, {0x81, node_id}, COBType::NMT, "(NMT) Reset Node"};
        // send NMT Reset message
        if (!send_and_check_can_message(reset_msg, node_id)) { lock.unlock(); transit_to_exit(true); return false; }
    }
    // PDO mapping
    for (const auto &device : can_devices_)
    {
        uint8_t node_id = device.id;
        uint32_t cob_id = static_cast<uint32_t>(0x600 + node_id);
        RCLCPP_INFO(this->get_logger(), "node-id : %d", node_id);
        // create TPDO1 Mapping messages
        CANMessage tpdo1_map_msg_01 = {cob_id, 8, {0x23, 0x00, 0x18, 0x01, static_cast<uint8_t>(0x80 + node_id), 0x01, 0x00, 0x80}, COBType::RSDO, 
            "(SDO, Wreq) Disable TPDO1"}; 
        CANMessage tpdo1_map_msg_02 = {cob_id, 8, {0x2F, 0x00, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Set Transmission 1sync"};
        CANMessage tpdo1_map_msg_03 = {cob_id, 8, {0x2B, 0x00, 0x18, 0x03, 0x64, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Set Inhibit Time 100us"};
        CANMessage tpdo1_map_msg_04 = {cob_id, 8, {0x2B, 0x00, 0x18, 0x05, 0x0A, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Set Event Time 10ms"};
        CANMessage tpdo1_map_msg_05 = {cob_id, 8, {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Initialize TPDO1 entry number as 0"};
        CANMessage tpdo1_map_msg_06 = {cob_id, 8, {0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) TPDO1 Mapping: Sub 01, Statusword, 16bit"};
        CANMessage tpdo1_map_msg_07 = {cob_id, 8, {0x23, 0x00, 0x1A, 0x02, 0x08, 0x00, 0x61, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) TPDO1 mapping: Sub 02, Mode of Operation Display, 8bit"};
        CANMessage tpdo1_map_msg_08 = {cob_id, 8, {0x23, 0x00, 0x1A, 0x03, 0x20, 0x00, 0x6C, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) TPDO1 mapping: Sub 03, Velocity Actual Value, 32bit"};
        CANMessage tpdo1_map_msg_09 = {cob_id, 8, {0x2F, 0x00, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Rewrite TPDO1 entry number to 3"};
        CANMessage tpdo1_map_msg_10 = {cob_id, 8, {0x23, 0x00, 0x18, 0x01, static_cast<uint8_t>(0x80 + node_id), 0x01, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Enable TPDO1"};
        // create TPDO2 Mapping messages
        CANMessage tpdo2_map_msg_01 = {cob_id, 8, {0x23, 0x01, 0x18, 0x01, static_cast<uint8_t>(0x80 + node_id), 0x02, 0x00, 0x80}, COBType::RSDO, 
            "(SDO, Wreq) Disable TPDO2"};
        CANMessage tpdo2_map_msg_02 = {cob_id, 8, {0x2F, 0x01, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Set Transmission 1sync"};
        CANMessage tpdo2_map_msg_03 = {cob_id, 8, {0x2B, 0x01, 0x18, 0x03, 0x64, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Set Inhibit Time 100us"};
        CANMessage tpdo2_map_msg_04 = {cob_id, 8, {0x2B, 0x01, 0x18, 0x05, 0x0A, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Set Event Time 10ms"};
        CANMessage tpdo2_map_msg_05 = {cob_id, 8, {0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Initialize TPDO2 entry number as 0"};
        CANMessage tpdo2_map_msg_06 = {cob_id, 8, {0x23, 0x01, 0x1A, 0x01, 0x20, 0x00, 0x64, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) TPDO2 mapping: Sub 01, Position Actual Value, 32bit"};
        CANMessage tpdo2_map_msg_07 = {cob_id, 8, {0x23, 0x01, 0x1A, 0x02, 0x10, 0x00, 0x77, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) TPDO2 mapping: Sub 02, Torque Actual Value, 32bit"};
        CANMessage tpdo2_map_msg_08 = {cob_id, 8, {0x2F, 0x01, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Rewrite TPDO2 entry number to 2"};
        CANMessage tpdo2_map_msg_09 = {cob_id, 8, {0x23, 0x01, 0x18, 0x01, static_cast<uint8_t>(0x80 + node_id), 0x02, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Enable TPDO2"};
        // create RPDO1 Mapping messages
        CANMessage rpdo1_map_msg_01 = {cob_id, 8, {0x23, 0x00, 0x14, 0x01, static_cast<uint8_t>(0x00 + node_id), 0x02, 0x00, 0x80}, COBType::RSDO, 
            "(SDO, Wreq) Disable RPDO1"};
        CANMessage rpdo1_map_msg_02 = {cob_id, 8, {0x2F, 0x00, 0x14, 0x02, 0x01, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Set Transmission 1sync"};
        CANMessage rpdo1_map_msg_03 = {cob_id, 8, {0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Initialize RPDO1 entry number"};
        CANMessage rpdo1_map_msg_04 = {cob_id, 8, {0x23, 0x00, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) RPDO1 mapping: Sub 01, Controlword, 16bit"};
        CANMessage rpdo1_map_msg_05 = {cob_id, 8, {0x23, 0x00, 0x16, 0x02, 0x08, 0x00, 0x60, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) RPDO1 mapping: Sub 02, Mode of Operation, 8bit"};
        CANMessage rpdo1_map_msg_06 = {cob_id, 8, {0x23, 0x00, 0x16, 0x03, 0x20, 0x00, 0xFF, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) RPDO1 mapping: Sub 03, Target Velocity, 32bit"};
        CANMessage rpdo1_map_msg_07 = {cob_id, 8, {0x2F, 0x00, 0x16, 0x00, 0x03, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Rewrite RPDO1 entry number to 2"};
        CANMessage rpdo1_map_msg_08 = {cob_id, 8, {0x23, 0x00, 0x14, 0x01, static_cast<uint8_t>(0x00 + node_id), 0x02, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Enable RPDO1"};
        // create RPDO2 Mapping messages
        CANMessage rpdo2_map_msg_01 = {cob_id, 8, {0x23, 0x01, 0x14, 0x01, static_cast<uint8_t>(0x00 + node_id), 0x03, 0x00, 0x80}, COBType::RSDO, 
            "(SDO, Wreq) Disable RPDO2"};
        CANMessage rpdo2_map_msg_02 = {cob_id, 8, {0x2F, 0x01, 0x14, 0x02, 0x01, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Set Transmission 1sync"};
        CANMessage rpdo2_map_msg_03 = {cob_id, 8, {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Initialize RPDO2 entry number"};
        CANMessage rpdo2_map_msg_04 = {cob_id, 8, {0x23, 0x01, 0x16, 0x01, 0x20, 0x00, 0x7A, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) RPDO2 mapping: Sub 01, Target Position, 32bit"};
        CANMessage rpdo2_map_msg_05 = {cob_id, 8, {0x23, 0x01, 0x16, 0x02, 0x10, 0x00, 0x71, 0x60}, COBType::RSDO, 
            "(SDO, Wreq) RPDO2 mapping: Sub 02, Target Torque, 32bit"};
        CANMessage rpdo2_map_msg_06 = {cob_id, 8, {0x2F, 0x01, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Rewrite RPDO2 entry number to 2"};
        CANMessage rpdo2_map_msg_07 = {cob_id, 8, {0x23, 0x01, 0x14, 0x01, static_cast<uint8_t>(0x00 + node_id), 0x03, 0x00, 0x00}, COBType::RSDO, 
            "(SDO, Wreq) Enable RPDO2"};
        // send TPDO1 Mapping messages
        if (!send_and_check_can_message(tpdo1_map_msg_01, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo1_map_msg_02, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo1_map_msg_03, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo1_map_msg_04, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo1_map_msg_05, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo1_map_msg_06, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo1_map_msg_07, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo1_map_msg_08, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo1_map_msg_09, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo1_map_msg_10, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        // send TPDO2 Mapping messages
        if (!send_and_check_can_message(tpdo2_map_msg_01, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo2_map_msg_02, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo2_map_msg_03, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo2_map_msg_04, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo2_map_msg_05, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo2_map_msg_06, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo2_map_msg_07, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo2_map_msg_08, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(tpdo2_map_msg_09, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        // send RPDO1 Mapping messages
        if (!send_and_check_can_message(rpdo1_map_msg_01, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo1_map_msg_02, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo1_map_msg_03, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo1_map_msg_04, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo1_map_msg_05, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo1_map_msg_06, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo1_map_msg_07, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo1_map_msg_08, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        // send RPDO2 Mapping messages
        if (!send_and_check_can_message(rpdo2_map_msg_01, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo2_map_msg_02, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo2_map_msg_03, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo2_map_msg_04, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo2_map_msg_05, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo2_map_msg_06, node_id)) { lock.unlock(); transit_to_exit(); return false; }
        if (!send_and_check_can_message(rpdo2_map_msg_07, node_id)) { lock.unlock(); transit_to_exit(); return false; }
    }
    // Transit to INIT state
    current_state_ = FSMState::INIT;
    RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State: %s",
                can_interface_.c_str(), state_to_string(current_state_).c_str());
    lock.unlock();
    // Transit to READY state automatically
    transit_to_ready();
    return true;
}

bool ElmoMasterNode::transit_to_exit(bool transition_error)
{   
    std::unique_lock<std::mutex> lock(sync_mutex_);
    if (current_state_ != FSMState::OP && !transition_error)
    {
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Cannot transition to EXIT from %s. Exit immediately",
                     can_interface_.c_str(), state_to_string(current_state_).c_str());
        transition_error = true;
    }
    if (current_state_ != FSMState::INIT)
    {
        sync_action_ = SYNCAction::TERMINATE;
        sync_cv_.wait(lock, [this]() { return sync_action_ == SYNCAction::TERMINATE; });
        if (current_state_ != FSMState::READY)
        {
            sync_cv_.wait(lock, 
            [this]() 
            {
                for (const auto &device : can_devices_)
                {
                    if (device.status != DeviceStatus::SWITCH_ON_DISABLED) 
                    { 
                        return false; 
                    }
                }
                return true;
            });
        }
        for (const auto &device : can_devices_)
        {
            uint8_t node_id = device.id;
            // create exit message
            CANMessage exit_msg = {0x000, 2, {0x81, node_id}, COBType::NMT, "(NMT) Reset Node"};
            // send exit message
            if (!send_and_check_can_message(exit_msg, node_id)) { lock.unlock(); }
        }
    }
    if (transition_error)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Transition to EXIT state due to transition error...", can_interface_.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "[%s][Master] Transition to EXIT state...", can_interface_.c_str());
    }
    // Transit to EXIT state
    current_state_ = FSMState::EXIT;
    RCLCPP_WARN(this->get_logger(), "[%s][Master] Current State: %s", 
        can_interface_.c_str(), state_to_string(current_state_).c_str());
    lock.unlock();
    rclcpp::shutdown();
    return true;
}

bool ElmoMasterNode::send_can_message(const CANMessage &message, uint8_t node_id)
{
    // Create can frame
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = message.can_id;
    frame.can_dlc = message.can_dlc;
    memcpy(frame.data, message.data, message.can_dlc);
    // Set the socket to non-blocking mode
    int flags = fcntl(can_socket_, F_GETFL, 0);
    fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
    // Flush the socket buffer
    struct can_frame flush_frame;
    while (read(can_socket_, &flush_frame, sizeof(flush_frame)) > 0) { }
    // Send the CAN frame
    if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] socket write, NodeID %d", can_interface_.c_str(), node_id);
        return false;
    }
    if (message.cob_type == COBType::SYNC)
    {
        // RCLCPP_INFO(this->get_logger(), "[%s][Master] %s", can_interface_.c_str(), message.description.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "[%s][Master] → [Node%02d] %s",
                    can_interface_.c_str(), node_id, message.description.c_str());
    }
    return true;
}

bool ElmoMasterNode::send_and_check_can_message(const CANMessage &message, uint8_t node_id)
{
    // Send the message
    if (!send_can_message(message, node_id))
    {
        return false;
    }
    // Get the expected response ID
    uint32_t expected_response_id = get_expected_response_id(message, node_id);
    // Poll for the response
    struct can_frame response;
    const int sleep_duration_us = 10000; // 10ms
    const int max_attempts = 100;        // 10ms * 100attemps = 1s
    for (int attemps = 0; attemps < max_attempts; ++attemps)
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(can_socket_, &readfds);
        // set up the timeout
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = sleep_duration_us;
        // poll the socket
        int ret = select(can_socket_ + 1, &readfds, nullptr, nullptr, &timeout);
        // RCLCPP_WARN(this->get_logger(), "[%s][Master] select ret: %d", can_interface_.c_str(), ret); // [DEBUG] 
        if (ret < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s][Master] socket select, NodeID %d", can_interface_.c_str(), node_id);
            return false;
        }
        else if (ret > 0 && FD_ISSET(can_socket_, &readfds))
        {
            int nbytes = read(can_socket_, &response, sizeof(response));
            if (nbytes > 0)
            {
                // Check EMCY Error
                if (response.can_id == static_cast<canid_t>(0x080 + node_id))
                {
                    RCLCPP_ERROR(this->get_logger(), "[%s][Node%02d] - EMCY : %s",
                                 can_interface_.c_str(), node_id, format_emergency_message(response).c_str());
                    return false;
                }
                // Check if the response ID matches the expected response
                if (response.can_id != expected_response_id)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "[%s][Node%02d] - Unexpected response (%s) about reqeust (%s)",
                                 can_interface_.c_str(), node_id,
                                 cobtype_to_string(hex_to_cobtype(response.can_id)).c_str(),
                                 cobtype_to_string(message.cob_type).c_str());
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] → [Master] OK", can_interface_.c_str(), node_id);
                return true;
            }
        }
        usleep(sleep_duration_us);
    }
    RCLCPP_ERROR(this->get_logger(), "[%s][Master] socket read timeout, NodeID %d", can_interface_.c_str(), node_id);
    return false;
}

bool ElmoMasterNode::check_pdo_message()
{
    struct can_frame response;
    const int sleep_duration_us = 10000; // 10ms
    const int max_attempts = 100;        // 10ms * 100attemps = 1s

    std::vector<bool> tpdo1_received(can_devices_.size(), false);
    std::vector<bool> tpdo2_received(can_devices_.size(), false);

    for (int attempts = 0; attempts < max_attempts; ++attempts)
    {
        // Poll for the response
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(can_socket_, &readfds);
        // Set up the timeout
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = sleep_duration_us;
        // Poll the socket
        int ret = select(can_socket_ + 1, &readfds, nullptr, nullptr, &timeout);
        if (ret < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s][Master] socket select", can_interface_.c_str());
            return false;
        }
        else if (ret > 0 && FD_ISSET(can_socket_, &readfds))
        {
            int nbytes = read(can_socket_, &response, sizeof(response));
            if (nbytes > 0)
            {
                uint8_t node_id = response.can_id & 0x7F; // Extract node ID from the CAN ID
                if (node_id > 0 && node_id <= can_devices_.size())
                {
                    // Check EMCY Error
                    if (response.can_id == static_cast<canid_t>(0x080 + node_id))
                    {
                        RCLCPP_ERROR(this->get_logger(), "[%s][Node%02d] - EMCY : %s",
                                     can_interface_.c_str(), node_id, format_emergency_message(response).c_str());
                        return false;
                    }
                    // Check TPDO1
                    else if (response.can_id == static_cast<canid_t>(0x180 + node_id))
                    {
                        tpdo1_received[node_id - 1] = true;
                        // Extract actual velocity value
                        memcpy(can_devices_[node_id - 1].actual_velocity.bytes, response.data + 3, 4);
                        std::reverse(std::begin(can_devices_[node_id - 1].actual_velocity.bytes),
                                     std::end(can_devices_[node_id - 1].actual_velocity.bytes));
                        int32_t velocity_uu = 
                            (static_cast<int32_t>(can_devices_[node_id - 1].actual_velocity.bytes[0]) << 24) |
                            (static_cast<int32_t>(can_devices_[node_id - 1].actual_velocity.bytes[1]) << 16) |
                            (static_cast<int32_t>(can_devices_[node_id - 1].actual_velocity.bytes[2]) << 8)  |
                            (static_cast<int32_t>(can_devices_[node_id - 1].actual_velocity.bytes[3]));
                        RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] - TPDO1 Actual Velocity : %d [uu]",
                                    can_interface_.c_str(), node_id, velocity_uu);
                        RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] - TPDO1 Actual Velocity : %f [m/s]",
                                    can_interface_.c_str(), node_id, uu_to_mps(velocity_uu));
                    }
                    // Check TPDO2
                    else if (response.can_id == static_cast<canid_t>(0x280 + node_id))
                    {
                        tpdo2_received[node_id - 1] = true;
                        // Extract actual position value
                        memcpy(can_devices_[node_id - 1].actual_position.bytes, response.data, 4);
                        std::reverse(std::begin(can_devices_[node_id - 1].actual_position.bytes),
                                     std::end(can_devices_[node_id - 1].actual_position.bytes));
                        int32_t position_uu =
                            (static_cast<int32_t>(can_devices_[node_id - 1].actual_position.bytes[0]) << 24) |
                            (static_cast<int32_t>(can_devices_[node_id - 1].actual_position.bytes[1]) << 16) |
                            (static_cast<int32_t>(can_devices_[node_id - 1].actual_position.bytes[2]) << 8)  |
                            (static_cast<int32_t>(can_devices_[node_id - 1].actual_position.bytes[3]));
                        RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] - TPDO2 Actual Position : %d [uu]",
                                    can_interface_.c_str(), node_id, position_uu);
                        RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] - TPDO2 Actual Position : %f [rad]",
                                    can_interface_.c_str(), node_id, uu_to_rad(position_uu));
                    }
                    // Check both TPDO1 and TPDO2 received
                    if (tpdo1_received[node_id - 1] && tpdo2_received[node_id - 1])
                    {
                        RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] → [Master] OK", can_interface_.c_str(), node_id);
                        return true;
                    }
                }
            }
        }
        usleep(sleep_duration_us);
    }
    RCLCPP_ERROR(this->get_logger(), "[%s][Master] socket read timeout", can_interface_.c_str());
    return false;
}


void ElmoMasterNode::publish_actual_values()
{
    std_msgs::msg::Float32MultiArray actual_velocity_msg;
    std_msgs::msg::Float32MultiArray actual_position_msg;
    for (const auto &device : can_devices_)
    {
        actual_velocity_msg.data.push_back(uu_to_mps(device.actual_velocity.velocity_uu));
        actual_position_msg.data.push_back(uu_to_rad(device.actual_position.position_uu));
    }
    actual_velocity_pub_->publish(actual_velocity_msg);
    actual_position_pub_->publish(actual_position_msg);
}

COBType ElmoMasterNode::hex_to_cobtype(uint32_t cob_id) const
{
    // Extract the function code & NodeID from COB-ID (Elmo: Gold DS301 Implementation Guide, p.16-17)
    uint8_t function_code = (cob_id >> 7) & 0x0F;
    uint8_t node_id = cob_id & 0x7F;
    switch (function_code)
    {
    case 0x0:
        return COBType::NMT;
    case 0x1:
        return node_id == 0 ? COBType::SYNC : COBType::EMCY;
    case 0x2:
        return COBType::TS;
    case 0x3:
        return COBType::TPDO1;
    case 0x4:
        return COBType::RPDO1;
    case 0x5:
        return COBType::TPDO2;
    case 0x6:
        return COBType::RPDO2;
    case 0x7:
        return COBType::TPDO3;
    case 0x8:
        return COBType::RPDO3;
    case 0x9:
        return COBType::TPDO4;
    case 0xA:
        return COBType::RPDO4;
    case 0xB:
        return COBType::TSDO;
    case 0xC:
        return COBType::RSDO;
    case 0xE:
        return COBType::NG;
    default:
        return COBType::UNKNOWN;
    }
}

std::string ElmoMasterNode::state_to_string(FSMState state) const
{
    switch (state)
    {
    case FSMState::SIBAL:
        return "SIBAL";
    case FSMState::INIT:
        return "INIT";
    case FSMState::READY:
        return "READY";
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

std::string ElmoMasterNode::cobtype_to_string(COBType type) const
{
    switch (type)
    {
    case COBType::NMT:
        return "NMT";
    case COBType::SYNC:
        return "SYNC";
    case COBType::TS:
        return "Time Stamp";
    case COBType::EMCY:
        return "Emergency";
    case COBType::TPDO1:
        return "PDO1-Tansmit";
    case COBType::RPDO1:
        return "PDO1-Receieve";
    case COBType::TPDO2:
        return "PDO2-Transmit";
    case COBType::RPDO2:
        return "PDO2-Receive";
    case COBType::TPDO3:
        return "PDO3-Transmit";
    case COBType::RPDO3:
        return "PDO3-Receive";
    case COBType::TPDO4:
        return "PDO4-Transmit";
    case COBType::RPDO4:
        return "PDO4-Receieve";
    case COBType::TSDO:
        return "SDO-Transmit";
    case COBType::RSDO:
        return "SDO-Receive";
    case COBType::NG:
        return "Node Guarding (Error Control)";
    default:
        return "UNKNOWN";
    }
}

std::string ElmoMasterNode::format_emergency_message(const struct can_frame &response) const
{
    std::stringstream ss;
    ss
        << std::endl
        << "   Error Code = 0x" << std::hex << static_cast<int>(response.data[0]) << static_cast<int>(response.data[1])
        << std::endl
        << "   Error Register = 0x" << std::hex << static_cast<int>(response.data[2])
        << std::endl
        << "   Error Code = 0x" << static_cast<int>(response.data[3])
        << std::endl
        << "   Data Field 1 = 0x" << std::hex << static_cast<int>(response.data[4]) << static_cast<int>(response.data[5])
        << std::endl
        << "   Data Field 2 = 0x" << std::hex << static_cast<int>(response.data[6]) << static_cast<int>(response.data[7]);

    return ss.str();
}

uint32_t ElmoMasterNode::get_expected_response_id(const CANMessage &message, uint8_t node_id) const
{
    switch (message.cob_type)
    {
    case COBType::NMT:
        return 0x700 + node_id;
    case COBType::RSDO:
        return 0x580 + node_id;
    case COBType::RPDO1:
        return 0x180 + node_id;
    case COBType::RPDO2:
        return 0x280 + node_id;
    case COBType::RPDO3:
        return 0x380 + node_id;
    case COBType::RPDO4:
        return 0x480 + node_id;
    default:
        RCLCPP_WARN(this->get_logger(), "[%s][Master] No Expected Response for COBType %s for NodeID %d",
                    can_interface_.c_str(), cobtype_to_string(message.cob_type).c_str(), node_id);
        return 0;
    }
}

float ElmoMasterNode::uu_to_rad(int32_t position_uu) 
{
    float radian = min_position_ + (max_position_ - min_position_) * static_cast<float>(position_uu - (-resolution_position_ / 2)) / static_cast<float>(resolution_position_);
    // while (radian > M_PI)
    // {
    //     RCLCPP_WARN(this->get_logger(), "[%s][Master] Radian Overflow: %f", can_interface_.c_str(), radian);
    //     radian -= 2 * M_PI;
    // }
    // while (radian < -M_PI)
    // {
    //     RCLCPP_WARN(this->get_logger(), "[%s][Master] Radian Underflow: %f", can_interface_.c_str(), radian);
    //     radian += 2 * M_PI;
    // }
    return radian;
}

float ElmoMasterNode::rad_to_uu(float position) 
{
    // while (position > M_PI)
    // {
    //     RCLCPP_WARN(this->get_logger(), "[%s][Master] Radian Overflow: %f", can_interface_.c_str(), position);
    //     position -= 2 * M_PI;
    // }
    // while (position < -M_PI)
    // {
    //     RCLCPP_WARN(this->get_logger(), "[%s][Master] Radian Underflow: %f", can_interface_.c_str(), position);
    //     position += 2 * M_PI;
    // }
    return static_cast<int32_t>(-0.5 * static_cast<float>(resolution_position_) 
                                + static_cast<float>(resolution_position_) * ((position - min_position_) / (max_position_ - min_position_)));
}

float ElmoMasterNode::uu_to_mps(int32_t velocity_uu) 
{
    // return min_velocity_ + (max_velocity_ - min_velocity_) * static_cast<float>(velocity_uu - (-resolution_velocity_ / 2)) / static_cast<float>(resolution_velocity_);
    return static_cast<float>(velocity_uu * (2 * M_PI * wheel_radius_) / resolution_velocity_);
}

float ElmoMasterNode::mps_to_uu(float velocity) 
{
    // return static_cast<int32_t>(-0.5 * static_cast<float>(resolution_velocity_) 
    //                             + static_cast<float>(resolution_velocity_) * ((velocity - min_velocity_) / (max_velocity_ - min_velocity_)));
    return static_cast<int32_t>(velocity * resolution_velocity_ / (2 * M_PI * wheel_radius_));
}

float ElmoMasterNode::rad_to_deg(float radian)
{
    float degree = radian * (180.f / M_PI);
    while (degree > 180.0f)
    {
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Degree Overflow: %f", can_interface_.c_str(), degree);
        degree -= 360.0f;
    }
    while (degree < -180.0f)
    {
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Degree Underflow: %f", can_interface_.c_str(), degree);
        degree += 360.0f;
    }
    return degree;
}

float ElmoMasterNode::deg_to_rad(float degree)
{
    float radian = degree * (M_PI / 180.0f);
    // Wrap the angle between -pi and pi radians
    while (radian > M_PI)
    {
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Radian Overflow: %f", can_interface_.c_str(), radian);
        radian -= 2 * M_PI;
    }
    while (radian < -M_PI)
    {
        RCLCPP_WARN(this->get_logger(), "[%s][Master] Radian Underflow: %f", can_interface_.c_str(), radian);
        radian += 2 * M_PI;
    }
    return radian;
}

void ElmoMasterNode::handle_start(const std_srvs::srv::Trigger::Request::SharedPtr,
                                  std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "[%s][Master] START Triggered", can_interface_.c_str());
    response->success = transit_to_init();
    if (response->success) { response->message = "Start Successfully"; }
    else { response->message = "Start Failed"; }
}

void ElmoMasterNode::handle_exit(const std_srvs::srv::Trigger::Request::SharedPtr,
                                 std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "[%s][Master] EXIT Triggered", can_interface_.c_str());
    
    response->success = transit_to_exit();
    if (response->success) { response->message = "Exit Successfully"; }
    else { response->message = "Exit Failed"; }
}

void ElmoMasterNode::handle_stop(const std_srvs::srv::Trigger::Request::SharedPtr,
                                 std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "[%s][Master] STOP Triggerd", can_interface_.c_str());
    response->success = transit_to_stop();
    if (response->success) { response->message = "Stop Successfully"; }
    else { response->message = "Stop Failed"; }
}

void ElmoMasterNode::handle_recover(const std_srvs::srv::Trigger::Request::SharedPtr,
                                    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "[%s][Master] RECOVER Triggered", can_interface_.c_str());
    response->success = transit_to_op();
    if (response->success) { response->message = "Recover Successfully"; }
    else { response->message = "Recover Failed"; }
}

void ElmoMasterNode::handle_reset(const std_srvs::srv::Trigger::Request::SharedPtr,
                                  std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_WARN(this->get_logger(), "[%s][Master] RESET Triggered", can_interface_.c_str());
    response->success = transit_to_init(false);
    if (response->success) { response->message = "Reset Successfully"; }
    else { response->message = "Reset Failed"; }
}

void ElmoMasterNode::target_velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(sync_mutex_);
    if (msg->data.size() != can_devices_.size())
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Target Velocity size mismatch with devices, can device size %lu and message size %lu",
                     can_interface_.c_str(), can_devices_.size(), msg->data.size());
        lock.unlock();
        call_exit_service();
        return;
    }
    for (size_t i = 0; i < can_devices_.size(); ++i)
    {
        float target_velocity = msg->data[i];
        if (fabs(target_velocity) >= max_velocity_)
        {
            RCLCPP_WARN(this->get_logger(), "[%s][Master] Clipping Target Velocity for Node%02lu, %f",
                        can_interface_.c_str(), i + 1, target_velocity);
            target_velocity = std::max(std::min(target_velocity, max_velocity_), - max_velocity_);
        }
        can_devices_[i].target_velocity.velocity_uu = mps_to_uu(target_velocity);
    }
    lock.unlock();
    return;
}

void ElmoMasterNode::target_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(sync_mutex_);
    if (msg->data.size() != can_devices_.size())
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Target Position size mismatch with devices", can_interface_.c_str());
        lock.unlock();
        call_exit_service();
        return;
    }
    for (size_t i = 0; i < can_devices_.size(); ++i)
    {
        float target_position = msg->data[i];
        if (fabs(target_position) >= max_position_)
        {
            RCLCPP_WARN(this->get_logger(), "[%s][Master] Clipping Target Position for Node%02lu, %f",
                        can_interface_.c_str(), i + 1, target_position);
            target_position = std::max(std::min(target_position, max_position_), min_position_);
        }
        can_devices_[i].target_position.position_uu = rad_to_uu(target_position);
    }
    lock.unlock();
    return;
}

void ElmoMasterNode::sync_callback()
{
    if (sync_action_ == SYNCAction::DISABLE_SYNC)
    {
        sync_start_time_ = this->now();
        sync_count_ = 0;
        sync_cv_.notify_all();
        return;
    }
    else if (sync_action_ == SYNCAction::RESET_AND_ENABLE_DEVICE)
    {
        for (auto &device : can_devices_)
        {
            uint8_t node_id = device.id;
            uint32_t cob_id = static_cast<uint32_t>(0x200 + node_id);
            switch (device.status)
            {
                case DeviceStatus::FAULT:
                case DeviceStatus::NOT_READY_TO_SWITCH_ON:
                case DeviceStatus::QUICK_STOP_ACTIVE:
                {
                    // Fault Reset
                    CANMessage enable_msg_01 = {cob_id, 7, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, COBType::RPDO1,
                        "(RPDO1) CW : Fault Reset"};
                    if (!send_can_message(enable_msg_01, node_id)) { call_exit_service(); return; }
                    device.status = DeviceStatus::SWITCH_ON_DISABLED;
                    break;
                }
                case DeviceStatus::SWITCH_ON_DISABLED:
                {
                    // Shutdown
                    CANMessage enable_msg_02 = {cob_id, 7, {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, COBType::RPDO1,
                        "(RPDO1) CW : Shutdown"};
                    if (!send_can_message(enable_msg_02, node_id)) { call_exit_service(); return; }
                    device.status = DeviceStatus::READY_TO_SWITCH_ON;
                    break;
                }
                case DeviceStatus::READY_TO_SWITCH_ON:
                {
                    // Switch ON
                    CANMessage enable_msg_03 = {cob_id, 7, {0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, COBType::RPDO1,
                        "(RPDO1) CW : Switch On"};
                    if (!send_can_message(enable_msg_03, node_id)) { call_exit_service(); return; }
                    device.status = DeviceStatus::SWITCHED_ON;
                    break;
                }
                case DeviceStatus::SWITCHED_ON:
                {
                    // Enable Operation
                    CANMessage enable_msg_04;
                    if (device.type == "throttle")
                    {
                        enable_msg_04 = {cob_id, 7, {0x0F, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00}, COBType::RPDO1,
                            "(RPDO1) CW : Enable Operation (Throttle)"};
                    }
                    else if (device.type == "steering")
                    {
                        enable_msg_04 = {cob_id, 7, {0x0F, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00}, COBType::RPDO1,
                            "(RPDO1) CW : Enable Operation (Steering)"};
                    }
                    if (!send_can_message(enable_msg_04, node_id)) { call_exit_service(); return; }
                    device.status = DeviceStatus::OPERATION_ENABLED;
                    break;
                }
                case DeviceStatus::OPERATION_ENABLED:
                {
                    sync_cv_.notify_all();
                    break;
                }
                default:
                {
                    RCLCPP_ERROR(this->get_logger(), "[%s][Master] Invalid Device Status", can_interface_.c_str());
                    call_exit_service();
                    return;
                }
            }
        }
    }
    else if (sync_action_ == SYNCAction::STOP_DEVICE || sync_action_ == SYNCAction::TERMINATE)
    {
        for (auto &device : can_devices_)
        {
            uint8_t node_id = device.id;
            uint32_t cob_id = static_cast<uint32_t>(0x200 + node_id);
            switch (device.status)
            {
                case DeviceStatus::SWITCH_ON_DISABLED:
                {
                    break;
                }
                case DeviceStatus::QUICK_STOP_ACTIVE:
                {
                    device.status = DeviceStatus::SWITCH_ON_DISABLED;
                    sync_cv_.notify_all();
                    break;
                }
                default:
                {
                    // Quick Stop
                    CANMessage stop_msg = {cob_id, 7, {0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, COBType::RPDO1,
                        "(RPDO1) CW : Quick Stop"};
                    if (!send_can_message(stop_msg, node_id)) { call_exit_service(); return; }
                    device.status = DeviceStatus::QUICK_STOP_ACTIVE;
                    break;
                }
            }
        }
        if (sync_action_ == SYNCAction::TERMINATE)
        {
            sync_cv_.notify_all();
            return;
        }
    }
    else if (sync_action_ == SYNCAction::PDO_CONTROL)
    {
        sync_cv_.notify_all();
        if (! check_pdo_message()) { call_exit_service(); return; }
        for (auto &device : can_devices_)
        {
            uint8_t node_id = device.id;
            uint32_t cob_id_rpdo1 = static_cast<uint32_t>(0x200 + node_id);
            uint32_t cob_id_rpdo2 = static_cast<uint32_t>(0x300 + node_id);
            // Check PDO messages and update actual values
            if (device.type == "throttle")
            {   
                // create RPDO message
                std::stringstream description;
                description << "(RPDO1) CW : Enable Operation, MO : Throttle, TV : " 
                            << uu_to_mps(device.target_velocity.velocity_uu) << "[m/s]";
                CANMessage rpdo_msg = {cob_id_rpdo1, 7, {0x0F, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00}, COBType::RPDO1,
                                       description.str()};
                // Print the 4 bytes of target_velocity before reversing
                int32_t velocity_uu = device.target_velocity.velocity_uu;
                // RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] - RPDO1 Target Velocity Bytes (Before Reverse): %02X %02X %02X %02X",
                //             can_interface_.c_str(), node_id,
                //             (velocity_uu >> 24) & 0xFF,
                //             (velocity_uu >> 16) & 0xFF,
                //             (velocity_uu >> 8) & 0xFF,
                //             velocity_uu & 0xFF);
                // // Reverse the byte order (Big Endian to Little Endian)
                // int32_t reversed_velocity_uu = 
                //     ((velocity_uu & 0xFF000000) >> 24) |
                //     ((velocity_uu & 0x00FF0000) >> 8)  |
                //     ((velocity_uu & 0x0000FF00) << 8)  |
                //     ((velocity_uu & 0x000000FF) << 24);
                // // Print the 4 bytes of target_velocity after reversing
                // RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] - RPDO1 Target Velocity Bytes (After Reverse): %02X %02X %02X %02X",
                //             can_interface_.c_str(), node_id,
                //             (reversed_velocity_uu >> 24) & 0xFF,
                //             (reversed_velocity_uu >> 16) & 0xFF,
                //             (reversed_velocity_uu >> 8) & 0xFF,
                //             reversed_velocity_uu & 0xFF);
                // memcpy(rpdo_msg.data + 3, &reversed_velocity_uu, sizeof(reversed_velocity_uu));
                memcpy(rpdo_msg.data + 3, &velocity_uu, sizeof(velocity_uu));
                if (!send_can_message(rpdo_msg, node_id)) { call_exit_service(); return; }
            }
            else if (device.type == "steering")
            {
                // create RPDO messages
                CANMessage rpdo1_msg = {cob_id_rpdo1, 7, {static_cast<uint8_t>(device.set_point ? 0x0F : 0x3F), 
                                        0x00, 0x01, 0x00, 0x00, 0x00, 0x00},
                                        COBType::RPDO1, 
                                        device.set_point ? "(RPDO1) CW : Enable Operation & Set Point, MO : Steering, TV : 0" :
                                                           "(RPDO1) CW : Enable Operation & Clear Point, MO : Steering, TV : 0"};
                std::stringstream rpdo2_description;
                rpdo2_description << "(RPDO2) TP : " << uu_to_rad(device.target_position.position_uu) << "[rad]"
                                  << ", TT : 0";
                CANMessage rpdo2_msg = {cob_id_rpdo2, 6, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, COBType::RPDO2,
                                        rpdo2_description.str()};
                // Print the 4 bytes of target_position before reversing
                int32_t position_uu = device.target_position.position_uu;
                // RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] - RPDO2 Target Position Bytes (Before Reverse): %02X %02X %02X %02X",
                //             can_interface_.c_str(), node_id,
                //             (position_uu >> 24) & 0xFF,
                //             (position_uu >> 16) & 0xFF,
                //             (position_uu >> 8) & 0xFF,
                //             position_uu & 0xFF);
                // // Reverse the byte order (Big Endian to Little Endian)
                // int32_t reversed_position_uu = 
                //     ((position_uu & 0xFF000000) >> 24) |
                //     ((position_uu & 0x00FF0000) >> 8)  |
                //     ((position_uu & 0x0000FF00) << 8)  |
                //     ((position_uu & 0x000000FF) << 24);
                // // Print the 4 bytes of target_position after reversing
                // RCLCPP_INFO(this->get_logger(), "[%s][Node%02d] - RPDO2 Target Position Bytes (After Reverse): %02X %02X %02X %02X",
                //             can_interface_.c_str(), node_id,
                //             (reversed_position_uu >> 24) & 0xFF,
                //             (reversed_position_uu >> 16) & 0xFF,
                //             (reversed_position_uu >> 8) & 0xFF,
                //             reversed_position_uu & 0xFF);
                // Copy reversed position into rpdo2_msg data
                // memcpy(rpdo2_msg.data, &reversed_position_uu, sizeof(reversed_position_uu));
                memcpy(rpdo2_msg.data, &position_uu, sizeof(position_uu));
                if (!send_can_message(rpdo1_msg, node_id)) { call_exit_service(); return; }
                if (!send_can_message(rpdo2_msg, node_id)) { call_exit_service(); return; }
                device.set_point = !device.set_point;
            }    
            publish_actual_values();
            // TODO : publish state
        }
    }
    else if (sync_action_ == SYNCAction::ENABLE_SYNC)
    {
        // RCLCPP_INFO(this->get_logger(), "[%s][Master] SYNC Count: %d", can_interface_.c_str(), sync_count_); // [DEBUG]
        sync_cv_.notify_all();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Invalid SYNC Action", can_interface_.c_str());
        call_exit_service();
        return;
    }
    // Send SYNC message
    double passed_time = (this->now() - sync_start_time_).seconds();
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << passed_time;
    CANMessage sync_msg = {0x080, 0, {}, COBType::SYNC, "(SYNC)" + ss.str() + "s"};
    if (!send_can_message(sync_msg, 0)) { call_exit_service(); return; }
    sync_count_++;
}

void ElmoMasterNode::call_exit_service()
{
    auto client = this->create_client<std_srvs::srv::Trigger>("elmo/exit");
    if (!client->wait_for_service(std::chrono::seconds(1))) 
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Failed to call /elmo/exit service: Service not available", 
            can_interface_.c_str());
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    try 
    {
        auto response = future.get();
        if (!response->success) 
        {
            RCLCPP_ERROR(this->get_logger(), "[%s][Master] /elmo/exit service call failed: %s", 
                can_interface_.c_str(), response->message.c_str());
        }
    } 
    catch (const std::exception &e) 
    {
        RCLCPP_ERROR(this->get_logger(), "[%s][Master] Exception while calling /elmo/exit service: %s", 
            can_interface_.c_str(), e.what());
    }
}