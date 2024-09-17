#ifndef ELMO_MASTER_NODE_HPP
#define ELMO_MASTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

// FSM States
enum class FSMState {
    INIT,
    READY, // Pre-operational
    OP,    // Operational
    STOP,
    EXIT
};

// COB Types
enum class COBType 
{
    NMT,
    SYNC,
    TS,     // Time Stamp
    EMCY,   // Emergency
    TPDO1,
    RPDO1,
    TPDO2,
    RPDO2,
    TPDO3,
    RPDO3,
    TPDO4,
    RPDO4,
    TSDO,
    RSDO,
    NG,    // Node Guarding
    UNKNOWN
};

// CAN message structure
struct CANMessage
{
    uint32_t    can_id;
    uint8_t     can_dlc;
    uint8_t     data[8];
    COBType     cob_type;
    std::string description;
};

// CAN device info
struct CANDevice
{
    int id;
    std::string name;
    std::string type;
};

class ElmoMasterNode : public rclcpp::Node 
{
public:
    ElmoMasterNode();
    ~ElmoMasterNode();
    void siginit_exit();
private:
    // methods
    void init_params();
    void init_can_devices(const std::string &config_file);
    void init_can_master_socket();
    void init_sync_timer();
    void transit_to_ready();
    void transit_to_op();
    void transit_to_stop();
    // void transit_to_init(bool first_init = false);
    void transit_to_init();
    void transit_to_exit(bool transition_error = false);
    bool send_and_check_can_message(const CANMessage &message, uint8_t node_id);
    bool send_can_message(const CANMessage &message, uint8_t node_id);
    void send_sync_message();
    static COBType hex_to_cobtype(uint32_t cob_id);
    static std::string state_to_string(FSMState state);
    static std::string cobtype_to_string(COBType cob_type);
    static std::string format_emergency_message(const struct can_frame &response);
    uint32_t get_expected_response_id(const CANMessage &message, uint8_t node_id) const;
    // service callbacks
    void handle_start(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_exit(const std_srvs::srv::Trigger::Request::SharedPtr request,
                     std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_stop(const std_srvs::srv::Trigger::Request::SharedPtr request,
                     std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_recover(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_reset(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);
    // subscriber callback
    void target_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    // timer callback
    rclcpp::TimerBase::SharedPtr sync_timer_;
    // publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_velocity_pub_;
    // subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_velocity_sub_;
    // services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr exit_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    // parameters
    std::string type_;
    std::string can_interface_;
    std::string can_config_file_;
    // variables
    FSMState current_state_;
    std::vector<CANDevice> can_devices_;
    rclcpp::Time sync_start_time_;
    int sync_interval_ms_;
    int can_socket_;
    float current_velocity_ = 0.0;
    float target_velocity_  = 0.0;
    bool enable_sync_ = false;
};

#endif // ELMO_MASTER_NODE_HPP