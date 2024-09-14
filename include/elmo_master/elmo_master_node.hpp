#ifndef ELMO_MASTER_NODE_HPP
#define ELMO_MASTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
// socketCAN
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
// debug
#include <cassert>

// FSM States
enum class FSMState {
    INIT,
    PREOP, // Pre-operational
    OP,    // Operational
    STOP,
    EXIT
};

class ElmoMasterNode : public rclcpp::Node {
public:
    ElmoMasterNode();
    ~ElmoMasterNode();
    void siginit_exit();
private:
    // methods
    void init_params();
    void transit_to_preop();
    void transit_to_op();
    void transit_to_stop();
    void transit_to_init();
    void transit_to_exit();
    bool send_can_message(uint32_t can_id, uint8_t can_dlc, const uint8_t data[8]); 
    // utils
    static std::string state_to_string(FSMState state);
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
    // variables
    FSMState current_state_;
    bool transition_error_flag_;
    float current_velocity_ = 0.0;
    float target_velocity_  = 0.0;
};

#endif // ELMO_MASTER_NODE_HPP