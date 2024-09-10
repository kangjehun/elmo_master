#include "elmo_master/elmo_master_node.hpp"

#include <csignal>

std::shared_ptr<ElmoMasterNode> node = nullptr;
// Signal handler
void signal_handler(int signal) 
{
    if (signal == SIGINT) 
    {
        node->siginit_exit();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    node = std::make_shared<ElmoMasterNode>();
    rclcpp::spin(node);
    return 0;
}