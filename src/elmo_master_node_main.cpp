#include "elmo_master/elmo_master_node.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto nd = std::make_shared<ElmoMasterNode>();
    rclcpp::spin(nd);
    rclcpp::shutdown();
    return 0;
}