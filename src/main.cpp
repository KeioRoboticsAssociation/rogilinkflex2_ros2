#include <rclcpp/rclcpp.hpp>
#include "rogilinkflex2/communication_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rogilinkflex2::CommunicationNode>();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting RogiLinkFlex2 Communication Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}