#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "rogilinkflex2/msg/read_request.hpp"
#include "rogilinkflex2/msg/write_request.hpp" 
#include "rogilinkflex2/msg/device_data.hpp"
#include "rogilinkflex2/msg/periodic_data.hpp"

class RogiLinkFlex2TestNode : public rclcpp::Node
{
public:
    RogiLinkFlex2TestNode() : Node("rogilinkflex2_test_node")
    {
        read_pub_ = this->create_publisher<rogilinkflex2::msg::ReadRequest>("/rogilinkflex2/read", 10);
        write_pub_ = this->create_publisher<rogilinkflex2::msg::WriteRequest>("/rogilinkflex2/write", 10);
        
        read_response_sub_ = this->create_subscription<rogilinkflex2::msg::DeviceData>(
            "/rogilinkflex2/read_response", 10,
            std::bind(&RogiLinkFlex2TestNode::readResponseCallback, this, std::placeholders::_1));
            
        periodic_sub_ = this->create_subscription<rogilinkflex2::msg::PeriodicData>(
            "/motor/angle_speed", 10,
            std::bind(&RogiLinkFlex2TestNode::periodicDataCallback, this, std::placeholders::_1));

        test_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&RogiLinkFlex2TestNode::runTests, this));

        test_counter_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "RogiLinkFlex2 Test Node initialized - testing communication without peripherals");
    }

private:
    void runTests()
    {
        test_counter_++;
        
        switch(test_counter_) {
            case 1:
                testReadRequest();
                break;
            case 2:
                testWriteRequest();
                break;
            case 3:
                testMultipleReadRequests();
                break;
            case 4:
                testMultipleWriteRequests();
                break;
            default:
                test_counter_ = 0;
                break;
        }
    }

    void testReadRequest()
    {
        RCLCPP_INFO(this->get_logger(), "=== Test 1: Single Read Request ===");
        
        auto msg = rogilinkflex2::msg::ReadRequest();
        msg.id = 0;
        msg.data_type = "angle";
        
        read_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent read request: ID=%d, type=%s", msg.id, msg.data_type.c_str());
    }

    void testWriteRequest()
    {
        RCLCPP_INFO(this->get_logger(), "=== Test 2: Single Write Request ===");
        
        auto msg = rogilinkflex2::msg::WriteRequest();
        msg.id = 1;
        msg.data_type = "target_angle";
        msg.value = 90.0;
        
        write_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent write request: ID=%d, type=%s, value=%f", 
                   msg.id, msg.data_type.c_str(), msg.value);
    }

    void testMultipleReadRequests()
    {
        RCLCPP_INFO(this->get_logger(), "=== Test 3: Multiple Read Requests ===");
        
        std::vector<std::pair<uint8_t, std::string>> read_tests = {
            {0, "angle"},
            {0, "speed"},
            {1, "angle"},
            {2, "temperature"}
        };

        for (const auto& test : read_tests) {
            auto msg = rogilinkflex2::msg::ReadRequest();
            msg.id = test.first;
            msg.data_type = test.second;
            
            read_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Sent read request: ID=%d, type=%s", 
                       msg.id, msg.data_type.c_str());
            
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    void testMultipleWriteRequests()
    {
        RCLCPP_INFO(this->get_logger(), "=== Test 4: Multiple Write Requests ===");
        
        std::vector<std::tuple<uint8_t, std::string, float>> write_tests = {
            {0, "target_angle", 45.0},
            {1, "target_speed", 100.0},
            {2, "enable", 1.0},
            {0, "target_angle", -30.0}
        };

        for (const auto& test : write_tests) {
            auto msg = rogilinkflex2::msg::WriteRequest();
            msg.id = std::get<0>(test);
            msg.data_type = std::get<1>(test);
            msg.value = std::get<2>(test);
            
            write_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Sent write request: ID=%d, type=%s, value=%f", 
                       msg.id, msg.data_type.c_str(), msg.value);
            
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    void readResponseCallback(const rogilinkflex2::msg::DeviceData::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received read response: ID=%d, type=%s, value=%f, status=%s",
                   msg->id, msg->data_type.c_str(), msg->value, msg->status.c_str());
    }

    void periodicDataCallback(const rogilinkflex2::msg::PeriodicData::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received periodic data for request: %s", msg->request.c_str());
        
        for (const auto& data : msg->data) {
            RCLCPP_INFO(this->get_logger(), "  Device ID=%d, value=%f, status=%s",
                       data.id, data.value, data.status.c_str());
        }
    }

    rclcpp::Publisher<rogilinkflex2::msg::ReadRequest>::SharedPtr read_pub_;
    rclcpp::Publisher<rogilinkflex2::msg::WriteRequest>::SharedPtr write_pub_;
    rclcpp::Subscription<rogilinkflex2::msg::DeviceData>::SharedPtr read_response_sub_;
    rclcpp::Subscription<rogilinkflex2::msg::PeriodicData>::SharedPtr periodic_sub_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    int test_counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RogiLinkFlex2TestNode>());
    rclcpp::shutdown();
    return 0;
}