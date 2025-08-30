#ifndef ROGILINKFLEX2_COMMUNICATION_NODE_HPP
#define ROGILINKFLEX2_COMMUNICATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>

#include "rogilinkflex2/msg/device_data.hpp"
#include "rogilinkflex2/msg/read_request.hpp"
#include "rogilinkflex2/msg/write_request.hpp"
#include "rogilinkflex2/msg/periodic_data.hpp"

namespace rogilinkflex2
{

struct PeriodicRequest 
{
    std::string request;
    std::vector<uint8_t> instance_ids;
    std::vector<std::string> data_names;
    std::vector<std::string> data_types;
    int interval_ms;
    std::string topic_name;
};

class CommunicationNode : public rclcpp::Node
{
public:
    CommunicationNode();
    ~CommunicationNode();

private:
    // Configuration management
    bool loadConfiguration(const std::string& config_file);
    bool loadDeviceMapping(const YAML::Node& config);
    bool loadPeriodicRequests(const YAML::Node& config);
    void sendConfigurationToHAL();

    // Serial communication
    void initializeSerial();
    void serialReceiveThread();
    void sendToSerial(const std::string& message);
    std::string receiveFromSerial();

    // Message handling
    void handleReadRequest(const rogilinkflex2::msg::ReadRequest::SharedPtr msg);
    void handleWriteRequest(const rogilinkflex2::msg::WriteRequest::SharedPtr msg);
    void processIncomingData(const std::string& data);
    
    // Utility functions
    uint8_t getDeviceId(const std::string& device_name);
    std::string getDeviceName(uint8_t device_id);
    std::vector<std::string> getSerialDeviceList();
    std::string findAvailableDevice();

    // Publishers and Subscribers
    rclcpp::Subscription<rogilinkflex2::msg::ReadRequest>::SharedPtr read_sub_;
    rclcpp::Subscription<rogilinkflex2::msg::WriteRequest>::SharedPtr write_sub_;
    rclcpp::Publisher<rogilinkflex2::msg::DeviceData>::SharedPtr read_response_pub_;
    
    std::map<std::string, rclcpp::Publisher<rogilinkflex2::msg::PeriodicData>::SharedPtr> periodic_publishers_;

    // Configuration data
    std::map<std::string, uint8_t> device_mapping_;
    std::map<uint8_t, std::string> reverse_device_mapping_;
    std::vector<PeriodicRequest> periodic_requests_;

    // Serial communication
    std::string serial_port_;
    int serial_fd_;
    std::thread serial_thread_;
    std::atomic<bool> running_;
    std::mutex serial_mutex_;

    // Synchronous read handling
    std::mutex read_mutex_;
    std::condition_variable read_cv_;
    bool read_response_received_;
    rogilinkflex2::msg::DeviceData latest_read_response_;
};

} // namespace rogilinkflex2

#endif // ROGILINKFLEX2_COMMUNICATION_NODE_HPP