#include "rogilinkflex2/communication_node.hpp"
#include "rogilinkflex2/float_hex_converter.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <json/json.h>
#include <condition_variable>
#include <filesystem>
#include <algorithm>

namespace rogilinkflex2
{

CommunicationNode::CommunicationNode() 
    : Node("rogilinkflex2_communication_node"),
      serial_port_(""),
      serial_fd_(-1),
      running_(false),
      read_response_received_(false)
{
    // Declare parameters
    this->declare_parameter("serial_port", "auto");
    this->declare_parameter("config_file", "config/rogilinkflex2_config.yaml");

    std::string serial_param = this->get_parameter("serial_port").as_string();
    if (serial_param == "auto") {
        serial_port_ = findAvailableDevice();
        if (serial_port_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No suitable serial device found. Please specify serial_port parameter.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Auto-detected serial device: %s", serial_port_.c_str());
    } else {
        serial_port_ = serial_param;
    }
    std::string config_file = this->get_parameter("config_file").as_string();

    // Load configuration
    if (!loadConfiguration(config_file)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load configuration");
        return;
    }

    // Initialize serial communication
    initializeSerial();

    // Create subscribers
    read_sub_ = this->create_subscription<rogilinkflex2::msg::ReadRequest>(
        "/rogilinkflex2/read", 10,
        std::bind(&CommunicationNode::handleReadRequest, this, std::placeholders::_1));

    write_sub_ = this->create_subscription<rogilinkflex2::msg::WriteRequest>(
        "/rogilinkflex2/write", 10,
        std::bind(&CommunicationNode::handleWriteRequest, this, std::placeholders::_1));

    // Create publishers
    read_response_pub_ = this->create_publisher<rogilinkflex2::msg::DeviceData>(
        "/rogilinkflex2/read_response", 10);

    // Create periodic publishers
    for (const auto& request : periodic_requests_) {
        periodic_publishers_[request.topic_name] = 
            this->create_publisher<rogilinkflex2::msg::PeriodicData>(request.topic_name, 10);
    }

    // Send configuration to HAL
    sendConfigurationToHAL();

    RCLCPP_INFO(this->get_logger(), "RogiLinkFlex2 Communication Node initialized");
}

CommunicationNode::~CommunicationNode()
{
    running_ = false;
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
    if (serial_fd_ >= 0) {
        close(serial_fd_);
    }
}

bool CommunicationNode::loadConfiguration(const std::string& config_file)
{
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        
        if (!loadDeviceMapping(config)) {
            return false;
        }
        
        if (!loadPeriodicRequests(config)) {
            return false;
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
        return false;
    }
}

bool CommunicationNode::loadDeviceMapping(const YAML::Node& config)
{
    if (!config["device_mapping"]) {
        RCLCPP_ERROR(this->get_logger(), "No device_mapping found in config");
        return false;
    }

    for (const auto& mapping : config["device_mapping"]) {
        std::string device_name = mapping.first.as<std::string>();
        uint8_t device_id = mapping.second.as<uint8_t>();
        
        device_mapping_[device_name] = device_id;
        reverse_device_mapping_[device_id] = device_name;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu device mappings", device_mapping_.size());
    return true;
}

bool CommunicationNode::loadPeriodicRequests(const YAML::Node& config)
{
    if (!config["periodic_requests"]) {
        RCLCPP_WARN(this->get_logger(), "No periodic_requests found in config");
        return true;
    }

    for (const auto& request_node : config["periodic_requests"]) {
        PeriodicRequest request;
        request.request = request_node["request"].as<std::string>();
        request.interval_ms = request_node["interval_ms"].as<int>();
        request.topic_name = request_node["topic_name"].as<std::string>();

        // Load instance IDs
        for (const auto& id : request_node["instance_ids"]) {
            request.instance_ids.push_back(id.as<uint8_t>());
        }

        // Load data names and types
        for (const auto& name : request_node["data_names"]) {
            request.data_names.push_back(name.as<std::string>());
        }
        for (const auto& type : request_node["data_types"]) {
            request.data_types.push_back(type.as<std::string>());
        }

        periodic_requests_.push_back(request);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu periodic requests", periodic_requests_.size());
    return true;
}

void CommunicationNode::initializeSerial()
{
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
        return;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting terminal attributes");
        close(serial_fd_);
        serial_fd_ = -1;
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting terminal attributes");
        close(serial_fd_);
        serial_fd_ = -1;
        return;
    }

    running_ = true;
    serial_thread_ = std::thread(&CommunicationNode::serialReceiveThread, this);

    RCLCPP_INFO(this->get_logger(), "Serial communication initialized on %s", serial_port_.c_str());
}

void CommunicationNode::serialReceiveThread()
{
    char buffer[8192]; // Further increased buffer size to 8192
    std::string receive_buffer;
    auto last_data_time = std::chrono::steady_clock::now();

    while (running_) {
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            receive_buffer += buffer;
            last_data_time = std::chrono::steady_clock::now();

            // Process complete messages (assuming newline delimiter)
            size_t pos = 0;
            while ((pos = receive_buffer.find('\n')) != std::string::npos) {
                std::string message = receive_buffer.substr(0, pos);
                receive_buffer.erase(0, pos + 1);
                
                if (!message.empty()) {
                    processIncomingData(message);
                }
            }
            
            // Handle timeout for incomplete messages (wait for more data)
            auto current_time = std::chrono::steady_clock::now();
            auto time_since_last_data = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_data_time).count();
            
            // If we have data waiting and no new data for 100ms, consider it incomplete
            if (!receive_buffer.empty() && time_since_last_data > 100) {
                // Check if it looks like a truncated JSON message
                if (receive_buffer.length() > 10 && 
                    (receive_buffer.find("{") == 0 || receive_buffer.find("[") == 0)) {
                    RCLCPP_WARN(this->get_logger(), 
                        "Incomplete message detected after timeout, discarding: '%s'", 
                        receive_buffer.c_str());
                }
                receive_buffer.clear();
            }
            
            // Prevent buffer from growing too large with incomplete messages
            if (receive_buffer.length() > 8192) { // Increased limit
                RCLCPP_WARN(this->get_logger(), 
                    "Receive buffer overflow (%zu bytes), clearing incomplete message", 
                    receive_buffer.length());
                receive_buffer.clear();
            }
        } else if (bytes_read == 0) {
            // No data available, but check for message timeouts
            auto current_time = std::chrono::steady_clock::now();
            auto time_since_last_data = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_data_time).count();
            
            if (!receive_buffer.empty() && time_since_last_data > 200) {
                RCLCPP_WARN(this->get_logger(), 
                    "Message timeout, discarding incomplete data: '%s'", 
                    receive_buffer.c_str());
                receive_buffer.clear();
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void CommunicationNode::sendToSerial(const std::string& message)
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ >= 0) {
        std::string msg_with_newline = message + "\n";
        write(serial_fd_, msg_with_newline.c_str(), msg_with_newline.length());
    }
}

void CommunicationNode::sendConfigurationToHAL()
{
    Json::Value config_json;
    config_json["type"] = "configuration";
    
    Json::Value periodic_array(Json::arrayValue);
    for (const auto& request : periodic_requests_) {
        Json::Value request_json;
        request_json["request"] = request.request;
        request_json["interval_ms"] = request.interval_ms;
        
        Json::Value instance_ids(Json::arrayValue);
        for (uint8_t id : request.instance_ids) {
            instance_ids.append(static_cast<int>(id));
        }
        request_json["instance_ids"] = instance_ids;
        
        Json::Value data_names(Json::arrayValue);
        for (const auto& name : request.data_names) {
            data_names.append(name);
        }
        request_json["data_names"] = data_names;
        
        periodic_array.append(request_json);
    }
    config_json["periodic_requests"] = periodic_array;

    Json::StreamWriterBuilder builder;
    std::string json_string = Json::writeString(builder, config_json);
    
    sendToSerial(json_string);
    RCLCPP_INFO(this->get_logger(), "Configuration sent to HAL");
}

void CommunicationNode::handleReadRequest(const rogilinkflex2::msg::ReadRequest::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(read_mutex_);
    
    Json::Value request_json;
    request_json["type"] = "read";
    request_json["id"] = static_cast<int>(msg->id);
    request_json["data_type"] = msg->data_type;

    Json::StreamWriterBuilder builder;
    std::string json_string = Json::writeString(builder, request_json);
    
    read_response_received_ = false;
    sendToSerial(json_string);
    
    // Wait for response with timeout
    if (read_cv_.wait_for(lock, std::chrono::seconds(5), [this] { return read_response_received_; })) {
        read_response_pub_->publish(latest_read_response_);
    } else {
        RCLCPP_WARN(this->get_logger(), "Read request timeout for device ID %d", msg->id);
        
        rogilinkflex2::msg::DeviceData timeout_response;
        timeout_response.id = msg->id;
        timeout_response.data_type = msg->data_type;
        timeout_response.value = 0.0;
        timeout_response.status = "timeout";
        read_response_pub_->publish(timeout_response);
    }
}

void CommunicationNode::handleWriteRequest(const rogilinkflex2::msg::WriteRequest::SharedPtr msg)
{
    Json::Value request_json;
    request_json["type"] = "write";
    request_json["id"] = static_cast<int>(msg->id);
    request_json["data_type"] = msg->data_type;
    request_json["value"] = msg->value;

    Json::StreamWriterBuilder builder;
    std::string json_string = Json::writeString(builder, request_json);
    
    sendToSerial(json_string);
    RCLCPP_INFO(this->get_logger(), "Write request sent: ID=%d, type=%s, value=%f", 
                msg->id, msg->data_type.c_str(), msg->value);
}

void CommunicationNode::processIncomingData(const std::string& data)
{
    try {
        // Skip empty or whitespace-only data
        if (data.empty() || data.find_first_not_of(" \t\r\n") == std::string::npos) {
            return;
        }
        
        // Remove any leading/trailing whitespace
        std::string trimmed_data = data;
        trimmed_data.erase(0, trimmed_data.find_first_not_of(" \t\r\n"));
        trimmed_data.erase(trimmed_data.find_last_not_of(" \t\r\n") + 1);
        
        Json::CharReaderBuilder builder;
        Json::CharReader* reader = builder.newCharReader();
        Json::Value json_data;
        std::string errors;

        if (!reader->parse(trimmed_data.c_str(), trimmed_data.c_str() + trimmed_data.length(), &json_data, &errors)) {
            // Check if this looks like a truncated JSON message
            if (trimmed_data.length() > 10 && 
                (trimmed_data.find("{") == 0 || trimmed_data.find("[") == 0) &&
                (trimmed_data.find("}") == std::string::npos || trimmed_data.find("]") == std::string::npos)) {
                RCLCPP_WARN(this->get_logger(), "Received incomplete JSON message (likely truncated): '%s'", trimmed_data.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to parse JSON: %s\nReceived data: '%s'", errors.c_str(), trimmed_data.c_str());
            }
            delete reader;
            return;
        }
        delete reader;

        std::string type = json_data["type"].asString();

        if (type == "read_response") {
            std::lock_guard<std::mutex> lock(read_mutex_);
            
            latest_read_response_.id = static_cast<uint8_t>(json_data["id"].asInt());
            latest_read_response_.data_type = json_data["data_type"].asString();
            
            // Handle hex float values
            if (json_data["value"].isString()) {
                std::string value_str = json_data["value"].asString();
                if (isHexFloatString(value_str)) {
                    latest_read_response_.value = hexStringToFloat(value_str);
                } else {
                    latest_read_response_.value = std::stod(value_str);
                }
            } else {
                latest_read_response_.value = json_data["value"].asDouble();
            }
            
            latest_read_response_.status = json_data["status"].asString();
            
            read_response_received_ = true;
            read_cv_.notify_one();
            
        } else if (type == "periodic_data") {
            std::string request = json_data["request"].asString();
            
            rogilinkflex2::msg::PeriodicData periodic_msg;
            periodic_msg.request = request;
            
            for (const auto& data_item : json_data["data"]) {
                uint8_t device_id = static_cast<uint8_t>(data_item["id"].asInt());
                
                // Handle multiple data fields per device (new format with angle, speed, etc.)
                for (auto it = data_item.begin(); it != data_item.end(); ++it) {
                    std::string key = it.key().asString();
                    if (key != "id") { // Skip the ID field
                        rogilinkflex2::msg::DeviceData device_data;
                        device_data.id = device_id;
                        device_data.data_type = key; // angle, speed, etc.
                        
                        // Handle hex float values
                        if (it->isString()) {
                            std::string value_str = it->asString();
                            if (isHexFloatString(value_str)) {
                                device_data.value = hexStringToFloat(value_str);
                            } else {
                                device_data.value = std::stod(value_str);
                            }
                        } else {
                            device_data.value = it->asDouble();
                        }
                        
                        device_data.status = "success";
                        
                        periodic_msg.data.push_back(device_data);
                    }
                }
            }
            
            // Find corresponding publisher and publish
            for (const auto& req : periodic_requests_) {
                if (req.request == request) {
                    if (periodic_publishers_.find(req.topic_name) != periodic_publishers_.end()) {
                        periodic_publishers_[req.topic_name]->publish(periodic_msg);
                    }
                    break;
                }
            }
        } else if (type == "debug") {
            std::string message = json_data["message"].asString();
            RCLCPP_INFO(this->get_logger(), "[HAL DEBUG] %s", message.c_str());
        } else if (type == "error") {
            std::string message = json_data["message"].asString();
            RCLCPP_ERROR(this->get_logger(), "[HAL ERROR] %s", message.c_str());
        } else if (type == "configuration_ack") {
            int periodic_requests_count = json_data.get("periodic_requests_count", 0).asInt();
            std::string status = json_data.get("status", "unknown").asString();
            RCLCPP_INFO(this->get_logger(), "HAL configuration acknowledged - %d periodic requests, status: %s", 
                       periodic_requests_count, status.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown message type received: %s", type.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing incoming data: %s", e.what());
    }
}

uint8_t CommunicationNode::getDeviceId(const std::string& device_name)
{
    auto it = device_mapping_.find(device_name);
    if (it != device_mapping_.end()) {
        return it->second;
    }
    return 255; // Invalid ID
}

std::string CommunicationNode::getDeviceName(uint8_t device_id)
{
    auto it = reverse_device_mapping_.find(device_id);
    if (it != reverse_device_mapping_.end()) {
        return it->second;
    }
    return "unknown";
}

std::vector<std::string> CommunicationNode::getSerialDeviceList()
{
    std::vector<std::string> device_list;
    
    try {
        for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
            std::string path = entry.path().string();
            if (path.find("/dev/ttyACM") != std::string::npos || 
                path.find("/dev/ttyUSB") != std::string::npos) {
                device_list.push_back(path);
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Error scanning /dev directory: %s", e.what());
    }
    
    // Sort the device list for consistent ordering
    std::sort(device_list.begin(), device_list.end());
    return device_list;
}

std::string CommunicationNode::findAvailableDevice()
{
    std::vector<std::string> devices = getSerialDeviceList();
    
    if (devices.empty()) {
        RCLCPP_WARN(this->get_logger(), "No serial devices found in /dev");
        return "";
    }
    
    // Try to open each device to check if it's available
    for (const auto& device : devices) {
        int test_fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (test_fd >= 0) {
            close(test_fd);
            RCLCPP_INFO(this->get_logger(), "Found available device: %s", device.c_str());
            return device;
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "No available serial devices found. Found devices: ");
    for (const auto& device : devices) {
        RCLCPP_WARN(this->get_logger(), "  %s (not available)", device.c_str());
    }
    
    return "";
}

} // namespace rogilinkflex2