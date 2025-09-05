#include "rogilinkflex2/float_hex_converter.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>
#include <algorithm>

namespace rogilinkflex2 {

std::string floatToHexString(float value) {
    // Use union to access float bits directly
    union {
        float f;
        uint32_t u;
    } converter;
    
    converter.f = value;
    
    std::stringstream ss;
    
    // Convert to signed hex string (format: 0x12345678 or -0x12345678)
    if (converter.u & 0x80000000) {
        // Negative number - flip all bits and add 1 (two's complement), then add minus sign
        uint32_t positive_bits = (~converter.u) + 1;
        ss << "-0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << positive_bits;
    } else {
        // Positive number
        ss << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << converter.u;
    }
    
    return ss.str();
}

float hexStringToFloat(const std::string& hex_str) {
    if (hex_str.empty()) {
        return 0.0f;
    }
    
    bool is_negative = false;
    std::string hex_part = hex_str;
    
    // Check for negative sign
    if (hex_part[0] == '-') {
        is_negative = true;
        hex_part = hex_part.substr(1);
    }
    
    // Skip "0x" prefix if present
    if (hex_part.length() >= 2 && 
        hex_part.substr(0, 2) == "0x" || hex_part.substr(0, 2) == "0X") {
        hex_part = hex_part.substr(2);
    }
    
    // Convert hex string to uint32_t
    uint32_t hex_value = 0;
    try {
        hex_value = static_cast<uint32_t>(std::stoul(hex_part, nullptr, 16));
    } catch (const std::exception&) {
        return 0.0f; // Return 0 for invalid hex strings
    }
    
    // Handle two's complement for negative numbers
    if (is_negative) {
        hex_value = (~hex_value) + 1;
        hex_value |= 0x80000000; // Set sign bit
    }
    
    // Convert back to float using union
    union {
        float f;
        uint32_t u;
    } converter;
    
    converter.u = hex_value;
    return converter.f;
}

bool isHexFloatString(const std::string& str) {
    if (str.empty()) {
        return false;
    }
    
    std::string test_str = str;
    
    // Check for optional negative sign
    if (test_str[0] == '-') {
        test_str = test_str.substr(1);
    }
    
    // Must start with "0x" or "0X"
    if (test_str.length() < 3 || 
        (test_str.substr(0, 2) != "0x" && test_str.substr(0, 2) != "0X")) {
        return false;
    }
    
    // Check that remaining characters are valid hex digits
    std::string hex_digits = test_str.substr(2);
    if (hex_digits.empty()) {
        return false;
    }
    
    return std::all_of(hex_digits.begin(), hex_digits.end(), 
                      [](char c) { 
                          return std::isxdigit(static_cast<unsigned char>(c)); 
                      });
}

} // namespace rogilinkflex2