#pragma once

#include <string>
#include <cstdint>

namespace rogilinkflex2 {

/**
 * @brief Convert a float to a signed hexadecimal string representation
 * @param value The float value to convert
 * @return Hexadecimal string (e.g., "0x12345678" or "-0x12345678")
 */
std::string floatToHexString(float value);

/**
 * @brief Convert a signed hexadecimal string back to a float
 * @param hex_str Hexadecimal string (e.g., "0x12345678", "-0x12345678")
 * @return The float value
 */
float hexStringToFloat(const std::string& hex_str);

/**
 * @brief Check if a string represents a hexadecimal float value
 * @param str The string to check
 * @return true if the string is a hex float representation
 */
bool isHexFloatString(const std::string& str);

} // namespace rogilinkflex2