#pragma once

#include <cctype>
#include <cstdlib>
#include <string>

#include <rclcpp/logging.hpp>

namespace teb_local_planner
{
namespace detail
{

inline const char * useCurbOrWallEnv()
{
  static const char * const value = std::getenv("USE_CURB_OR_WALL");
  return value;
}

inline std::string normalizeEdgeModeEnv(const char* environment_string)
{
  
  if (environment_string == nullptr) {
    RCLCPP_WARN(rclcpp::get_logger("controller_server"), "USE_CURB_OR_WALL 环境变量为空");
    return "";
  }
  std::string normalized_characters;
  for (const char* character_pointer = environment_string; *character_pointer != '\0';
       ++character_pointer) {
    if (*character_pointer != ' ' && *character_pointer != '\t') {
      normalized_characters.push_back(static_cast<char>(
        std::tolower(static_cast<unsigned char>(*character_pointer))));
    }
  }
  return normalized_characters;
}

}  // namespace detail
}  // namespace teb_local_planner
