#include "rviz_aerial_plugins/utils/utils.hpp"

std::vector<std::string> split (const std::string &s, char delim)
{
  std::vector<std::string> result;
  std::stringstream ss (s);
  std::string item;

  while (getline (ss, item, delim)) {
    if(!item.empty())
      result.push_back (item);
  }

  return result;
}
