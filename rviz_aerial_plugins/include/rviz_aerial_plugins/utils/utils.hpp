#ifndef RVIZ_AERIAL_PLUGINS_UTILS_HPP_
#define RVIZ_AERIAL_PLUGINS_UTILS_HPP_

#include <string>
#include <vector>
#include <sstream>
#include <set>

std::vector<std::string> split (const std::string &s, char delim);
std::set<std::string> get_namespaces(std::vector<std::string>& names_and_namespaces);
int get_target_system(std::string current_namespace);

#endif  // RVIZ_AERIAL_PLUGINS_UTILS_HPP_
