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

std::set<std::string> get_namespaces(std::vector<std::string>& names_and_namespaces)
{
  std::set<std::string> namespaces;
  for(auto topic_name: names_and_namespaces){
    std::vector<std::string> topic_name_tokens = split (topic_name, '/');
    if(topic_name_tokens.size() > 1){
      std::string namespace_topic = topic_name_tokens[0];
      if (namespace_topic.find("plane")!=std::string::npos ||
          namespace_topic.find("iris")!=std::string::npos){
            namespaces.insert(namespace_topic);
      }
    }
  }
  return namespaces;
}

int get_target_system(std::string current_namespace)
{
  int result = -1;

  std::vector<std::string> namespace_tokens = split (current_namespace, '_');
  if(namespace_tokens.size() > 1){
    result = atoi(namespace_tokens[1].c_str());
  }
  return result + 1;
}
