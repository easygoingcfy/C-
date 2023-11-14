#pragma once

#include <regex>
#include <string>
#include <iostream>

namespace websocket_conn {

std::string get_ip_from_str(std::string input) { std::regex ip_regex(R"(\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b)"); 
  std::smatch matches;
  if (!std::regex_search(input, matches, ip_regex)) {
    std::cout << "no ip found" << std::endl;
    return "";
  }
  return matches[0];
}


}  // namespace webbsocket_conn