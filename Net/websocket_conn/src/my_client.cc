#include <iostream>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "client.h"



void clientThread() {
  std::string uri = "ws://121.199.30.205:9002";
  WebsocketClient client;
  std::cout << "client run" << std::endl;
  client.run(uri);
  std::cout << "test block" << std::endl;

}

int main() {
  std::thread ct(clientThread);
  while (true) {};
  //TODO(caofangyu): 服务器需要管理连接池，暂定用ip地址对应connection_hdl
}