#include <iostream>
#include <string>
#include <thread>

#include "client.h"
#include "server.h"

bool condition = false;

void serverThread() {
  WebsocketServer server;
  server.run(9002);
}

void clientThread() {
  std::string uri = "ws://localhost:9002";
  WebsocketClient client;
  std::cout << "client run" << std::endl;
  client.run(uri);
  std::cout << "test block" << std::endl;
}

int main() {
  std::thread st(serverThread);
  st.detach();
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::thread ct(clientThread);
  std::condition_variable cv;
  std::mutex mtx;
  std::unique_lock<std::mutex> lock(mtx);
  cv.wait(lock, [] { return condition; });
  std::cout << "process end" << std::endl;
  // TODO(caofangyu): 服务器需要管理连接池，暂定用ip地址对应connection_hdl
}