#include <iostream>
#include <string>
#include <thread>

#include "server.h"

void serverThread() {
  WebsocketServer server;
  server.run(9002);
}

int main() {
  std::thread st(serverThread);
  st.join();
}