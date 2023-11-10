#include <stdio.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <thread>

using namespace boost::asio;
using boost::system::error_code;
io_service service;

bool read_flag = false;

size_t read_complete(char* buff, const error_code& err, size_t bytes) {
  if (err) {
    std::cout << "read error: " << err.message() << std::endl;
    return 0;
  }
  bool found = std::find(buff, buff + bytes, '\n') < buff + bytes;
  // 我们一个一个读取直到读到回车，不缓存
  // return found ? 0 : 1;
  printf("bytes: %d \n", bytes);
  return bytes > 0 ? 0 : 1;
}

void handle_connections() {
  printf("start\n");
  ip::tcp::acceptor acceptor(service, ip::tcp::endpoint(ip::tcp::v4(), 8001));
  char buff[1024];
  time_t cur;
  struct tm* local_time;
  while (true) {
    ip::tcp::socket sock(service);
    acceptor.accept(sock);
    // std::cout << "accept, read to read" << std::endl;
    std::cout << "start read" << std::endl;
    int bytes =
        read(sock, buffer(buff), boost::bind(read_complete, buff, _1, _2));
    std::string msg(buff, bytes);
    char byte = buff[0];
    std::cout << "read msg: **" << int(byte) << "**" << std::endl;
    sock.write_some(buffer(msg));
    sock.close();
  }
}

int main(int argc, char* argv[]) { handle_connections(); }