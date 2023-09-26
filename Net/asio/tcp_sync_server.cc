#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <chrono>
#include <ctime>
#include <iostream>
#include <sstream>
#include <thread>

using namespace boost::asio;
using boost::system::error_code;
io_service service;

size_t read_complete(char* buff, const error_code& err, size_t bytes) {
  if (err) {
    std::cout << "read error: " << err.message() << std::endl;
    return 0;
  }
  bool found = std::find(buff, buff + bytes, '\n') < buff + bytes;
  // 我们一个一个读取直到读到回车，不缓存
  return found ? 0 : 1;
}

void handle_connections() {
  ip::tcp::acceptor acceptor(service, ip::tcp::endpoint(ip::tcp::v4(), 8001));
  char buff[1024];
  ip::tcp::socket sock(service);
  acceptor.accept(sock);
  std::cout << "accept, read to read" << std::endl;
  time_t cur;
  struct tm* local_time;
  while (true) {
    std::cout << "start read" << std::endl;
    int bytes =
        read(sock, buffer(buff), boost::bind(read_complete, buff, _1, _2));
    std::string msg(buff, bytes);
    std::cout << "read msg: " << msg << std::endl;
    sock.write_some(buffer(msg));
  }
}
int main(int argc, char* argv[]) { handle_connections(); }