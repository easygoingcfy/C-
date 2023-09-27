#include "pinling_protocol.cc"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <iostream>

using namespace boost::asio;
using boost::system::error_code;

int main(int argc, char* argv[]) {
    io_service service;
    ip::tcp::endpoint ep(ip::address::from_string("192.168.2.119"), 2000);
    CombinationControlA cmd = get_cmd();
    error_code ec;
    ip::tcp::socket sock(service);
    sock.connect(ep, ec);
    if (ec) std::cout << ec.message() << std::endl;
    sock.write_some(buffer((const void*)&cmd, sizeof(cmd)));
    char buf[1024];
    int bytes = read(sock, buffer(buf));
    std::cout << "read msg:" << *buf << std::endl;
    boost::this_thread::sleep(boost::posix_time::millisec(100));

    sock.close();
    return 0;
}