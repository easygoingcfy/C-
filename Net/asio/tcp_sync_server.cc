#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>
#include <sstream>

using namespace boost::asio;
using boost::system::error_code;
io_service service;

size_t read_complete(char * buff, const error_code & err, size_t bytes) {
    if ( err) return 0;
    bool found = std::find(buff, buff + bytes, '\n') < buff + bytes;
    // 我们一个一个读取直到读到回车，不缓存
    return found ? 0 : 1;
}
void handle_connections() {
    ip::tcp::acceptor acceptor(service, ip::tcp::endpoint(ip::tcp::v4(),8001));
    char buff[1024];
    ip::tcp::socket sock(service);
    acceptor.accept(sock);
    std::cout << "accept, read to read" << std::endl;
    //time print
    time_t cur;
    struct tm *local_time;
    while ( true) {
        std::cout << "start read" << std::endl;
        int bytes = read(sock, buffer(buff), boost::bind(read_complete,buff,_1,_2));
        std::string msg(buff, bytes);
        std::cout << "read msg: " << msg;
        sock.write_some(buffer(msg));
        //for (int i = 0; i < 1000; ++i) {
        //    //time
        //    std::stringstream ss;
        //    time(&cur);
        //    local_time = localtime(&cur);
        //    ss << local_time->tm_hour << ":" 
        //       << local_time->tm_min  << ":" 
        //       << local_time->tm_sec;
        //    sock.write_some(buffer(ss.str()));
        //    std::this_thread::sleep_for(std::chrono::seconds(1));
        //}
        //sock.close();
    }
}
int main(int argc, char* argv[]) {
    handle_connections();
}