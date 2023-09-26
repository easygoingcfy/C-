#include "tcp.h"

#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>

using namespace flight_brain::mountable;
using namespace std;


void on_receive(char* data, size_t len) {
    std::string msg(data, len);
    cout << "receive: " << msg << endl;
}


void test() {
    std::shared_ptr<ConnTcpClient> client = std::make_shared<ConnTcpClient>();
    client->set_receive_callback(&on_receive);
    client->connect();
    client->run();
    //std::thread io_thread(&ConnTcpClient::run, client);
    //io_thread.detach();
    stringstream ss;
    while (true) {
        auto now = std::chrono::system_clock::now();
        std::time_t end_time = std::chrono::system_clock::to_time_t(now);
        ss << "time: " << std::ctime(&end_time);
        client->send_message(ss.str());
        cout << "send msg: " << ss.str();
        ss.clear();
        ss.str("");
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

int main() {
    test();
    return 0;
}