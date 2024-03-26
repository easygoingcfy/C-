/**
 * @file udp.h
 * @author caofangyu (caofy@antwork.link)
 * @brief  基于Boost asio的异步UDP通信
 * @version 0.1
 * @date 2024-03-25
 *
 * Copyright (c) 2015-2024 Xunyi Ltd. All rights reserved.
 *
 */

#pragma once

#include <atomic>
#include <deque>

#include <boost/asio.hpp>

#include "interface.h"

namespace flight_brain {
namespace conn {

class ConnUdpClient : public ConnInterface, public std::enable_shared_from_this<ConnUdpClient> {
   public:
    ConnUdpClient(const std::string &ip, int port);
    ~ConnUdpClient() override;
    void connect() override {}
    void close() override;
    inline bool is_open() override { return socket_.is_open(); }
    void send_message(const std::string &message) override;
    void send_bytes(const void *data, size_t len) override;
    void do_send(bool check_tx_state) override;
    void do_receive() override;

   private:
    udp::socket socket_;
    udp::endpoint endpoint_;
};

}  // namespace conn
}  // namespace flight_brain