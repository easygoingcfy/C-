/**
 * @file tcp.h
 * @author caofangyu (caofy@antwork.link)
 * @brief  tcp client for moutable device
 * @version 0.1
 * @date 2023-09-25
 *
 * Copyright (c) 2015-2022 Xunyi Ltd. All rights reserved.
 *
 */

#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <deque>

#include "interface.h"

namespace flight_brain {
namespace conn {

// TODO(caofy): add conn detection
// TODO(caofy): auto detect conn and reconn
class ConnTcpClient : public ConnInterface, public std::enable_shared_from_this<ConnTcpClient> {
   public:
    ConnTcpClient(std::string server_host = "127.0.0.1", int server_port = 8001);
    ~ConnTcpClient() override;

    void connect() override;
    void close() override;

    inline bool is_open() override { return socket_.is_open(); }

   private:
    void reconnect();
    void conn_handler(const boost::system::error_code &ec);
    void read_handler(const boost::system::error_code &ec);
    void write_handler(const boost::system::error_code &ec);

    void send_message(const std::string &message) override;
    void send_bytes(const void *data, size_t len) override;
    void do_receive() override;
    void do_send(bool check_tx_state = true) override;

    boost::asio::ip::tcp::socket socket_;
    boost::asio::ip::tcp::endpoint endpoint_;
};

}  // namespace conn
}  // namespace flight_brain