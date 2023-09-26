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

#include "asio_timer.cpp"
#include "interface.h"

namespace flight_brain {
namespace mountable {

// TODO(caofy): add conn detection
// TODO(caofy): auto detect conn and reconn
class ConnTcpClient : public ConnInterface,
                      public std::enable_shared_from_this<ConnTcpClient> {
 public:
  ConnTcpClient(std::string server_host = "127.0.0.1",
                unsigned short server_port = 8001);
  ~ConnTcpClient();

  void connect() override;
  void close() override;
  void run() override;

  void send_message(const std::string message) override;
  void send_bytes(const void *data, size_t len) override;

  void run_every(const uint64_t timeout_ms, TimerCallback cb) override;

  inline bool is_open() override { return socket_.is_open(); }

  void set_receive_callback(ReceiveCb cb) override {
    receive_cb_ = std::move(cb);
  }
  void set_conn_callback(ConnectionCb cb) override { conn_cb_ = std::move(cb); }
  void set_closed_callback(ClosedCb cb) override { closed_cb_ = std::move(cb); }

 private:
  void reconnect();
  void conn_handler();
  void read_handler(const boost::system::error_code &ec);
  void write_handler(const boost::system::error_code &ec);
  void do_receive();
  void do_send(bool check_tx_state = true);
  // callback
  ClosedCb closed_cb_;
  ReceiveCb receive_cb_;
  ConnectionCb conn_cb_;

  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::io_service::work> io_work_;
  std::thread io_thread_;

  enum { max_msg = 1024 };
  char read_buffer_[max_msg];
  char write_buffer_[max_msg];

  boost::asio::ip::tcp::socket socket_;
  boost::asio::ip::tcp::endpoint server_ep_;

  std::atomic<bool> is_destroying_;
  std::atomic<bool> tx_in_progress_;
  std::recursive_mutex mutex_;

  // std::vector<std::shared_ptr<AsioTimer>> timer_array_;
  std::vector<AsioTimer *> timer_array_;

  std::deque<std::string> write_msgs_;
};

}  // namespace mountable
}  // namespace flight_brain