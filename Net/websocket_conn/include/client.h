/**
 * @file client.h
 * @author caofangyu (caofy@antwork.link)
 * @brief  websocket客户端代码，使用websocketpp库实现
 * @version 0.1
 * @date 2023-11-07
 *
 * Copyright (c) 2015-2022 Xunyi Ltd. All rights reserved.
 *
 */

#pragma once

#include "websocketpp/client.hpp"
#include "websocketpp/config/asio_no_tls.hpp"

#include <functional>
#include <iostream>

typedef websocketpp::client<websocketpp::config::asio> client;
typedef websocketpp::config::asio::message_type::ptr message_ptr;

class WebsocketClient {
 public:
  WebsocketClient();

  void connect(const std::string& server);
  void send(websocketpp::connection_hdl hdl, const std::string& message);
  void run(const std::string& server);

 private:
  void on_message(websocketpp::connection_hdl hdl, message_ptr msg);
  void on_open(websocketpp::connection_hdl hdl);
  void on_close(websocketpp::connection_hdl hdl);

  client m_client;
  websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
};