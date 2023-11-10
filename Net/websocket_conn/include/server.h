/**
 * @file server.h
 * @author caofangyu (caofy@antwork.link)
 * @brief  websocket服务端，使用websocketpp库实现
 * @version 0.1
 * @date 2023-11-07
 *
 * Copyright (c) 2015-2022 Xunyi Ltd. All rights reserved.
 *
 */

#pragma once

#include <functional>
#include <iostream>

#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

typedef websocketpp::server<websocketpp::config::asio> server;

class WebsocketServer {
 public:
  WebsocketServer();

  void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg);

  void run(uint16_t port);

 private:
  server m_server;
};