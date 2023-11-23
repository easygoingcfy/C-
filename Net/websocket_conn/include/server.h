/**
 * @file server.h
 * @author caofangyu (caofy@antwork.link)
 * @brief websocket服务端，使用websocketpp库实现
 * @warning 服务端与客户端的通信基于ip, 一个ip地址只能持有一个连接
 * @version 0.1
 * @date 2023-11-07
 *
 * Copyright (c) 2015-2022 Xunyi Ltd. All rights reserved.
 *
 */

#pragma once

#include <functional>
#include <iostream>
#include <unordered_map>
#include <thread>
#include <string>
#include <vector>

#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

typedef websocketpp::server<websocketpp::config::asio> server;

class WebsocketServer {
 public:
  using message_cb = std::function<void(std::string)>;
  using open_cb = std::function<void()>;
  using close_cb = std::function<void()>;

  WebsocketServer();

  void run(uint16_t port);

  /**
   * @brief  发送数据到指定的客户端
   * 
   * @param msg 
   * @param ip 
   */
  void send_to_client(std::string msg, std::string ip);

  void set_message_cb(message_cb cb) {m_message_cb = std::move(cb);};
  void set_open_cb(open_cb cb) {m_open_cb = std::move(cb);};
  void set_close_cb(close_cb cb) {m_close_cb = std::move(cb);};

  std::vector<std::string> get_ips();

 private:
  void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg);
  void on_open(websocketpp::connection_hdl hdl);
  void on_close(websocketpp::connection_hdl hdl);

  std::unordered_map<std::string, server::connection_ptr> ip_connection_map;
  server m_server;
  message_cb m_message_cb;
  open_cb m_open_cb;
  close_cb m_close_cb;
};