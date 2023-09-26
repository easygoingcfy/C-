/**
 * @file interface.cpp
 * @author caofangyu (caofy@antwork.link)
 * @brief  moutable device connection interface
 * @version 0.1
 * @date 2023-09-25
 * 
 * Copyright (c) 2015-2022 Xunyi Ltd. All rights reserved.
 * 
 */

#include "interface.h"
#include "tcp.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace flight_brain {
namespace mountable {
ConnInterface::ConnInterface()
    : tx_total_bytes_(0),
      rx_total_bytes_(0),
      last_tx_total_bytes_(0),
      last_rx_total_bytes_(0),
      last_iostat_(steady_clock::now()) {}

/**
 * Parse host:port pairs
 */
static void url_parse_host(std::string host, std::string &host_out,
                           int &port_out, const std::string def_host,
                           const int def_port) {
  std::string port;

  auto sep_it = std::find(host.begin(), host.end(), ':');
  if (sep_it == host.end()) {
    // host
    if (!host.empty()) {
      host_out = host;
      port_out = def_port;
    } else {
      host_out = def_host;
      port_out = def_port;
    }
    return;
  }

  if (sep_it == host.begin()) {
    // :port
    host_out = def_host;
  } else {
    // host:port
    host_out.assign(host.begin(), sep_it);
  }

  port.assign(sep_it + 1, host.end());
  port_out = std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 */
static void url_parse_query(std::string query) {
  const std::string ids_end("ids=");
  std::string sys, comp;

  if (query.empty()) return;

  auto ids_it =
      std::search(query.begin(), query.end(), ids_end.begin(), ids_end.end());
  if (ids_it == query.end()) {
    return;
  }

  std::advance(ids_it, ids_end.length());
  auto comma_it = std::find(ids_it, query.end(), ',');
  if (comma_it == query.end()) {
    return;
  }
}

static ConnInterface::Ptr url_parse_tcp_client(std::string host,
                                                 std::string query) {
  std::string server_host;
  int server_port;

  // tcp://localhost:5760
  url_parse_host(host, server_host, server_port, "localhost", 5760);
  url_parse_query(query);

  return std::make_shared<ConnTcpClient>(server_host, server_port);
}

ConnInterface::Ptr ConnInterface::open_url(std::string url) {
  /* Based on code found here:
   * http://stackoverflow.com/questions/2616011/easy-way-to-parse-a-url-in-c-cross-platform
   */

  const std::string proto_end("://");
  std::string proto;
  std::string host;
  std::string path;
  std::string query;

  auto proto_it =
      std::search(url.begin(), url.end(), proto_end.begin(), proto_end.end());

  // copy protocol
  proto.reserve(std::distance(url.begin(), proto_it));
  std::transform(url.begin(), proto_it, std::back_inserter(proto),
                 std::ref(tolower));

  // copy host
  std::advance(proto_it, proto_end.length());
  auto path_it = std::find(proto_it, url.end(), '/');
  std::transform(proto_it, path_it, std::back_inserter(host),
                 std::ref(tolower));

  // copy path, and query if exists
  auto query_it = std::find(path_it, url.end(), '?');
  path.assign(path_it, query_it);
  if (query_it != url.end()) ++query_it;
  query.assign(query_it, url.end());

  printf("URL: %s: proto: %s, host: %s, path: %s, query: %s", url.c_str(),
         proto.c_str(), host.c_str(), path.c_str(), query.c_str());

  if (proto == "tcp")
    return url_parse_tcp_client(host, query);
  else
    throw DeviceError("url", "Unknown URL type");
}
}  // namespace flight_brain::mountable
}  // namespace flight_brain