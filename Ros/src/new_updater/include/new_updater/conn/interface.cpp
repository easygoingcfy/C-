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

ConnInterface::Ptr ConnInterface::create_client(std::string host, int port,
                                                std::string proto) {
  if (proto == "udp") {
  }  // TODO(antwork): 暂时只有tcp协议的云台
  return std::make_shared<ConnTcpClient>(host, port);
}
}  // namespace flight_brain::mountable
}  // namespace flight_brain