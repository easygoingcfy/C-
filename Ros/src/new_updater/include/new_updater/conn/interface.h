/**
 * @file interface.h
 * @author caofangyu (caofy@antwork.link)
 * @brief  moutable device connection interface
 * @version 0.1
 * @date 2023-09-25
 *
 * Copyright (c) 2015-2022 Xunyi Ltd. All rights reserved.
 *
 */

#pragma once

#include <boost/system/system_error.hpp>

#include <atomic>
#include <cassert>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <ros/ros.h>

namespace flight_brain {
namespace mountable {
using steady_clock = std::chrono::steady_clock;
using lock_guard = std::lock_guard<std::recursive_mutex>;

/**
 * @brief Common exception for communication error
 */
class DeviceError : public std::runtime_error {
 public:
  /**
   * @breif Construct error.
   */
  template <typename T>
  DeviceError(const char *module, T msg)
      : std::runtime_error(make_message(module, msg)) {}

  template <typename T>
  static std::string make_message(const char *module, T msg) {
    std::ostringstream ss;
    ss << "DeviceError:" << module << ":" << msg_to_string(msg);
    return ss.str();
  }

  static std::string msg_to_string(const char *description) {
    return description;
  }

  static std::string msg_to_string(boost::system::system_error &err) {
    return err.what();
  }
};

/**
 * @brief client interface
 *
 */
class ConnInterface {
 private:
  ConnInterface(const ConnInterface &) = delete;

 public:
  using Ptr = std::shared_ptr<ConnInterface>;
  using ConstPtr = std::shared_ptr<ConnInterface const>;
  using ReceiveCb = std::function<void(char *, size_t)>;
  using ConnectionCb = std::function<void()>;
  using ClosedCb = std::function<void()>;
  typedef std::function<void()> TimerCallback;

  /**
   * @brief Construct a new Gimbal Interface object
   *
   * @param server_host    remote host
   * @param server_port    remote port
   */
  ConnInterface();

  /**
   * @brief  close connection
   *
   */
  virtual void close() = 0;

  // virtual void Connect() = 0;
  // virtual void Disconnect() = 0;

  virtual void send_message(const std::string message) = 0;
  virtual void send_bytes(const void *data, size_t len) = 0;
  virtual bool is_open() = 0;

  virtual void connect() = 0;
  virtual void run() = 0;
  virtual void run_every(const uint64_t timeout_ms, TimerCallback cb) = 0;
  virtual void set_receive_callback(ReceiveCb cb) = 0;
  virtual void set_conn_callback(ConnectionCb cb) = 0;
  virtual void set_closed_callback(ClosedCb cb) = 0;

  /**
   * @brief Construct connection from URL
   *
   * Supported URL schemas:
   * - tcp://
   *
   * @param[in] url    resource locator
   * @return Ptr
   */
  static Ptr create_client(std::string host, int port, std::string proto);

 private:
  // for statistic
  std::atomic<size_t> tx_total_bytes_, rx_total_bytes_;
  std::recursive_mutex iostat_mutex_;
  size_t last_tx_total_bytes_, last_rx_total_bytes_;
  std::chrono::time_point<steady_clock> last_iostat_;
};

}  // namespace mountable
}  // namespace flight_brain