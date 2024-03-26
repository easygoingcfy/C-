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
#include <deque>

#include <ros/ros.h>

#include "asio_timer.cpp"

namespace flight_brain {
namespace conn {
using steady_clock = std::chrono::steady_clock;
using lock_guard = std::lock_guard<std::mutex>;
using boost::asio::io_service;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;
using boost::system::error_code;

/**
 * @brief Common exception for communication error
 */
class DeviceError : public std::runtime_error {
   public:
    /**
     * @breif Construct error.
     */
    template <typename T>
    DeviceError(const char *module, T msg) : std::runtime_error(make_message(module, msg)) {}

    template <typename T>
    static std::string make_message(const char *module, T msg) {
        std::ostringstream ss;
        ss << "DeviceError:" << module << ":" << msg_to_string(msg);
        return ss.str();
    }

    static std::string msg_to_string(const char *description) { return description; }

    static std::string msg_to_string(boost::system::system_error &err) { return err.what(); }
};

/**
 * @brief client interface
 *
 */
//TODO(caofy: )
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
    virtual ~ConnInterface();

    // virtual function
    virtual void connect() = 0;

    /**
     * @brief  close connection
     *
     */
    virtual void close() = 0;
    virtual bool is_open() = 0;
    virtual void do_send(bool check_tx_state) = 0;
    virtual void do_receive() = 0;

    /**
     * @brief 在独立线程中运行io_service
     *
     */
    void run();

    virtual void send_message(const std::string &message) = 0;
    virtual void send_bytes(const void *data, size_t len) = 0;


    void run_every(const uint64_t timeout_ms, TimerCallback cb);


    void set_receive_callback(ReceiveCb cb) { receive_cb_ = std::move(cb); }
    void set_conn_callback(ConnectionCb cb) { conn_cb_ = std::move(cb); }
    void set_closed_callback(ClosedCb cb) { closed_cb_ = std::move(cb); }

    /**
     * @brief Construct connection from URL
     *
     * Supported URL schemas:
     * - tcp://
     *
     * @param[in] url    resource locator
     * @return Ptr
     */
    static Ptr open_url(std::string url);



    // for statistic: 目前没有使用
    std::atomic<size_t> tx_total_bytes_, rx_total_bytes_;
    size_t last_tx_total_bytes_, last_rx_total_bytes_;
    std::chrono::time_point<steady_clock> last_iostat_;

    std::string protocol_type_;

    ClosedCb closed_cb_;
    ReceiveCb receive_cb_;
    ConnectionCb conn_cb_;

    boost::asio::io_service io_service_;
    std::unique_ptr<boost::asio::io_service::work> io_work_;
    std::thread io_thread_;

    std::atomic<bool> is_destroying_;
    std::mutex mutex_;
    std::vector<std::shared_ptr<AsioTimer>> timer_array_;

    enum { max_msg_size = 1024 };
    char read_buffer_[max_msg_size];
    char write_buffer_[max_msg_size];
    std::atomic<bool> tx_in_progress_;
    std::deque<std::string> tx_queue_;
};

}  // namespace conn
}  // namespace flight_brain