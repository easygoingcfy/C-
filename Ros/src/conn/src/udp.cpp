/**
 * @file udp.cpp
 * @author caofangyu (caofy@antwork.link)
 * @brief
 * @version 0.1
 * @date 2024-03-25
 *
 * Copyright (c) 2015-2024 Xunyi Ltd. All rights reserved.
 *
 */

#include "udp.h"

namespace flight_brain {
namespace conn {
static void resolve_address_udp(io_service& io, const std::string& ip, int port, udp::endpoint& ep) {
    udp::resolver resolver(io);
    udp::resolver::query query(udp::v4(), ip, std::to_string(port));

    // resolver.async_resolve(query, [&](const error_code& ec, udp::resolver::iterator it) {
    //     if (ec) {
    //         ROS_ERROR("resolve address error: %s", ec.message().c_str());
    //         return;
    //     }
    //     ep = *it;
    // });
    // 使用同步的 resolve 方法替代异步的 async_resolve 方法
    udp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    // 检查是否有错误发生
    if (endpoint_iterator == udp::resolver::iterator()) {
        ROS_ERROR("Failed to resolve address");
        return;
    }

    // 使用解析得到的第一个端点
    ep = *endpoint_iterator;
    if (ep.address().is_v4()) {
        ROS_INFO("Resolved address: %s", ep.address().to_string().c_str());
    }
}

ConnUdpClient::ConnUdpClient(const std::string& ip, int port)
    : ConnInterface(),
      socket_(io_service_) {
    ROS_INFO("ConnUdpClient::ConnUdpClient()");
    protocol_type_ = "udp";
    socket_.open(udp::v4());
    io_service_.post(std::bind(&ConnUdpClient::do_receive, this));
    resolve_address_udp(io_service_, ip, port, endpoint_);  // 异步操作：在执行io_service_.run()后才会执行
}

ConnUdpClient::~ConnUdpClient() {
    ROS_INFO("%s desctructed", protocol_type_.c_str());
    is_destroying_ = true;
    lock_guard lock(mutex_);
    for (auto &timer : timer_array_) {
        timer->stop();
    }
    timer_array_.clear();
    close();
}

void ConnUdpClient::close() {
    ROS_INFO("ConnUdpClient::close()");
    lock_guard lock(mutex_);

    if (!is_open()) return;
    socket_.cancel();
    socket_.close();

    io_work_.reset();
    io_service_.stop();
    if (io_thread_.joinable()) io_thread_.join();
    io_service_.reset();

    if (closed_cb_) closed_cb_();
}

void ConnUdpClient::send_message(const std::string &message) {
    {
        lock_guard lock(mutex_);
        tx_queue_.emplace_back(message);
    }
    io_service_.post(boost::bind(&ConnUdpClient::do_send, shared_from_this(), true));
}

void ConnUdpClient::send_bytes(const void *data, size_t len) {
    {
        lock_guard lock(mutex_);
        tx_queue_.emplace_back((const char *)data, len);
    }
    io_service_.post(std::bind(&ConnUdpClient::do_send, shared_from_this(), true));
}


void ConnUdpClient::do_send(bool check_tx_state) {
    if (check_tx_state && tx_in_progress_) return;

    lock_guard lock(mutex_);
    if (tx_queue_.empty()) return;

    tx_in_progress_ = true;
    auto sthis = shared_from_this();
    // TODO(caofy): 自己编写缓冲区类
    auto& buf_ref = tx_queue_.front();
    ROS_INFO("do send");

    socket_.async_send_to(boost::asio::buffer(buf_ref.data(), buf_ref.length()), endpoint_,
                          [sthis, &buf_ref](const error_code& ec, size_t bytes_transferred) {
                              assert(bytes_transferred <= buf_ref.length());
                              if (ec) {
                                  ROS_ERROR("send error: %s", ec.message().c_str());
                                  sthis->tx_in_progress_ = false;
                                  return;
                              }
                              lock_guard lock(sthis->mutex_);
                              if (sthis->tx_queue_.empty() || sthis->is_destroying_) {
                                  sthis->tx_in_progress_ = false;
                                  return;
                              }
                              sthis->tx_queue_.pop_front();

                              if (!sthis->tx_queue_.empty()) {
                                  sthis->do_send(false);
                              } else {
                                  sthis->tx_in_progress_ = false;
                              }
                          });
}

void ConnUdpClient::do_receive() {
    ROS_INFO("do receive");
    if (is_destroying_) return;
    auto sthis = shared_from_this();
    socket_.async_receive(boost::asio::buffer(read_buffer_), [sthis](const error_code& ec, size_t bytes_transferred) {
        if (ec) {
            ROS_ERROR("recv error: %s", ec.message().c_str());
            return;
        }
        ROS_INFO_STREAM("recv: " << sthis->read_buffer_);
        if (sthis->receive_cb_) {
            lock_guard lock(sthis->mutex_);
            sthis->receive_cb_(sthis->read_buffer_, bytes_transferred);
        }
        sthis->do_receive();
    });
}

}  // namespace conn
}  // namespace flight_brain