/**
 * @file asio_timer.cpp
 * @author caofangyu (caofy@antwork.link)
 * @brief  timer to run timed task
 * @version 0.1
 * @date 2023-09-25
 *
 * Copyright (c) 2015-2022 Xunyi Ltd. All rights reserved.
 *
 */

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp>

class AsioTimer : public std::enable_shared_from_this<AsioTimer> {
 public:
  AsioTimer(
      boost::asio::io_service &io_service,
      std::function<void(const boost::system::error_code &e)> timeout_handler)
      : timer_(io_service), timeout_handler_(std::move(timeout_handler)) {}

  ~AsioTimer() {}

  void start(const uint64_t timeout_ms) { reset(timeout_ms); }

  void stop() { is_running_.store(false); }

  void reset(const uint64_t timeout_ms) {
    is_running_.store(true);
    do_set_expired(timeout_ms);
  }

 private:
  void do_set_expired(const uint64_t timeout_ms) {
    if (!is_running_.load()) {
      return;
    }

    timer_.expires_from_now(std::chrono::milliseconds(timeout_ms));
    // auto sthis = shared_from_this();
    timer_.async_wait([this, timeout_ms](const boost::system::error_code &e) {
      if (e.value() == boost::asio::error::operation_aborted ||
          !is_running_.load()) {
        return;
      }
      timeout_handler_(e);
      do_set_expired(timeout_ms);
    });
  }

 private:
  // The actual boost timer.
  boost::asio::steady_timer timer_;

  std::atomic<bool> is_running_ = {false};
  // The handler that will be triggered once the time's up.
  std::function<void(const boost::system::error_code &ec)> timeout_handler_;
};