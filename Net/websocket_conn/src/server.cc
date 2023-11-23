#include "server.h"
#include "util.h"

WebsocketServer::WebsocketServer() : m_message_cb(nullptr), m_close_cb(nullptr), m_open_cb(nullptr) {
  // Set logging settings
  m_server.set_access_channels(websocketpp::log::alevel::all);
  m_server.clear_access_channels(websocketpp::log::alevel::frame_payload);
  // Initialize Asio
  m_server.init_asio();
  // Register handler
  m_server.set_message_handler(
      std::bind(&WebsocketServer::on_message, this, std::placeholders::_1, std::placeholders::_2));
  m_server.set_open_handler(std::bind(&WebsocketServer::on_open, this, std::placeholders::_1));
  m_server.set_close_handler(std::bind(&WebsocketServer::on_close, this, std::placeholders::_1));
}

void WebsocketServer::send_to_client(std::string message, std::string ip) {
  std::cout << "send to client " << ip << std::endl;
  if (ip_connection_map.find(ip) == ip_connection_map.end()) {
    std::cout << "Invalid ip: " << ip << std::endl;
    return;
  }
  server::connection_ptr con = ip_connection_map[ip];
  auto hdl = con->get_handle();
  std::error_code ec;
  m_server.send(hdl, message.c_str(), message.size(), websocketpp::frame::opcode::TEXT, ec);
  if (ec) {
    std::cout << "server send error: " << ec.message() << std::endl;
  }
}

void WebsocketServer::on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
  std::cout << "on_message called with hdl: " << hdl.lock().get() << " and message: " << msg->get_payload()
            << std::endl;

  server::connection_ptr con = m_server.get_con_from_hdl(hdl);
  // Check for a special command to instruct the server to stop listening so it can be cleanly exited.
  if (msg->get_payload() == "stop-listening") {
    m_server.stop_listening();
    return;
  }
  if (m_message_cb) {
    m_message_cb(msg->get_payload());
  }
}

void WebsocketServer::on_open(websocketpp::connection_hdl hdl) {
  auto con = m_server.get_con_from_hdl(hdl);
  std::string ip = websocket_conn::get_ip_from_str(con->get_remote_endpoint());
  if (ip_connection_map.find(ip) != ip_connection_map.end()) {
    std::cout << "Duplicate connection from:" << ip << std::endl;
    std::cout << "Replace original conn:" << ip_connection_map[ip] << std::endl;
  } else {
    std::cout << "New connection from: " << ip << std::endl;
  }
  ip_connection_map[ip] = con;
  std::cout << "current connection: " << ip_connection_map.size() << std::endl;
  if (m_open_cb) m_open_cb();
}

void WebsocketServer::on_close(websocketpp::connection_hdl hdl) {
  auto con = m_server.get_con_from_hdl(hdl);
  for (auto it = ip_connection_map.begin(); it != ip_connection_map.end();) {
    if (it->second == con) {
      it = ip_connection_map.erase(it);
    } else {
      ++it;
    }
  }
  std::cout << "current connection: " << ip_connection_map.size() << std::endl;
  if (m_close_cb) m_close_cb();
}

void WebsocketServer::run(uint16_t port) {
  // Listen on specified port number
  std::cout << "server run in port: " << port << std::endl;
  m_server.listen(port);
  // Start the server accept loop
  m_server.start_accept();
  // Start the ASIO io_service run loop
  std::thread run_thread(&server::run, &m_server);
  run_thread.detach();
  // m_server.run();
}

std::vector<std::string> WebsocketServer::get_ips() {
  std::vector<std::string> ips;
  for (auto const& pair : ip_connection_map) {
    ips.emplace_back(pair.first);
  }
  return ips;
}