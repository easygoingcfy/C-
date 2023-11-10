#include "client.h"

WebsocketClient::WebsocketClient() {
  m_client.set_access_channels(websocketpp::log::alevel::all);
  m_client.clear_access_channels(websocketpp::log::alevel::frame_payload);

  m_client.init_asio();

  m_client.set_message_handler(
      std::bind(&WebsocketClient::on_message, this, std::placeholders::_1, std::placeholders::_2));
  m_client.set_open_handler(std::bind(&WebsocketClient::on_open, this, std::placeholders::_1));
  m_client.set_close_handler(std::bind(&WebsocketClient::on_close, this, std::placeholders::_1));
}

void WebsocketClient::connect(const std::string& uri) {
  websocketpp::lib::error_code ec;
  client::connection_ptr con = m_client.get_connection(uri, ec);
  if (ec) {
    std::cout << "Failed to create connection: " << ec.message() << std::endl;
    return;
  }
  m_client.connect(con);
}

void WebsocketClient::send(websocketpp::connection_hdl hdl, const std::string& message) {
  // Send a message to the server
  std::cout << "client send: " << message << std::endl;
  std::error_code ec;
  m_client.send(hdl, message.c_str(), message.size(), websocketpp::frame::opcode::TEXT, ec);
  if (ec) {
    std::cout << "send error: " << ec.message() << std::endl;
  }
}

void WebsocketClient::run(const std::string& server) {
  // Connect to the server and start the ASIO io_service run loop
  connect(server);
  //std::thread run_thread(&client::run, &m_client);
  //run_thread.detach();
  m_client.run();
}

void WebsocketClient::on_message(websocketpp::connection_hdl hdl, message_ptr msg) {
  std::cout << "client on_message called with hdl: " << hdl.lock().get() << " and message: " << msg->get_payload()
            << std::endl;
  client::connection_ptr con = m_client.get_con_from_hdl(hdl);
  std::cout << "host:" << con->get_host() << " port: " << con->get_port() << 
      " uri: " << con->get_uri();
  websocketpp::lib::error_code ec;
  m_client.send(hdl, msg->get_payload(), msg->get_opcode(), ec);
  if (ec) {
    std::cout << "Echo faild because: " << ec.message() << std::endl;
  }
}

void WebsocketClient::on_open(websocketpp::connection_hdl hdl) {
  std::cout << "on_open called with hdl: " << hdl.lock().get() << std::endl;
  send(hdl, "hello");
}

void WebsocketClient::on_close(websocketpp::connection_hdl hdl) {
  std::cout << "on_close called with hdl: " << hdl.lock().get() << std::endl;
}