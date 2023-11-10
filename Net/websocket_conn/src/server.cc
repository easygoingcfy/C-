#include "server.h"

typedef websocketpp::server<websocketpp::config::asio> server;

WebsocketServer::WebsocketServer() {
  // Set logging settings
  m_server.set_access_channels(websocketpp::log::alevel::all);
  m_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

  // Initialize Asio
  m_server.init_asio();

  // Register our message handler
  m_server.set_message_handler(
      std::bind(&WebsocketServer::on_message, this, std::placeholders::_1, std::placeholders::_2));
}

void WebsocketServer::on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
  std::cout << "on_message called with hdl: " << hdl.lock().get() << " and message: " << msg->get_payload()
            << std::endl;

  server::connection_ptr con = m_server.get_con_from_hdl(hdl);
  std::cout << "host:" << con->get_host() << " port: " << con->get_port() << 
      " uri: " << con->get_uri();

  // Check for a special command to instruct the server to stop listening so it can be cleanly exited.
  if (msg->get_payload() == "stop-listening") {
    m_server.stop_listening();
    return;
  }

  std::this_thread::sleep_for(std::chrono::seconds(5));
  try {
    m_server.send(hdl, msg->get_payload(), msg->get_opcode());
  } catch (websocketpp::exception const& e) {
    std::cout << "Echo failed because: "
              << "(" << e.what() << ")" << std::endl;
  }
}

void WebsocketServer::run(uint16_t port) {
  // Listen on specified port number
  std::cout << "server run in port: " << port << std::endl;
  m_server.listen(port);

  // Start the server accept loop
  m_server.start_accept();

  // Start the ASIO io_service run loop
  m_server.run();
}