#include <google/protobuf/util/json_util.h>

#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "nlohmann/json.hpp"
#include "server.h"
#include "test.pb.h"

using json = nlohmann::json;

// void msg_cb(std::string msg) {
//   std::cout << "receive :" << msg << std::endl;
// }

/*
测试服务器：
请求：
{
  "billboard_id": id,
  "cmd_type": req,
}

返回：
{
  "billboard_id": id,
  "cmd_type":
  "msg": string,
}
*/

void msg_cb(WebsocketServer* server, std::string msg) {
  std::cout << "in msg_cb" << std::endl;
  json response;
  json parsed_data;
  try {
    parsed_data = json::parse(msg);
  } catch (nlohmann::detail::exception& e) {
    response["cmd_type"] = 7;
    response["msg"] = e.what();
    for (auto const& ip : server->get_ips()) {
      server->send_to_client(response.dump(), ip);
    }
    return;
  }
  if (!parsed_data.contains("billboard_id") || !parsed_data.contains("cmd_type")) {
    for (auto const& ip : server->get_ips()) {
      response["cmd_type"] = 7;
      response["msg"] = "json must contains billboard_id cmd_type field";
      server->send_to_client(response.dump(), ip);
    }
  } else {
    try {
      int cmd_type = parsed_data.at("cmd_type");
    } catch (nlohmann::detail::exception& e) {
      response["cmd_type"] = 7;
      response["msg"] = e.what();
      for (auto const& ip : server->get_ips()) {
        server->send_to_client(response.dump(), ip);
      }
      return;
    }
    response["billboard_id"] = parsed_data.at("billboard_id");
    json ad_info_1 = json::parse(R"(
          {
            "ad_id": 1,
            "ad_type": 0,
            "url": "url1",
            "sec": 100
          }
        )");
    json ad_info_2 = json::parse(R"(
        {
          "ad_id": 2,
          "ad_type": 1,
          "url": "url2",
          "sec": 7
        }
        )");
    json ad_info_3 = json::parse(R"(
        {
          "ad_id": 3,
          "ad_type": 2,
          "url": "url3",
          "sec": 777
        }
        )");
    response["cmd_type"] = parsed_data["cmd_type"];
    switch (int(parsed_data.at("cmd_type"))) {
      case 1:
        response["ad_infos"] = {ad_info_1, ad_info_2, ad_info_3};
        break;
      case 2:
        response["control_type"] = 4;
        response["control_param"] = 50;
        break;
      case 3:
        response["volume"] = 0;
        response["ad_infos"] = {ad_info_1};
        break;
      case 4:
        response["url"] = "update_url";
        break;
      case 5:
        response["billboard_main_status"] = 1;
        response["billboard_base_status"] = 7;
        break;
      case 6:
        if (!parsed_data.contains("code")) {
          response["cmd_type"] = 7;
          response["msg"] = "no code!!!";
        } else {
          response["code"] = parsed_data.at("code");
        }
        break;
      default:
        response["cmd_type"] = 7;
        response["msg"] = "invalid cmd";
        break;
    }
    for (auto const& ip : server->get_ips()) {
      server->send_to_client(response.dump(), ip);
    }
  }
}

void serverThread() {
  // test json
  json data = json::parse(R"(
    {
      "name": "caofangyu",
      "age": 26,
      "id": "1"
    }
  )");
  std::string json_string = data.dump();
  std::cout << "json string: " << json_string << std::endl;
  // parse
  json parsed_data = json::parse(json_string);
  std::cout << "name: " << parsed_data["name"] << std::endl;
  std::cout << "age: " << parsed_data["age"] << std::endl;
  std::cout << "id: " << parsed_data["id"] << std::endl;

  // test proto
  test::People person;
  person.set_name("caofangyu");
  person.set_age("26");
  person.set_id(777);

  WebsocketServer server;
  server.set_message_cb(std::bind(msg_cb, &server, std::placeholders::_1));
  server.run(9002);
  int n = 0;
  std::stringstream ss;
  while (++n) {
    // nlohmann json test
    // data["seq"] = n;
    // server.send_to_client(data.dump(), "127.0.0.1");

    // proto
    person.set_seq(n);
    std::string person_json;
    google::protobuf::util::MessageToJsonString(person, &person_json);
    // server.send_to_client(person_json, "127.0.0.1");

    // std::cout << ss.str() << std::endl;
    // server.send_to_client(data.dump(), "192.168.3.66");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    ss.str("");
  };
}

int main() {
  std::thread st(serverThread);
  st.join();
}