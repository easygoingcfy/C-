#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "nlohmann/json.hpp"
#include <google/protobuf/util/json_util.h>
#include "test.pb.h"

#include "server.h"

using json = nlohmann::json;

void msg_cb(std::string msg) {
  std::cout << "receive :" << msg << std::endl;
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
  if (data.contains("xx")) {
    std::cout << data["xx"] << std::endl;
  } else {
    std::cout << "no xx" << std::endl;
  }
  return;
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
  server.set_message_cb(msg_cb);
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
    server.send_to_client(person_json, "127.0.0.1");

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