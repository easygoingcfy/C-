#include <ros/ros.h>
#include <stdio.h>
#include <iostream>

#include <actionlib/server/simple_action_server.h>
#include <cfy/FocusAdjustAction.h>

#include "mapping.h"
#include "tcp.h"

using namespace flight_brain::mountable;

typedef actionlib::SimpleActionServer<cfy::FocusAdjustAction> Server;

void execute(const cfy::FocusAdjustGoalConstPtr& goal,
             Server* as)  // Note: "Action" is not appended to DoDishes here
{
  // Do lots of awesome groundbreaking robot stuff here

  std::shared_ptr<ConnTcpClient> client = std::make_shared<ConnTcpClient>();
  //client->set_receive_callback(&on_receive);
  client->connect();
  client->run();
  uint8_t cmd = goal->focus_adjust_cmd;
  std::cout << "cmd: " << int(cmd) << std::endl;
  cfy::FocusAdjustResult result_;
  result_.result = focus_map[cmd];
  client->send_bytes((void*)&result_, sizeof(result_));
  client->close();
  as->setSucceeded(result_);
}

void on_receive(char* data, size_t len) {
    std::string msg(data, len);
    std::cout << "receive: " << msg << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle n;
  Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}