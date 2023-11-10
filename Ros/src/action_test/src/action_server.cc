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
             Server* as,
             std::shared_ptr<ConnTcpClient>& client)  // Note: "Action" is not appended to DoDishes here
{
  // Do lots of awesome groundbreaking robot stuff here

  uint8_t cmd = goal->focus_adjust_cmd;
  std::cout << "cmd: " << int(cmd) << std::endl;
  cfy::FocusAdjustResult result_;
  result_.result = focus_map[cmd];
  //client->send_bytes((void*)&result_, sizeof(result_));
  client->send_bytes((void*)&cmd, sizeof(cmd));
  as->setSucceeded();
  printf("execute done\n");
}

void on_receive(char* data, size_t len) {
  std::string msg(data, len);
  std::cout << "receive: " << msg << std::endl;
}

int main(int argc, char** argv) {
  printf("ros init\n");
  ros::init(argc, argv, "focus_server");
  ros::NodeHandle n;
  //client
  std::shared_ptr<ConnTcpClient> client = std::make_shared<ConnTcpClient>();
  // client->set_receive_callback(&on_receive);
  client->connect();
  client->run();
  printf("init server\n");
  Server server(n, "focus", boost::bind(&execute, _1, &server, client), false);
  printf("start server\n");
  server.start();
  printf("ros spin\n");
  ros::spin();
  return 0;
}