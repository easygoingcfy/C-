#include <ros/ros.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>

#include <actionlib/client/simple_action_client.h>

#include <cfy/FocusAdjustAction.h>  // Note: "Action" is appended

typedef actionlib::SimpleActionClient<cfy::FocusAdjustAction> Client;

void send_goal() {}

int main(int argc, char** argv) {
  printf("ros init\n");
  ros::init(argc, argv, "focus_client");
  Client client("focus", true);  // true -> don't need ros::spin()
  client.waitForServer();
  printf("connected\n");
  cfy::FocusAdjustGoal goal;
  goal.focus_adjust_cmd = goal.FOCUS_ADJUST_CONT_ZOOM_IN;
  // Fill in goal here
  printf("send goal\n");
  client.sendGoal(goal);
  printf("wait for result\n");
  client.waitForResult(ros::Duration(1.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("succeeded\n");
  auto result_ = client.getResult();
  std::cout << "result: " << int(result_->result) << std::endl;
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}