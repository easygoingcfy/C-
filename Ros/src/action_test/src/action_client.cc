#include <ros/ros.h>
#include <stdint.h>
#include <iostream>

#include <actionlib/client/simple_action_client.h>

#include <cfy/FocusAdjustAction.h>  // Note: "Action" is appended

typedef actionlib::SimpleActionClient<cfy::FocusAdjustAction> Client;

int main(int argc, char** argv) {
  ros::init(argc, argv, "do_dishes_client");
  Client client("do_dishes", true);  // true -> don't need ros::spin()
  client.waitForServer();
  cfy::FocusAdjustGoal goal;
  goal.focus_adjust_cmd = goal.FOCUS_ADJUST_CONT_ZOOM_IN;
  // Fill in goal here
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean\n");
  auto result_ = client.getResult();
  std::cout << "result: " << int(result_->result) << std::endl;
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}