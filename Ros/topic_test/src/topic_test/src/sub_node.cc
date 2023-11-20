#include <ros/ros.h>

#include "topic_test/People.h"

using namespace topic_test;

void PeopleCb(const PeopleConstPtr& msg) {
  std::cout << "receive msg:" << std::endl;
  std::cout << "id:" << (int)msg->id << std::endl;
  std::cout << "name:" << msg->name << std::endl;
  std::cout << "age:" << (int)msg->age << std::endl;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sub_node");
  ros::NodeHandle nh;

  ros::Subscriber suber = nh.subscribe("pub_node/people", 10, PeopleCb);
  ROS_INFO_STREAM("sub topic");
  ros::spin();
  ROS_INFO_STREAM("spin end");

  ros::waitForShutdown();
  std::cout << "shutdown node..." << std::endl;
}