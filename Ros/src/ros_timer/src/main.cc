#include <vector>

#include "ros/ros.h"

class Class {
 public:
  Class() : nh() {
    auto timer_ = std::make_shared<ros::Timer>(nh.createTimer(ros::Duration(1.), &Class::timerCallback, this));
    timers_.emplace_back(timer_);
  }

  void timerCallback(const ros::TimerEvent& event) {
    ROS_INFO("Timer callback triggered at time %.2f", event.current_real.toSec());
  }

  void run() { ros::spin(); }

 private:
  ros::NodeHandle nh;
  // ros::Timer timer_;
  std::vector<std::shared_ptr<ros::Timer>> timers_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "timer_example_node");
  Class cls;
  cls.run();

  return 0;
}