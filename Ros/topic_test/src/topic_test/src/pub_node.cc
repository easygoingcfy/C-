#include <ros/ros.h>

#include <thread>

#include "topic_test/People.h"

using namespace topic_test;


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pub_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher publisher;
    publisher = pnh.advertise<People>("people", 10);

    People cfy;
    cfy.id = 0;
    cfy.name = "caofangyu";
    cfy.age = 26;
    while (++cfy.id) {
        ROS_INFO_STREAM("pub topic");
        publisher.publish(cfy);
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    ros::waitForShutdown();
    std::cout << "shutdown node..." << std::endl;
}