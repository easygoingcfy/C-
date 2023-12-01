#include <ros/ros.h>

#include "millimeter_wave_radar.cc"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "radar");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Radar radar;
  radar.run();

  spinner.stop();
  return 0;
}