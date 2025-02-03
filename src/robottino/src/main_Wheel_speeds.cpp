#include "ros/ros.h"

#include "robottino/Wheel_speeds.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "Wheel_speeds_node");

  Wheel_speeds my_wheel_speeds;

  my_wheel_speeds.main_loop();

  return 0;
}