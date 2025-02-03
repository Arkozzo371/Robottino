#include "ros/ros.h"

#include "robottino/Odometry.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "Odometry_node");

  Odometry my_odometry;

  my_odometry.main_loop();

  return 0;
}