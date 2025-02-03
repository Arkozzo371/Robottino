#include "ros/ros.h"

#include "robottino/Velocities.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "Velocities_node");

  Velocities my_velocities;

  my_velocities.main_loop();

  return 0;
}