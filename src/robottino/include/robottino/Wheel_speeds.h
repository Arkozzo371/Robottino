// To avoid multiple definitions of the class
#ifndef WHEEL_SPEEDS_H
#define WHEEL_SPEEDS_H


//Standard include for Ros
#include "ros/ros.h"

//Utility Inludes


//Message includes
#include "geometry_msgs/TwistStamped.h"
#include "robottino/Wheel_speeds_message.h"                  
 
//Dynamic Configuration includes
#include <dynamic_reconfigure/server.h>                 
#include <robottino/robot_parametersConfig.h>     


class Wheel_speeds {
  public:

    // Constructor
    Wheel_speeds(); 

    //Main loop
    void main_loop();

    // Functions

    void Wheel_speeds_msg_writting(int &index);

    // Callbacks

    void geo_message_update_Callback(const geometry_msgs::TwistStamped::ConstPtr& geo_msg);
    void robot_parameters_callback(robottino::robot_parametersConfig &config, uint32_t level);


  private:

  //Node Handler and All subs and pubs

    ros::NodeHandle Wheel_speeds_Node_Handler; 
    ros::Subscriber Velos_sub;
    ros::Publisher RPM_pub;

  //Message Declaration

    robottino::Wheel_speeds_message custom_msg;

  // Parameters

    double r;
    double l;
    double w;
    double T;
    double N;
    int freq;

  // Time

    double tnowsec;
    double tnownsec; 

  // Velocities

    double v_x;
    double v_y;
    double omega;
};

#endif