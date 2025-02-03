// To avoid multiple definitions of the class
#ifndef VELOCITIES_H
#define VELOCITIES_H


//Standard include for Ros
#include "ros/ros.h"

//Utility Inludes


//Message includes
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"                   

//Dynamic Configuration includes
#include <dynamic_reconfigure/server.h>                 
#include <robottino/robot_parametersConfig.h>


class Velocities {
  public:

    // Constructor
    Velocities(); 

    //Main loop
    void main_loop();

    // Functions

    void geo_msg_writting(int &index);

    // Callbacks

    void joint_message_update_Callback(const sensor_msgs::JointState::ConstPtr& jointmsg);
    void robot_parameters_callback(robottino::robot_parametersConfig &config, uint32_t level);
    


  private:

  //Node Handler and All subs and pubs

    ros::NodeHandle Velocities_Node_Handler; 
    ros::Subscriber Tick;
    ros::Publisher Velos_pub;

  //Message Declaration

    geometry_msgs::TwistStamped geo_msg;

  // Parameters

    double r;
    double l;
    double w;
    double T;
    double N;
    int freq;

  // Conversion Variables

  // Time

    double tnowsec;
    double tnownsec;
    double tpresec;
    double tprensec;
    double deltat;     //deltat = (tnowsec-tpresec)*10^9+(tnownsec-tprensec);

  // Array[Front_Left, Front_Right, Rear_Left, Rear_Right]

    double posnow[4];
    double pospre[4];
    double deltapos[4];   //deltapos = posnow-pospre;
    double omega[4];      //(deltapos/deltat)*(2*3.14159/(5*42));

};

#endif