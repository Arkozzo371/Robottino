//Standard include for Ros
#include "ros/ros.h"

//Message includes
#include "geometry_msgs/TwistStamped.h"
#include "robottino/Wheel_speeds_message.h"

//Class definition File include
#include "robottino/Wheel_speeds.h"                      

//Dynamic Configuration includes
#include <dynamic_reconfigure/server.h>                 
#include <robottino/robot_parametersConfig.h>           



Wheel_speeds::Wheel_speeds() { // class constructor

  //initializations of the sub and pub nodes 

  Velos_sub = Wheel_speeds_Node_Handler.subscribe("/cmd_vel", 1000, &Wheel_speeds::geo_message_update_Callback, this);
  RPM_pub = Wheel_speeds_Node_Handler.advertise<robottino::Wheel_speeds_message>("/wheels_rpm", 1000);


  Wheel_speeds_Node_Handler.getParam("/r", r);
  Wheel_speeds_Node_Handler.getParam("/l", l);
  Wheel_speeds_Node_Handler.getParam("/w", w);
  Wheel_speeds_Node_Handler.getParam("/T", T);
  Wheel_speeds_Node_Handler.getParam("/N", N);
  Wheel_speeds_Node_Handler.getParam("/freq", freq);

}

// Functions

void Wheel_speeds::Wheel_speeds_msg_writting(int &index){

    custom_msg.header.seq = index;
    custom_msg.header.stamp.sec = tnowsec;
    custom_msg.header.stamp.nsec = tnownsec;
    custom_msg.header.frame_id = "wheels_rpm";

    custom_msg.rpm_fl = (T*N)/r*( -(l+r) * omega + v_x - v_y);
    custom_msg.rpm_fr = (T*N)/r*( +(l+r) * omega + v_x + v_y);
    custom_msg.rpm_rl = (T*N)/r*( -(l+r) * omega + v_x + v_y);
    custom_msg.rpm_rr = (T*N)/r*( +(l+r) * omega + v_x - v_y);

    RPM_pub.publish(custom_msg);

  
}

// Callbacks

void Wheel_speeds::geo_message_update_Callback(const geometry_msgs::TwistStamped::ConstPtr& geo_msg){

  // Exctract Datas from geometry_msgs/TwistStamped

  tnowsec = geo_msg->header.stamp.sec;
  tnownsec = geo_msg->header.stamp.nsec;

  v_x = geo_msg->twist.linear.x;
  v_y = geo_msg->twist.linear.y;
  omega = geo_msg->twist.angular.z;
 
}


void Wheel_speeds::robot_parameters_callback(robottino::robot_parametersConfig &config, uint32_t level) {
  r = config.r;
  l = config.l;
  w = config.w;
  N = config.N;
}


// Main Loop

void Wheel_speeds::main_loop() {

  // Initialization of the message and its index
  int Wheel_speeds_msg_index = 0;
  

  //Initialization of the parameters and the 

  dynamic_reconfigure::Server<robottino::robot_parametersConfig> robot_param_Server;        //Initialization of the server
  dynamic_reconfigure::Server<robottino::robot_parametersConfig>::CallbackType f;           //Initialization of the server's callback
  
  f = boost::bind(&Wheel_speeds::robot_parameters_callback, this, _1, _2);                            
  robot_param_Server.setCallback(f);


  ros::Rate loop_rate(freq);

  //Main working loop

  while (ros::ok()) {       

    //Update of the Message
    Wheel_speeds_msg_writting(Wheel_speeds_msg_index);
    Wheel_speeds_msg_index++;

    //Cycle operations
    ros::spinOnce();
    loop_rate.sleep();

  }
}