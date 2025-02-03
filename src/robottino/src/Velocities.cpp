//Standard include for Ros
#include "ros/ros.h"

//Utility Inludes
#include <sstream>

//Message includes
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"

//Class definition File include
#include "robottino/Velocities.h"                      

//Dynamic Configuration includes
#include <dynamic_reconfigure/server.h>                 
#include <robottino/robot_parametersConfig.h>           



Velocities::Velocities() { // class constructor

  //initializations of the sub and pub nodes 

  Tick = Velocities_Node_Handler.subscribe("/wheel_states", 1000, &Velocities::joint_message_update_Callback, this);
  Velos_pub= Velocities_Node_Handler.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);


  //Initialization paramethers for Robot (Already Calibrated)

  Velocities_Node_Handler.getParam("/r", r);
  Velocities_Node_Handler.getParam("/l", l);
  Velocities_Node_Handler.getParam("/w", w);
  Velocities_Node_Handler.getParam("/T", T);
  Velocities_Node_Handler.getParam("/N", N);
  Velocities_Node_Handler.getParam("/freq", freq);

  //Inizialization of the visible values to 0 and all the others to 1 to avoid issue with divisions or sqrt

  tpresec = 1;
  tprensec = 1;
  tnowsec = 1;
  tnownsec = 1;
  deltat = 1;

  for (int i = 0; i<4; i++){

    pospre[i] = 1;
    posnow[i] = 1;
    deltapos[i] = 1;    
    omega[i] = 0;

  } 

}

// Functions

void Velocities::geo_msg_writting(int &index){

    geo_msg.header.seq = index;
    geo_msg.header.stamp.sec = tnowsec;
    geo_msg.header.stamp.nsec = tnownsec;
    geo_msg.header.frame_id = "base_link";

    geo_msg.twist.linear.x = (r/2)*( omega[0] + omega[1] + omega[2] + omega[3]);
    geo_msg.twist.linear.y = (r/2)*( - omega[0] + omega[1] + omega[2] - omega[3]);
    geo_msg.twist.linear.z = 0;

    geo_msg.twist.angular.x = 0;
    geo_msg.twist.angular.y = 0;
    geo_msg.twist.angular.z = (r/(2*(l+w)))*( - omega[0] + omega[1] - omega[2] + omega[3]);

    Velos_pub.publish(geo_msg);
  
}

// Callbacks

void Velocities::joint_message_update_Callback(const sensor_msgs::JointState::ConstPtr& jointmsg){

  // Exctract Datas from sensor_msgs/JointState

  tpresec = tnowsec;
  tprensec = tnownsec;
  tnowsec = jointmsg->header.stamp.sec;
  tnownsec = jointmsg->header.stamp.nsec;
  deltat = (tnowsec - tpresec)+(tnownsec-tprensec)*0.000000001;

  //Computation of the Wheel's speeds

  // Legenda of the order of the motors:
  // - front-left(fl):  motor 0
  // - front-right(fr): motor 1
  // - rear-left(rl):   motor 2
  // - rear-right(rr):  motor 3

  for (int i = 0; i<4; i++){
    pospre[i] = posnow[i];
    posnow[i] = jointmsg->position[i];
    deltapos[i] = posnow[i] - pospre[i];
    omega[i] = (deltapos[i]/deltat)*(2*3.14159/(T*N));      //PI used as approssimation: 3.14159

  } 
  
}


void Velocities::robot_parameters_callback(robottino::robot_parametersConfig &config, uint32_t level) {
  r = config.r;
  l = config.l;
  w = config.w;
  N = config.N;
}


// Main Loop

void Velocities::main_loop() {

  // Initialization of the message and its index
  int geo_msg_index = 0;
  

  //Initialization of the parameters and the 

  dynamic_reconfigure::Server<robottino::robot_parametersConfig> robot_param_Server;        //Initialization of the server
  dynamic_reconfigure::Server<robottino::robot_parametersConfig>::CallbackType f;           //Initialization of the server's callback
  
  f = boost::bind(&Velocities::robot_parameters_callback, this, _1, _2);                            
  robot_param_Server.setCallback(f);


  ros::Rate loop_rate(freq);

  //Main working loop

  while (ros::ok()) {       

    //Update of the Message
    geo_msg_writting(geo_msg_index);
    geo_msg_index++;

    //Cycle operations
    ros::spinOnce();
    loop_rate.sleep();

  }
}