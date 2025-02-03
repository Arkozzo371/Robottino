//Standard include for Ros
#include "ros/ros.h"

//Utility Inludes
#include <sstream>

//Message includes
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/TransformStamped.h>

//Class definition File include
#include "robottino/Odometry.h"   

//Kinematic tree includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>                   

//Service include
#include "robottino/Pose_Reset.h"

//Dynamic Configuration includes
#include <dynamic_reconfigure/server.h>                 
#include <robottino/Integration_parametersConfig.h>   


Odometry::Odometry() { // class constructor

  //initializations of the sub and pub nodes 

  Velos_sub = Odometry_Node_Handler.subscribe("/cmd_vel", 1000, &Odometry::geo_message_update_Callback, this);
  Integrator_pub = Odometry_Node_Handler.advertise<nav_msgs::Odometry>("/odom", 1000);
  
  //initialization of the reset service

  Pose_Reset_Service = Odometry_Node_Handler.advertiseService("Pose_Reset", &Odometry::pose_reset_callback, this);

  //Initialization of integration param

  Integration_method = Euler;

  //Initialization coordintes as Static Parameters
  Odometry_Node_Handler.getParam("/x0", x);
  Odometry_Node_Handler.getParam("/y0", y);
  Odometry_Node_Handler.getParam("/theta0", theta);
  Odometry_Node_Handler.getParam("/freq", freq);

}


// Functions

void Odometry::odom_msg_writting(int &index){

// Header and References of the Coordinates
    
  na_msg.header.seq = index;
  na_msg.header.stamp.sec = tnowsec;
  na_msg.header.stamp.nsec = tnownsec;
  na_msg.header.frame_id = "odom";
  na_msg.child_frame_id = "base_link";

// Pose part of the Message

  na_msg.pose.pose.position.x = x;
  na_msg.pose.pose.position.y = y;
  na_msg.pose.pose.position.z = 0;

  // Quaternion Generation
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);

  na_msg.pose.pose.orientation.x = q.x();
  na_msg.pose.pose.orientation.y = q.y();
  na_msg.pose.pose.orientation.z = q.z();
  na_msg.pose.pose.orientation.w = q.w();

//Velocties part of the message

  na_msg.twist.twist.linear.x = v_x;
  na_msg.twist.twist.linear.y = v_y;
  na_msg.twist.twist.linear.z = 0;
  na_msg.twist.twist.angular.x = 0;
  na_msg.twist.twist.angular.y = 0;
  na_msg.twist.twist.angular.z = omega;

  Integrator_pub.publish(na_msg);
  
}


void Odometry::tf_msg_writting(){

// Header and References of the Coordinates

  odomTransform.header.stamp.sec = tnowsec;
  odomTransform.header.stamp.nsec = tnownsec;
  odomTransform.header.frame_id = "world";
  odomTransform.child_frame_id = "base_link";

// Translation part of the Message

  odomTransform.transform.translation.x = x;
  odomTransform.transform.translation.y = y;
  odomTransform.transform.translation.z = 0;

  // Quaternion Generation
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);

// Rotation part of the Message

  odomTransform.transform.rotation.x = q.x();
  odomTransform.transform.rotation.y = q.y();
  odomTransform.transform.rotation.z = q.z();
  odomTransform.transform.rotation.w = q.w();

  Odometry_Broadcaster.sendTransform(odomTransform);
  
}


void Odometry::compute_odometry(){

  x_prec = x;
  y_prec = y;
  theta_prec = theta;


  switch (Integration_method){

    case Euler:

      x = x_prec + deltat * (v_x * cos(theta_prec) - v_y * sin(theta_prec));
      y = y_prec + deltat * (v_x * sin(theta_prec) + v_y * cos(theta_prec));
      theta = theta_prec + omega * deltat;

      break;

    case Rk: //Rk Middlepoint

      x = x_prec + deltat * (v_x * cos(theta_prec + (deltat * omega)/2) - v_y * sin(theta_prec + (deltat * omega)/2));
      y = y_prec + deltat * (v_x * sin(theta_prec + (deltat * omega)/2) + v_y * cos(theta_prec + (deltat * omega)/2));
      theta = theta_prec + omega * deltat;

      break;

  }
}



// Callbacks

void Odometry::geo_message_update_Callback(const geometry_msgs::TwistStamped::ConstPtr& geo_msg){
 
// Exctract Datas from geometry_msgs/TwistStamped

  tpresec = tnowsec;
  tprensec = tnownsec;
  
  tnowsec = geo_msg->header.stamp.sec;
  tnownsec = geo_msg->header.stamp.nsec;
  deltat = (tnowsec - tpresec)+(tnownsec - tprensec)*0.000000001;

  v_x = geo_msg->twist.linear.x;
  v_y = geo_msg->twist.linear.y;
  omega = geo_msg->twist.angular.z; 


// Update of the Odometry
 
  compute_odometry();
  
}


void Odometry::integrator_parameters_callback(robottino::Integration_parametersConfig &config, uint32_t level){

  switch (config.Integration){

    case 0:

      Integration_method = Euler;

      break;

    case 1:

      Integration_method = Rk;

      break;

  }

}


bool Odometry::pose_reset_callback(robottino::Pose_Reset::Request  &req, robottino::Pose_Reset::Response &res){

//Passing old values

  res.old_x = x;
  res.old_y = y;
  res.old_theta = theta;
  
//Getting new values

  x = req.new_x;
  y = req.new_y;
  theta = req.new_theta;

//A print for control

  ROS_INFO("Position resetted from ( %f, %f, %f) to ( %f, %f, %f)", (float) res.old_x, (float) res.old_y, (float) res.old_theta, (float) req.new_x, (float) req.new_y, (float) req.new_theta);

  return true;

}



// Main Loop

void Odometry::main_loop() {

// Initialization of the message and its index
  na_msg_index = 0;
  
//Initialization of the parameters and the 

  dynamic_reconfigure::Server<robottino::Integration_parametersConfig> Integration_param_Server;        //Initialization of the server
  dynamic_reconfigure::Server<robottino::Integration_parametersConfig>::CallbackType f;           //Initialization of the server's callback
  
  f = boost::bind(&Odometry::integrator_parameters_callback, this, _1, _2);                            
  Integration_param_Server.setCallback(f);


  ros::Rate loop_rate(freq);

//Main working loop

  while (ros::ok()) {   

  //Publishing of the Odometry message 

    odom_msg_writting(na_msg_index);
    na_msg_index++;

  //Publishing of the TF message 

    tf_msg_writting();  

  //Cycle operations

    ros::spinOnce();
    loop_rate.sleep();

  }
}