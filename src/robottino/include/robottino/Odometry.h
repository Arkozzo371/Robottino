// To avoid multiple definitions of the class
#ifndef ODOMETRY_H
#define ODOMETRY_H


//Standard include for Ros
#include "ros/ros.h"

//Utility Inludes
#include <sstream>

//Message includes
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h" 
#include <geometry_msgs/TransformStamped.h>

//Kinematic tree includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>                   

//Service include
#include "robottino/Pose_Reset.h"

//Dynamic Configuration includes
#include <dynamic_reconfigure/server.h>                 
#include <robottino/Integration_parametersConfig.h> 



class Odometry {
  public:

    // Constructor
    Odometry(); 

    //Main loop
    void main_loop();

    // Functions

    void odom_msg_writting(int &index);
    void tf_msg_writting();
    void compute_odometry();

    // Callbacks

    void geo_message_update_Callback(const geometry_msgs::TwistStamped::ConstPtr& geo_msg);
    void integrator_parameters_callback(robottino::Integration_parametersConfig &config, uint32_t level);
    bool pose_reset_callback(robottino::Pose_Reset::Request  &req, robottino::Pose_Reset::Response &res);


  private:

  //Node Handler and All subs and pubs

    ros::NodeHandle Odometry_Node_Handler; 
    ros::Subscriber Velos_sub;
    ros::Publisher Integrator_pub;

    tf2_ros::TransformBroadcaster Odometry_Broadcaster;
    
  //Reset Service Call

    ros::ServiceServer Pose_Reset_Service;

  //Message Declaration

    int na_msg_index;
    nav_msgs::Odometry na_msg;

    geometry_msgs::TransformStamped odomTransform;


  // Parameters

    enum Integration_method_type{ Euler, Rk };

    Integration_method_type Integration_method;

    int freq;


  // Coordinates 

    double x;
    double y;
    double theta;

    double x_prec;
    double y_prec;
    double theta_prec;

    double v_x;
    double v_y;
    double omega;


  // Time

    double tnowsec;
    double tnownsec;
    double tpresec;
    double tprensec;
    double deltat;
    

};

#endif