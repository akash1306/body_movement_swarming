#ifndef GEST_CONT
#define GEST_CONT

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mrs_lib/param_loader.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <math.h>
#include <body_movement_swarming/IntStamped.h>
#include <std_srvs/Trigger.h>

class GestureControlClass
{
    public:
        GestureControlClass(ros::NodeHandle* nodehandle);
        void servicestarter();
        bool start_trig;
        bool land_trig;
        std::string _uav_name_;
        ros::ServiceClient srv_ehover;
        ros::ServiceClient srv_eland;

    private: 
    nav_msgs::Odometry odomdata;
    ros::Subscriber gps_subscriber;
    ros::ServiceServer srv_land_trigger;
    ros::ServiceServer srv_server_trigger;

    boost::array<double,4> goal = {0.0,0.0,0.0,0.0};
    ros::Subscriber gesture_sub;
    body_movement_swarming::IntStamped filtered_gesture;

    ros::NodeHandle nh;

    void odomCallback(const nav_msgs::Odometry& message_holder); 
    void gestureCallback(const body_movement_swarming::IntStamped& ros_data);
    bool servicetrigcallback(std_srvs::Trigger::Request& 
                                        req, std_srvs::Trigger::Response& res);
    bool servicelandcallback(std_srvs::Trigger::Request& 
                                        req, std_srvs::Trigger::Response& res);
    


};

#endif