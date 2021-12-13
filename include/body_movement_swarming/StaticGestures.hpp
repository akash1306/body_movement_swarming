#ifndef FORMATION_H
#define FORMATION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>

#include <mrs_msgs/ReferenceStampedSrv.h>

#include <std_srvs/Trigger.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<std_msgs/Int32.h>


#include <string>
#include<stdio.h>
#include <algorithm>
#include<math.h>
#include <stdlib.h> 
#include<vector>

// Custom Message
#include <body_movement_swarming/landmark.h>
#include <body_movement_swarming/IntStamped.h>
#define PI 3.14159265

class StaticGestureClass
{
    public:
        StaticGestureClass(ros::NodeHandle* nodehandle); 

    private:
        ros::Subscriber landmarksub;
        ros::NodeHandle nh;
        int last_seq;

        void Callback(const body_movement_swarming::landmark& ros_data);
        body_movement_swarming::landmark incoming_landmarks;
        body_movement_swarming::IntStamped gesture_outgoing;
        

        ros::Publisher ges_pub;
        double CalculateAngle(double firstx, double firsty, 
                                double secondx, double secondy,
                                double thirdx, double thirdy);




};


#endif