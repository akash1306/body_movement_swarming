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


#include <string>
#include<stdio.h>
#include <algorithm>

// Custom Message
#include <body_movement_swarming/landmark.h>


class ClassifierGestureClass
{
    public:
        ClassifierGestureClass(ros::NodeHandle* nodehandle); 


    private:
        ros::Subscriber landmark_subscriber;
        int gesture_buffer[200]; 
        double header_buffer[200];
        int buffer_index;
        int gesture_number;
        double current_time;
        int zero_state_counter;
        int one_state_counter;
        int two_state_counter;
        int gesture_to_publish;

        ros::NodeHandle nh;
        
        void landmarkCallback(const body_movement_swarming::landmark& ros_data);
        body_movement_swarming::landmark landmark_data;


        void                callbackTimer(const ros::TimerEvent& event);
        ros::Timer          timer_pub_gesture;
        ros::Publisher      gesture_pub;
};

#endif