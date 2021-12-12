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

// Custom Message
#include <body_movement_swarming/landmark.h>
#include <body_movement_swarming/IntStamped.h>


class ClassifierGestureClass
{
    public:
        ClassifierGestureClass(ros::NodeHandle* nodehandle); 


    private:
        ros::Subscriber landmark_subscriber;
        ros::Subscriber raw_gesture_pub;
        int gesture_buffer[200]; 
        double header_buffer[200];
        int buffer_index;
        int gesture_number;
        double current_time;
        int zero_state_counter;
        int one_state_counter;
        int two_state_counter;
        int last_seq;
        body_movement_swarming::IntStamped filtered_gesture;

        ros::NodeHandle nh;
    
        void Callback(const body_movement_swarming::IntStamped& ros_data);
        body_movement_swarming::IntStamped incoming_gesture;

        
        void                callbackTimer(const ros::TimerEvent& event);
        ros::Timer          timer_pub_gesture;
        ros::Publisher      gesture_pub;
};

#endif