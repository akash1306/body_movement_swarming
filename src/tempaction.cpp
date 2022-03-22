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

class TempActionClass
{
    public:
        TempActionClass(ros::NodeHandle* nodehandle); 

    private:
        ros::Subscriber landmarksub;
        ros::NodeHandle nh;
        int last_seq;
        bool left_ver;
        bool left_hor ;
        bool left_mid;
        bool right_ver;
        bool right_hor;
        bool right_mid;

        void Callback(const body_movement_swarming::landmark& ros_data);
        body_movement_swarming::landmark incoming_landmarks;
        body_movement_swarming::IntStamped gesture_outgoing;
        

        ros::Publisher ges_pub;
        double CalculateAngle(double firstx, double firsty, 
                                double secondx, double secondy,
                                double thirdx, double thirdy);
        
        void                callbackTimer(const ros::TimerEvent& event);
        ros::Timer          timer_pub_gesture;




};




TempActionClass::TempActionClass(ros::NodeHandle* nodehandle):
                                                                nh(*nodehandle)
{
    left_ver = false;
    left_hor = false;
    left_mid = false;
    right_ver = false;
    right_hor = false;
    right_mid = false;
    std::string _uav_name_;
    mrs_lib::ParamLoader param_loader(nh, "TempAction");
    param_loader.loadParam("/TempAction/uav_name", _uav_name_);
    ges_pub = nh.advertise<body_movement_swarming::IntStamped>(
                                                _uav_name_ + "/gesture_filtered", 1);

    landmarksub = nh.subscribe(_uav_name_ + "/landmarkCoord", 50, 
                            &TempActionClass::Callback, this);

    timer_pub_gesture = nh.createTimer(ros::Duration(2), 
                                &TempActionClass::callbackTimer, this);

    last_seq = -1;


} 

void TempActionClass::callbackTimer(const ros::TimerEvent& event)
{
    ges_pub.publish(gesture_outgoing);
    left_ver = false;
    left_hor = false;
    left_mid = false;
    right_ver = false;
    right_hor = false;
    right_mid = false;
    gesture_outgoing.int_data = 0;
}

void TempActionClass::Callback(const body_movement_swarming::landmark& 
                                                                    ros_data)
{
    incoming_landmarks = ros_data;

    if(incoming_landmarks.header.seq == last_seq)
    {
        std::cout <<"Same Seq" <<std::endl;
        return;
    }
    last_seq = incoming_landmarks.header.seq;
    double left_hand_angle = CalculateAngle(incoming_landmarks.x[15],
                                        incoming_landmarks.y[15],
                                        incoming_landmarks.x[11],
                                        incoming_landmarks.y[11],
                                        incoming_landmarks.x[12],
                                        incoming_landmarks.y[12]);

    double right_hand_angle = CalculateAngle(incoming_landmarks.x[11],
                                        incoming_landmarks.y[11],
                                        incoming_landmarks.x[12],
                                        incoming_landmarks.y[12],
                                        incoming_landmarks.x[16],
                                        incoming_landmarks.y[16]);


    if(left_hand_angle>160 && left_hand_angle <205)
    {
        left_hor = true;
    }
    if(left_hand_angle >230 && left_hand_angle<260)
    {
        left_ver = true;
    }
    if(left_hand_angle >205 && left_hand_angle<230)
    {
        left_mid = true;
    }
    if(right_hand_angle>-195 && right_hand_angle<-145)
    {
        right_hor = true;
    }
    if(right_hand_angle > -115 && right_hand_angle < -85)
    {
        right_ver = true;
    }
    if(right_hand_angle > -145 && right_hand_angle < -115)
    {
        right_mid = true;
    }
    bool right_hand = right_hor && right_mid && right_ver;
    bool left_hand = left_hor && left_mid && left_ver;


    if(right_hand && left_hand)
    {
        gesture_outgoing.int_data = 0;
    }
    else if(right_hand)
    {
        gesture_outgoing.int_data = 4;
    }
    else if(left_hand)
    {
        gesture_outgoing.int_data = 2;
    }
    else{
        gesture_outgoing.int_data = 0;
    }

    gesture_outgoing.header.frame_id = incoming_landmarks.header.frame_id;
    gesture_outgoing.header.stamp = incoming_landmarks.header.stamp;
    // ges_pub.publish(gesture_outgoing);

}

double TempActionClass::CalculateAngle(double firstPointx, double firstPointy 
                                ,double midPointx, double midPointy,
                                double endPointx, double endPointy)
{
    double direct_angle = atan2(endPointy - midPointy, endPointx - midPointx) - 
                    atan2(firstPointy - midPointy , firstPointx - midPointx);
    double angle_in_degrees = direct_angle * 180/PI;

    return angle_in_degrees;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TempAction");
    ros::NodeHandle nh;
    ROS_INFO("Lets See");
    TempActionClass tempaction_object(&nh);
    ros::spin();
    return 0;
}
