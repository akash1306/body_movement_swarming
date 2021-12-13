#include<StaticGestures.hpp>



StaticGestureClass::StaticGestureClass(ros::NodeHandle* nodehandle):
                                                                nh(*nodehandle){

    ges_pub = nh.advertise<body_movement_swarming::IntStamped>(
                                                "/uav1/raw_gesture", 1);

    landmarksub = nh.subscribe("/landmarkCoord", 50, 
                            &StaticGestureClass::Callback, this);

    last_seq = -1;

} 

void StaticGestureClass::Callback(const body_movement_swarming::landmark& 
                                                                    ros_data)
{
    incoming_landmarks = ros_data;

    if(incoming_landmarks.header.seq == last_seq)
    {
        return;
    }

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

    bool left_ver = false;
    bool left_hor = false;
    bool right_ver = false;
    bool right_hor = false;
    if(left_hand_angle>160 && left_hand_angle <205)
    {
        left_hor = true;
    }
    if(left_hand_angle >230 && left_hand_angle<260)
    {
        left_ver = true;
    }
    if(right_hand_angle>-195 && right_hand_angle<-145)
    {
        right_hor = true;
    }
    if(right_hand_angle > -115 && right_hand_angle < -85)
    {
        right_ver = true;
    }

    if(right_hor)
    {
        gesture_outgoing.int_data = 1;
    }
    else if(left_hor)
    {
        gesture_outgoing.int_data = 2;
    }
    else{
        gesture_outgoing.int_data = 0;
    }

    gesture_outgoing.header.frame_id = incoming_landmarks.header.frame_id;
    gesture_outgoing.header.stamp = incoming_landmarks.header.stamp;
    ges_pub.publish(gesture_outgoing);
     
    



}

double StaticGestureClass::CalculateAngle(double firstPointx, double firstPointy 
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
    ros::init(argc, argv, "StaticGestures");
    ros::NodeHandle nh;
    ROS_INFO("Lets See");
    StaticGestureClass staticgest_object(&nh);
    ros::spin();
    return 0;
}