#include<ClassifierGesture.hpp>

ClassifierGestureClass::ClassifierGestureClass(ros::NodeHandle* nodehandle):
                                                                nh(*nodehandle){
    ROS_INFO("Classifier Class Engaged");
    landmark_subscriber = nh.subscribe("/mediapipe/image_raw", 100, 
                            &ClassifierGestureClass::landmarkCallback, this);
    timer_pub_gesture = nh.createTimer(ros::Rate(50.0), 
                                &ClassifierGestureClass::callbackTimer, this);


}

void ClassifierGestureClass::landmarkCallback(
                            const body_movement_swarming::landmark& ros_data){
    landmark_data = ros_data;
}

void ClassifierGestureClass::callbackTimer(const ros::TimerEvent& event){
    
}
