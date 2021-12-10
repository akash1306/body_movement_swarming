#include<ClassifierGesture.hpp>

using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

ClassifierGestureClass::ClassifierGestureClass(ros::NodeHandle* nodehandle):
                                                                nh(*nodehandle){
    ROS_INFO("Classifier Class Engaged");
    landmark_subscriber = nh.subscribe("/mediapipe/image_raw", 50, 
                            &ClassifierGestureClass::landmarkCallback, this);
    timer_pub_gesture = nh.createTimer(ros::Rate(60.0), 
                                &ClassifierGestureClass::callbackTimer, this);

    debugsub = nh.subscribe("/uav1/gesture_output", 50, 
                            &ClassifierGestureClass::Callback, this);

    gesture_pub = nh.advertise<std_msgs::Int32>("/uav1/gesture_filtered", 1);

    buffer_index = 0;
    std::fill(std::begin(gesture_buffer), std::begin(gesture_buffer), -1);


}

void ClassifierGestureClass::Callback(const std_msgs::Int32& ros_data)
{
    tempdata = ros_data;
}

void ClassifierGestureClass::landmarkCallback(
                            const body_movement_swarming::landmark& ros_data){
    landmark_data = ros_data;
}

void ClassifierGestureClass::callbackTimer(const ros::TimerEvent& event){
    

    zero_state_counter = 0;
    one_state_counter = 0;
    two_state_counter = 0;
    current_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.0;
    // current_time = landmark_data.header.stamp.sec + 
    //                                 landmark_data.header.stamp.nsec / pow(10,9);
    gesture_number = tempdata.data;
    if (buffer_index >0 && current_time - header_buffer[buffer_index - 1]>1)
    {
        std::fill(std::begin(gesture_buffer), std::begin(gesture_buffer), -1);
        buffer_index = 0;
    }
    if(buffer_index < 199)
    {
        header_buffer[buffer_index] = current_time;
        gesture_buffer[buffer_index] = gesture_number;
        buffer_index++;
    }
    else if(buffer_index == 199)
    {
            memmove(&header_buffer[0], &header_buffer[1], 
                                            199*sizeof(header_buffer[0]));
            memmove(&gesture_buffer[0], &gesture_buffer[1], 
                                            199*sizeof(gesture_buffer[0]));

            header_buffer[buffer_index] = current_time;
            gesture_buffer[buffer_index] = gesture_number;
            
    }


    for(int i=199; i>=0; i--)
    {
        if(header_buffer[199] - header_buffer[i]>2.0)
        {
            break;
        }
        

        if(gesture_buffer[i] == 1)
        {
            one_state_counter++;
        }
        if(gesture_buffer[i] == 2)
        {
            two_state_counter++;
        }
        if(gesture_buffer[i] == 0)
        {
            zero_state_counter++;
        }
    }
    std::cout<<header_buffer[199] - header_buffer[0]<<std::endl;
    if(one_state_counter > two_state_counter && (float)one_state_counter/(one_state_counter+two_state_counter+zero_state_counter) >0.9)
    {
        gesture_to_publish.data = 1;
    }
    else if(two_state_counter > one_state_counter && (float)two_state_counter/(one_state_counter+two_state_counter+zero_state_counter) >0.9)
    {
        gesture_to_publish.data = 2;
    }
    else
    {
        gesture_to_publish.data = 0;
    }

    gesture_pub.publish(gesture_to_publish);
    // tempdata.data = 0;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "GestureClassifier");
    ros::NodeHandle nh;
    ROS_INFO("Lets See");
    ClassifierGestureClass classifier_object(&nh);
    ros::spin();
    return 0;
}