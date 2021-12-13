#include<ClassifierGesture.hpp>



ClassifierGestureClass::ClassifierGestureClass(ros::NodeHandle* nodehandle):
                                                                nh(*nodehandle){
    ROS_INFO("Classifier Class Engaged");

    timer_pub_gesture = nh.createTimer(ros::Rate(60.0), 
                                &ClassifierGestureClass::callbackTimer, this);

    raw_gesture_sub = nh.subscribe("/uav1/raw_gesture", 50, 
                            &ClassifierGestureClass::Callback, this);

    gesture_pub = nh.advertise<body_movement_swarming::IntStamped>(
                                                "/uav1/gesture_filtered", 1);

    buffer_index = 0;
    last_seq = -1;
    std::fill(std::begin(gesture_buffer), std::begin(gesture_buffer), -1);
    std::fill(std::begin(header_buffer), std::begin(header_buffer), -1);


}

void ClassifierGestureClass::Callback(const body_movement_swarming::IntStamped&
                                                                     ros_data)
{
    incoming_gesture = ros_data;
}

void ClassifierGestureClass::callbackTimer(const ros::TimerEvent& event){
    
    if(incoming_gesture.header.seq == last_seq)
    {
        return;
    }
    filtered_gesture.header.stamp = incoming_gesture.header.stamp;
    filtered_gesture.header.frame_id = incoming_gesture.header.frame_id;
    last_seq = incoming_gesture.header.seq;
    zero_state_counter = 0;
    one_state_counter = 0;
    two_state_counter = 0;
    // current_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.0;
    current_time = incoming_gesture.header.stamp.sec + 
                        incoming_gesture.header.stamp.nsec / (float)pow(10,9);
    if(buffer_index>0)
    {
        std::cout<<current_time - header_buffer[0]<<std::endl;
    }
    
    gesture_number = incoming_gesture.int_data;

    if (buffer_index >0 && current_time - header_buffer[buffer_index - 1]>1)
    {
        std::fill(std::begin(gesture_buffer), std::begin(gesture_buffer), -1);
        std::fill(std::begin(header_buffer), std::begin(header_buffer), -1);
        buffer_index = 0;
    }
    if(buffer_index < 99)
    {
        header_buffer[buffer_index] = current_time;
        gesture_buffer[buffer_index] = gesture_number;
        buffer_index++;
    }
    else if(buffer_index == 99)
    {
            memmove(&header_buffer[0], &header_buffer[1], 
                                            99*sizeof(header_buffer[0]));
            memmove(&gesture_buffer[0], &gesture_buffer[1], 
                                            99*sizeof(gesture_buffer[0]));

            header_buffer[buffer_index] = current_time;
            gesture_buffer[buffer_index] = gesture_number;
            
    }


    for(int i=99; i>=0; i--)
    {
        if(header_buffer[99] - header_buffer[i]>1.0)
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

    if(one_state_counter > two_state_counter && (float)one_state_counter/(
                one_state_counter+two_state_counter+zero_state_counter) >0.9)
    {
        filtered_gesture.int_data = 1;
    }
    else if(two_state_counter > one_state_counter && (float)two_state_counter/(
                one_state_counter+two_state_counter+zero_state_counter) >0.9)
    {
        filtered_gesture.int_data = 2;
    }
    else
    {
        filtered_gesture.int_data = 0;
    }

    gesture_pub.publish(filtered_gesture);
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