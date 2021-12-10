#include<ClassifierGesture.hpp>

ClassifierGestureClass::ClassifierGestureClass(ros::NodeHandle* nodehandle):
                                                                nh(*nodehandle){
    ROS_INFO("Classifier Class Engaged");
    landmark_subscriber = nh.subscribe("/mediapipe/image_raw", 50, 
                            &ClassifierGestureClass::landmarkCallback, this);
    timer_pub_gesture = nh.createTimer(ros::Rate(60.0), 
                                &ClassifierGestureClass::callbackTimer, this);

    buffer_index = 0;
    std::fill(std::begin(gesture_buffer), std::begin(gesture_buffer), -1);


}

void ClassifierGestureClass::landmarkCallback(
                            const body_movement_swarming::landmark& ros_data){
    landmark_data = ros_data;
}

void ClassifierGestureClass::callbackTimer(const ros::TimerEvent& event){
    

    zero_state_counter = 0;
    one_state_counter = 0;
    two_state_counter = 0;
    current_time = landmark_data.header.stamp.sec + 
                                    landmark_data.header.stamp.nsec / pow(10,9);

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

    if(one_state_counter > two_state_counter && (float)one_state_counter/(one_state_counter+two_state_counter+zero_state_counter) >0.9)
    {
        gesture_to_publish = 1;
    }
    else if(two_state_counter > one_state_counter && (float)two_state_counter/(one_state_counter+two_state_counter+zero_state_counter) >0.9)
    {
        gesture_to_publish = 2;
    }
    else
    {
        gesture_to_publish = 0;
    }



}


