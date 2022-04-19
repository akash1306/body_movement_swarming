#include<GestureController.hpp>

GestureControlClass::GestureControlClass(ros::NodeHandle* nodehandle):nh(*
                                                                    nodehandle)
{
    ROS_INFO("In Controller");
    start_trig = false;
    
    mrs_lib::ParamLoader param_loader(nh, "GestureController");
    param_loader.loadParam("/GestureController/uav_name", _uav_name_);
    gps_subscriber = nh.subscribe(_uav_name_ + "/odometry/odom_gps",1, 
                                    &GestureControlClass::odomCallback, this);
    gesture_sub = nh.subscribe(_uav_name_ + "/gesture_filtered",1, 
                                &GestureControlClass::gestureCallback, this);

    srv_server_trigger = nh.advertiseService(_uav_name_ + "/start_action_movement", 
                                    &GestureControlClass::servicetrigcallback, 
                                    this);

}

bool GestureControlClass::servicetrigcallback([[maybe_unused]] std_srvs::Trigger::Request& 
                                        req, std_srvs::Trigger::Response& res)
{
    ROS_INFO("Started Body Movement Swarming");
    res.message = "Started Body movement Swarming";
    res.success = true;
    start_trig = true;
    return true;
    
}

void GestureControlClass::odomCallback(const nav_msgs::Odometry& message_holder)
{
    odomdata = message_holder;
}

void GestureControlClass::gestureCallback(const 
                                body_movement_swarming::IntStamped& ros_data)
{
    filtered_gesture = ros_data;
}

void GestureControlClass::servicestarter()
{
    ros::ServiceClient client1 = nh.serviceClient<mrs_msgs::Vec4>(
                                                _uav_name_ + "/control_manager/goto");

    if(filtered_gesture.int_data == 1)
    {
        goal[0] = odomdata.pose.pose.position.x;
        goal[1] = odomdata.pose.pose.position.y + 1.0;
        goal[2] = odomdata.pose.pose.position.z;
    }
    if(filtered_gesture.int_data == 2)
    {
        goal[0] = odomdata.pose.pose.position.x;
        goal[1] = odomdata.pose.pose.position.y - 1.0;
        goal[2] = odomdata.pose.pose.position.z;
    }

    if(filtered_gesture.int_data == 0)
    {
        goal[0] = odomdata.pose.pose.position.x;
        goal[1] = odomdata.pose.pose.position.y;
        goal[2] = odomdata.pose.pose.position.z;
    }

    mrs_msgs::Vec4 srv1;
    srv1.request.goal = goal;
    client1.call(srv1);
    filtered_gesture.int_data = 0;

}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "GestureController"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    ROS_INFO("main: instantiating an object of type GestureControlClass");
    GestureControlClass gestureObject(&nh);  //instantiate an UAVClass object and pass in pointer to nodehandle for constructor to use
    ros::Rate r(10);
    while(true){
        ROS_INFO_ONCE("Waiting for Start Trigger");
        
        if (gestureObject.start_trig)
        {
            
            break;
        }
        
        ros::spinOnce();
        r.sleep();

    }
    ros::service::waitForService(gestureObject._uav_name_ + "/control_manager/goto",10); 
    while (ros::ok()){
    
        
        gestureObject.servicestarter();
        
        
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 
