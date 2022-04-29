#include<GestureController.hpp>

GestureControlClass::GestureControlClass(ros::NodeHandle* nodehandle):nh(*
                                                                    nodehandle)
{
    ROS_INFO("In Controller");
    start_trig = false;
    land_trig = false;
    
    mrs_lib::ParamLoader param_loader(nh, "GestureController");
    param_loader.loadParam("/GestureController/uav_name", _uav_name_);
    gps_subscriber = nh.subscribe(_uav_name_ + "/odometry/odom_gps",1, 
                                    &GestureControlClass::odomCallback, this);
    gesture_sub = nh.subscribe(_uav_name_ + "/gesture_filtered",1, 
                                &GestureControlClass::gestureCallback, this);

    srv_server_trigger = nh.advertiseService(_uav_name_ + "/start_action_movement", 
                                    &GestureControlClass::servicetrigcallback, 
                                    this);

    srv_land_trigger = nh.advertiseService(_uav_name_ + "/end_experiment", 
                                    &GestureControlClass::servicelandcallback, 
                                    this);
    srv_ehover = nh.serviceClient<std_srvs::Trigger>(_uav_name_ + "/control_manager/ehover");
    srv_eland = nh.serviceClient<std_srvs::Trigger>(_uav_name_ + "/control_manager/eland");
    
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

bool GestureControlClass::servicelandcallback([[maybe_unused]] std_srvs::Trigger::Request& 
                                        req, std_srvs::Trigger::Response& res)
{

    ROS_INFO("Ending Experiment");
    res.message = "Ending Experiment";
    res.success = true;
    land_trig = true;
    return true;
    
}
void GestureControlClass::odomCallback(const nav_msgs::Odometry& message_holder)
{
    odomdata = message_holder;
}

void GestureControlClass::gestureCallback(const 
                                body_movement_swarming::IntStamped& ros_data)
{
    if (!start_trig)
      return;
    filtered_gesture = ros_data;
    servicestarter();
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
    ROS_INFO("Command Sent");
    std::cout<<filtered_gesture.int_data<<std::endl;
    std::cout<<srv1.response<<std::endl;
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
    ros::service::waitForService(gestureObject._uav_name_ + "/control_manager/goto",10); 
    while (ros::ok()){
        ROS_INFO_ONCE("Waiting for Start Trigger");
        
        if (gestureObject.start_trig)
        {
            
            break;
        }
        
        ros::spinOnce();
        r.sleep();

    }
    std_srvs::Trigger trig_var;
    while (ros::ok()){
        if (gestureObject.land_trig){
            gestureObject.srv_ehover.call(trig_var);
            ROS_INFO_ONCE("Triggering Hover");
            ros::Rate rl(0.2);
            rl.sleep();
            gestureObject.srv_eland.call(trig_var);
            ROS_INFO_ONCE("Triggering Landing");
            return 0;
        }
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 
