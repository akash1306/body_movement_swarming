#include<GestureController.hpp>

GestureControlClass::GestureControlClass(ros::NodeHandle* nodehandle):nh(*
                                                                    nodehandle)
{
    ROS_INFO("In Controller");
    gps_subscriber = nh.subscribe("/uav1/odometry/odom_gps",1, 
                                    &GestureControlClass::odomCallback, this);
    gesture_sub = nh.subscribe("/uav1/gesture_filtered",1, 
                                &GestureControlClass::gestureCallback, this);
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
                                                "/uav1/control_manager/goto");

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

}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "GestureController"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type GestureControlClass");
    GestureControlClass gestureObject(&nh);  //instantiate an UAVClass object and pass in pointer to nodehandle for constructor to use

    ros::Rate r(10);
    while (ros::ok()){
    ros::service::waitForService("/uav1/control_manager/goto",10); 
    gestureObject.servicestarter();
    
    
    ros::spinOnce();
    r.sleep();
    }

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 