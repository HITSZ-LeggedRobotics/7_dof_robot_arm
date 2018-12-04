//#include <ros/ros.h>
#include "servocontrol.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_control_node");
    ros::NodeHandle nodehandle("~");
    tf::TransformListener tf;
    dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig> server;
    servo_control::ServoControl servocontrol(nodehandle,tf,server);
//    fake_pose::FakePose fakepose(nodehandle);

    ROS_INFO("Hello world!");
//    ros::Rate rate(50);
//    while(ros::ok())
//    {
//        ros::spinOnce();
//        rate.sleep();
//    }
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
