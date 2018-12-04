#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/JointState.h"
#include "visual_servo_test/CartesianPose.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    ros::Publisher fake_joint_pub = nh.advertise<sensor_msgs::JointState>("fake_joint_states", 1);
    ros::Publisher fake_cartesian_pose_pub = nh.advertise<visual_servo_test::CartesianPose>("fake_cartesian_diff_pose",1);

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform transform;

    ros::Rate loop_rate(10);

    double t=0;
    ROS_INFO("get here");
    while (ros::ok())
    {
        transform.setOrigin(tf::Vector3(-0.2, 0.0, 0.5));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/link7", "/desire_pose"));

        sensor_msgs::JointState fakeJointStates;
        fakeJointStates.position.push_back(1);
        fake_joint_pub.publish(fakeJointStates);

        visual_servo_test::CartesianPose fakeCartesianDiffPose;
        fakeCartesianDiffPose.x = 0.1*sin(t*3.14159);
        fakeCartesianDiffPose.y = 0.1*cos(t*3.14159);
        fakeCartesianDiffPose.z = 0;
        fakeCartesianDiffPose.roll = 0;
        fakeCartesianDiffPose.pitch = 0;
        fakeCartesianDiffPose.yaw = 0;
        fake_cartesian_pose_pub.publish(fakeCartesianDiffPose);


        ros::spinOnce();

        loop_rate.sleep();
        t=t+0.01;
    }

    return 0;
}
