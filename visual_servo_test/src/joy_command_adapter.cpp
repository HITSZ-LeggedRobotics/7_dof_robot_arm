#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "servocontrol.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "visual_servo_test/visual_servo_testConfig.h"
#include "dynamic_reconfigure/client.h"

class TeleopTurtle
{
public:
    TeleopTurtle(ros::NodeHandle& nodehandle);
    void publishCommand();
    void publishDirectJointCommand();
    int directJointFlag_;
    int enable_visual, change_config;
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_, resetFlag_;
  double l_scale_, a_scale_, dx_, dy_, dz_, droll_, dpitch_, dyaw_;
  visual_servo_test::CartesianPose diffPoseMsg_;
//  visual_servo_test::visual_servo_testConfig config_;
  sensor_msgs::JointState directJointCommandMsg_;
//  dynamic_reconfigure::Client<visual_servo_test::visual_servo_testConfig> client("servo_control_node");
  ros::Publisher vel_pub_, cartesianDiffPub_,jointCommandPub_, homingCommandPub_, emergencyStopPub_, resetCommandPub_;
  ros::Subscriber joy_sub_;


};


TeleopTurtle::TeleopTurtle(ros::NodeHandle& nodehandle):
  directJointFlag_(0),
  nh_(nodehandle),
  linear_(1),
  angular_(2),
  resetFlag_(0)
{

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, 1.0);
    nh_.param("scale_linear", l_scale_, 1.0);
    ROS_INFO("augular scale is %f, linear scale is %f",a_scale_,l_scale_);

    for(int i = 0;i<4;i++)
        directJointCommandMsg_.position.push_back(0);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    cartesianDiffPub_ = nh_.advertise<visual_servo_test::CartesianPose>("/cartesian_diff_pose", 1);
    homingCommandPub_ = nh_.advertise<std_msgs::Bool>("/homing_cmd",1);
    emergencyStopPub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop",1);
    resetCommandPub_ = nh_.advertise<std_msgs::Bool>("/reset_flag",1);
    jointCommandPub_ = nh_.advertise<sensor_msgs::JointState>("/direct_joint_command",1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
//    visual_servo_test::CartesianPose diffPoseMsg;
    twist.angular.z = a_scale_*joy->axes[angular_];
    twist.linear.x = l_scale_*joy->axes[linear_];
    vel_pub_.publish(twist);

    float leftStickLR = joy->axes[0];//contiunus 1~-1
    float leftStickUD = joy->axes[1];//contiunus 1~-1
    float LT = 1 - joy->axes[2];//contiunus 0~2
    float rightStickLR = joy->axes[3];//contiunus 1~-1
    float rightStickUD = joy->axes[4];//contiunus 1~-1
    float RT = 1 - joy->axes[5];//contiunus 0~2
    float crossKeyLR = joy->axes[6];//1/-1
    float crossKeyUD = joy->axes[7];//1/-1
    int keyA = joy->buttons[0];//0/1
    int keyB = joy->buttons[1];//0/1
    int keyX = joy->buttons[2];//0/1
    int keyY = joy->buttons[3];//0/1
    int LB = joy->buttons[4];//0/1
    int RB = joy->buttons[5];//0/1
    int keyBack = joy->buttons[6];//0/1
    int keyStart = joy->buttons[7];//0/1
    int keyPower = joy->buttons[8];//0/1

    float velocityScale = l_scale_* 0.001*RT + 0.0005f;
    float angularScale = a_scale_* 0.01*LT + 0.001f;
    if(keyStart == 1)
    {
        ROS_INFO("Reset");
        std_msgs::Bool resetFlag;
        resetFlag.data = true;
        resetCommandPub_.publish(resetFlag);
        resetFlag_ = 1;
        enable_visual = 0;
    }

    if(keyPower == 1)
    {
        ROS_INFO("Emergency Stop");
        std_msgs::Bool stopmsg;
        stopmsg.data = true;
        emergencyStopPub_.publish(stopmsg);
        diffPoseMsg_.x = 0;
        diffPoseMsg_.y = 0;
        diffPoseMsg_.z = 0;
        diffPoseMsg_.roll = 0;
        diffPoseMsg_.pitch = 0;
        diffPoseMsg_.yaw = 0;
        resetFlag_ = 0;

        return;
    }
    if(keyBack == 1 && resetFlag_==1)
    {
        ROS_INFO("Homing");
        std_msgs::Bool homingMsg;
        homingMsg.data = true;
        homingCommandPub_.publish(homingMsg);
        resetFlag_ = 0;
        return;
    }
    if(crossKeyUD>0 && resetFlag_==1)
    {
//        change_config = 1;
        enable_visual = 1;
    }
    if(crossKeyUD<0 && resetFlag_==1)
    {
//        change_config = 1;
        enable_visual = 0;
    }
    if(LB==1 && resetFlag_==1 && RB!=1)//control x, y, z
    {
        dx_ = velocityScale*leftStickLR;
        dy_ = velocityScale*leftStickUD;
        dz_ = velocityScale*rightStickUD;
        ROS_INFO("move dx = %f, dy = %f, dz = %f",dx_,dy_,dz_);
    }

    if(RB==1 && resetFlag_==1 && LB!=1)//control roll, pitch, yaw
    {
        droll_ = angularScale*leftStickLR;
        dpitch_ = angularScale*leftStickUD;
        dyaw_ = angularScale*rightStickUD;
        ROS_INFO("move droll = %f, dpitch = %f, dyaw = %f",droll_,dpitch_,dyaw_);
    }
    directJointFlag_ = 0;
    if(RB==1 && resetFlag_ == 1 && LB==1)
    {
        if(keyA)
            directJointCommandMsg_.position[0] = leftStickLR*angularScale;
        if(keyB)
            directJointCommandMsg_.position[1] = leftStickLR*angularScale;
        if(keyX)
            directJointCommandMsg_.position[2] = leftStickLR*angularScale;
        if(keyY)
            directJointCommandMsg_.position[3] = leftStickLR*angularScale;
        directJointFlag_ = 1;
        ROS_INFO("move single joint");
    }

    if(LB == 0 && RB == 0)
    {
        dx_ = 0;
        dy_ = 0;
        dz_ = 0;
        droll_ = 0;
        dpitch_ = 0;
        dyaw_ = 0;
    }
    diffPoseMsg_.x = dx_;
    diffPoseMsg_.y = dy_;
    diffPoseMsg_.z = dz_;
    diffPoseMsg_.roll = droll_;
    diffPoseMsg_.pitch = dpitch_;
    diffPoseMsg_.yaw = dyaw_;
}

void TeleopTurtle::publishCommand()
{
    cartesianDiffPub_.publish(diffPoseMsg_);
}

void TeleopTurtle::publishDirectJointCommand()
{
    jointCommandPub_.publish(directJointCommandMsg_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_command_adapter_node");
    ros::NodeHandle nh("~");
    TeleopTurtle teleop_turtle(nh);
      visual_servo_test::visual_servo_testConfig config_;
//      sensor_msgs::JointState directJointCommandMsg_;
      dynamic_reconfigure::Client<visual_servo_test::visual_servo_testConfig> client_("servo_control_node");
      int changeFlag = 1;
    ros::Rate rate(50);
    while (ros::ok()) {
        teleop_turtle.publishCommand();
        if(teleop_turtle.directJointFlag_)
            teleop_turtle.publishDirectJointCommand();
        if(teleop_turtle.enable_visual==1&&changeFlag==1)
        {
            ROS_INFO("visual Servo");
            config_.use_visual_servo = true;
            config_.manual_contro = false;
            client_.setConfiguration(config_);
            changeFlag = 0;
        }
        if(teleop_turtle.enable_visual==0&&changeFlag==0)
        {
            ROS_INFO("manual Control");
            config_.use_visual_servo = false;
            config_.manual_contro = true;
            client_.setConfiguration(config_);
            changeFlag = 1;
        }
        rate.sleep();
        ros::spinOnce();
    }

}
