#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include "ros/ros.h"
#include "boost/thread.hpp"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "global.h"
#include "stdio.h"
#include "stdlib.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Eigen"
#include "dynamic_reconfigure/server.h"
#include "visual_servo_test/visual_servo_testConfig.h"
#include "visual_servo_test/CartesianPose.h"
#include "std_msgs/Bool.h"
namespace servo_control {

class ServoControl
{
public:

    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    ServoControl(ros::NodeHandle& nodeHandle, tf::TransformListener& tf, dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig>& server);

    /*!
     * Destructor.
     */
    virtual ~ServoControl();

    /*!
     * Callback function for get model states from gazebo and pub as a absolute locate.
     * @param modelStates is a gazebo message containing position and orientation.
     */
    void readParameters();
    void initializes();
    int getLatestCartesianPose();
    int getLatestJointAngle();
    void jointFeedbackCallback(const sensor_msgs::JointState::ConstPtr& jointFeedbackMsg);
    void cartesianDiffPoseCallback(const visual_servo_test::CartesianPose::ConstPtr& cartesianPoseMsg);
    void reconfigureCallback(visual_servo_test::visual_servo_testConfig &config, uint32_t level);
    int getTransformTargetToCamera(ros::Time& timestamp, float (&fTempMatrix4)[4][4]);
    int homing();
    void homingCommandCallback(const std_msgs::Bool::ConstPtr& isHomingMsg);
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& isEmergencyStopMsg);
    void resetCommandCallback(const std_msgs::Bool::ConstPtr& resetFlagMsg);
    void directJointCommandCallback(const sensor_msgs::JointState::ConstPtr& directJointCommandMsg);
    void transformFixedTargetToBaseSend();
private:

    //! subscribe loop thread
    void servoLoopThread();

    //! ROS nodehandle
    ros::NodeHandle& nodeHandle_;

    //! ROS subscriber
    ros::Subscriber endEffectorPoseSub_, cartesianDiffPoseSub_, jointFeedbackSub_, homingCommandSub_, emergencyStopSub_,resetCommandSub_,directJointCommandSub_;

    //! message filter subscriber
    //message_filters::Subscriber<gazebo_msgs::ModelStates> timeSeqSub_;

    //! ROS publisher
    ros::Publisher jointCommandPub_;

    //! pose
//    geometry_msgs::PoseWithCovarianceStamped fakePoseMsg_;

    //
    sensor_msgs::JointState jointCommand;
    ros::Time timeStamp_;
    //! TF boardcaster
//    tf::TransformBroadcaster tfBoardcaster_;
    tf::TransformListener& visualServoTransformListener_;
    tf::StampedTransform targetToCameraTransform_;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform fixedTargetTransform;
    tf::StampedTransform transformTargetToBase;

    Eigen::Quaterniond q;
    //! boost thread
    boost::thread servoLoopThread_;
// dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_
    dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig>& server_;
//    dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig>::CallbackType callbackFunction;

    unsigned char iCapFlag;
    float fTempMatrix4_[4][4]={
            {1.0000,   0.0000 ,   0.0000,    0.2342},
            {0.0000,   1.0000 ,  0.0000 ,  -0.0000},
            {0.0000 ,   0.0000 ,   1.0000,    0.3054},
            {0,0,0,1.0000}
    };
    double initialJointAngle[7] = {0,PI/6,0,-PI/3,0,-PI/3,0};
    float fPosedeltaX[6]={0.01, 0.01, 0.01, 0, 0, PI/6};
    float fPosedeltaXBuffer[6]={0.01, 0.01, 0.01, 0, 0, PI/6};
    float f07matrix4[4][4],f0tmatrix4[4][4],f70matrix4[4][4];
    float CurrentJntAngle[JNT_NUM]={0*PI/180.0, 90*PI/180.0,  0*PI/180.0, -90*PI/180,  0*PI/180,   -90*PI/180, 0*PI/180};
    float CurrentJntAngleBuffer[JNT_NUM]={0*PI/180.0, 90*PI/180.0,  0*PI/180.0, -90*PI/180,  0*PI/180,   -90*PI/180, 0*PI/180};

    float CurrentJntRate[JNT_NUM];
    float fJntAngleDesired[JNT_NUM];
    float fJntRateDesired[JNT_NUM];
    float fJntAccDesired[JNT_NUM];
    bool operationMode, isVisualServo_, isManualControl_;
    int visualServoFlag_ = 0;
    int remoteControlFlag_ = 0;
    int controlFrequences_;
    int isHoming_,isEmergencyStop,resetFlag,directJointMoveFlag_;

};

}/* namespace */
#endif // SERVOCONTROL_H



