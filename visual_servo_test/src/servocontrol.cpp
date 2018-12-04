#include "servocontrol.h"

namespace servo_control {
using namespace std;

ServoControl::ServoControl(ros::NodeHandle& nodehandle, tf::TransformListener& tf, dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig>& server)
    : nodeHandle_(nodehandle),
        visualServoTransformListener_(tf),
        server_(server),
      isHoming_(0),
    isEmergencyStop(1),
  resetFlag(0),
directJointMoveFlag_(0)
{

    readParameters();
    initializes();
//    endEffectorPoseSub_ = nodeHandle_.subscribe("end_effector_pose", 1, desiredPoseCallback);
    cartesianDiffPoseSub_ = nodeHandle_.subscribe("/cartesian_diff_pose",1,&ServoControl::cartesianDiffPoseCallback, this);
    jointFeedbackSub_ = nodeHandle_.subscribe("/joint_states", 1, &ServoControl::jointFeedbackCallback, this);
    homingCommandSub_ = nodeHandle_.subscribe("/homing_cmd",1,&ServoControl::homingCommandCallback, this);
    emergencyStopSub_ = nodeHandle_.subscribe("/emergency_stop",1,&ServoControl::emergencyStopCallback,this);
    resetCommandSub_ = nodeHandle_.subscribe("/reset_flag",1,&ServoControl::resetCommandCallback,this);
    directJointCommandSub_ = nodeHandle_.subscribe("/direct_joint_command",1,&ServoControl::directJointCommandCallback,this);
    jointCommandPub_ = nodeHandle_.advertise<sensor_msgs::JointState>("/joint_command",1);

//    dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig> server;
//    dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig>::CallbackType callbackFunction;

    dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig>::CallbackType callbackFunction =
            boost::bind(&ServoControl::reconfigureCallback, this, _1, _2);
    server.setCallback(callbackFunction);

    servoLoopThread_ = boost::thread(boost::bind(&ServoControl::servoLoopThread, this));

}

ServoControl::~ServoControl()
{
    ROS_INFO("Shut down servo control");
}

void ServoControl::readParameters()
{
    nodeHandle_.param("control_frequence", controlFrequences_, 50);
}

void ServoControl::initializes()
{
    for(int i = 0;i<7;i++)
        jointCommand.position.push_back(0);
    if(getLatestJointAngle())
    {
        Rbt_fkine(CurrentJntAngle,D_H,f07matrix4);
        Rbt_MulMtrx(4, 4, 4, f07matrix4[0], fTempMatrix4_[0], f0tmatrix4[0]);
        ROS_INFO("initialized");
    }
}

void ServoControl::servoLoopThread()
{
    ros::Rate rate(controlFrequences_);

    while(ros::ok())
    {
        int i = 0;
//        if(getLatestJointAngle())
//            ROS_INFO("updating latest Joint Angles");
        while(isEmergencyStop){
            ROS_INFO("Emergency STOP");
            ros::Duration(0.02).sleep();
            if(resetFlag){
                isEmergencyStop = 0;
                resetFlag = 0;
                break;
            }
        }

        while(directJointMoveFlag_){
            ROS_INFO("Move Joint directly");
            getLatestJointAngle();
            ros::Duration(0.02).sleep();
            jointCommandPub_.publish(jointCommand);
            if(resetFlag||isEmergencyStop){
                directJointMoveFlag_ = 0;
                resetFlag = 0;
                break;
            }
        }
        if(isHoming_){
            ROS_INFO("Start Homing");
            while(!homing());
            isHoming_ = 0;
            ROS_INFO("End Homing");
            resetFlag = 0;
        }
        if(isVisualServo_)
        {
            transformFixedTargetToBaseSend();
            if(resetFlag){
                timeStamp_ = ros::Time(0);
                if(getTransformTargetToCamera(timeStamp_, fTempMatrix4_))
                {
                    transformFixedTargetToBaseSend();
                    visualServoFlag_ = 1;
                    if(getLatestJointAngle())
                        ROS_INFO("updating latest Joint Angles j1 = %f j2 = %f j3 = %f j4 = %f j5 = %f j6 = %f j7 = %f ",
                                 CurrentJntAngle[0],CurrentJntAngle[1],CurrentJntAngle[2],CurrentJntAngle[3],
                                 CurrentJntAngle[4],CurrentJntAngle[5],CurrentJntAngle[6]);
                        Rbt_fkine(CurrentJntAngle,D_H,f07matrix4);
                        Rbt_MulMtrx(4, 4, 4, f07matrix4[0], fTempMatrix4_[0], f0tmatrix4[0]);
                }
                resetFlag = 0;
            }
        }
        if(isManualControl_)
        {
            ROS_INFO("wait for updaing a cartesian pose");
            if(getLatestCartesianPose())
            {
                remoteControlFlag_ = 1;
                ROS_INFO("updated a new cartesian pose diff with dx = %f, dy = %f,"
                         "dz = %f, droll = %f, dpitch = %f, dyaw = %f", fPosedeltaX[0],
                        fPosedeltaX[1],fPosedeltaX[2],fPosedeltaX[3],fPosedeltaX[4],
                        fPosedeltaX[5]);
                if(getLatestJointAngle()){
                    ROS_INFO("updating latest Joint Angles j1 = %f j2 = %f j3 = %f j4 = %f j5 = %f j6 = %f j7 = %f ",
                             CurrentJntAngle[0],CurrentJntAngle[1],CurrentJntAngle[2],CurrentJntAngle[3],
                             CurrentJntAngle[4],CurrentJntAngle[5],CurrentJntAngle[6]);
                }
             }
         }
        if(isVisualServo_&&visualServoFlag_==1){
            ROS_INFO("Start visual servo");
            while (!nfVisualServoPlanPBVS(fTempMatrix4_, CurrentJntAngle, CurrentJntRate, fJntAngleDesired, fJntRateDesired, fJntAccDesired)) {
//                iCapFlag=nfVisualServoPlanPBVS(fTempMatrix4_, CurrentJntAngle, CurrentJntRate, fJntAngleDesired, fJntRateDesired, fJntAccDesired);
                transformFixedTargetToBaseSend();
                for(i=0;i<JNT_NUM;i++)
                    CurrentJntAngle[i]=fJntAngleDesired[i];
                Rbt_fkine(CurrentJntAngle,D_H,f07matrix4);
                Rbt_InvMtrx( f07matrix4[0], f70matrix4[0], 4 );
                Rbt_MulMtrx(4, 4, 4, f70matrix4[0], f0tmatrix4[0], fTempMatrix4_[0]);
                for(i=0;i<JNT_NUM;i++)
                {
                    printf("%f ",fJntAngleDesired[i]*180/PI);
                    jointCommand.position[i]=fJntAngleDesired[i];
                }
                printf("\n");
                jointCommandPub_.publish(jointCommand);
                rate.sleep();
            }
            ROS_INFO("reach a target pose once");
//            iCapFlag = 0;
            visualServoFlag_ = 0;
        }
        if(isManualControl_&&remoteControlFlag_==1)
        {
            ROS_INFO("Move to manual give pose");
//            float fPosedeltaX[6]={0.01, 0.01, 0.01, 0, 0, PI/6};
            if(nfremotecontrol(fPosedeltaX, CurrentJntAngle,fJntAngleDesired))
            {
                ROS_INFO("Calculate desired Joint Angles");
                for(i=0;i<JNT_NUM;i++)
                    {
                        printf("%f ",fJntAngleDesired[i]*180/PI);
                        jointCommand.position[i]=fJntAngleDesired[i];
                    }
                    printf("\n");
                    jointCommandPub_.publish(jointCommand);
            }
            else{
                ROS_WARN("Joint Angle Exceed the Limits");
            }
            remoteControlFlag_ = 0;
        }
        rate.sleep();
    }
    return;
}

void ServoControl::cartesianDiffPoseCallback(const visual_servo_test::CartesianPose::ConstPtr& cartesianPoseMsg)
{
    fPosedeltaXBuffer[0] = static_cast<float>(cartesianPoseMsg->x);
    fPosedeltaXBuffer[1] = static_cast<float>(cartesianPoseMsg->y);
    fPosedeltaXBuffer[2] = static_cast<float>(cartesianPoseMsg->z);
    fPosedeltaXBuffer[3] = static_cast<float>(cartesianPoseMsg->yaw);
    fPosedeltaXBuffer[4] = static_cast<float>(cartesianPoseMsg->pitch);
    fPosedeltaXBuffer[5] = static_cast<float>(cartesianPoseMsg->roll);

}

void ServoControl::jointFeedbackCallback(const sensor_msgs::JointState::ConstPtr& jointFeedbackMsg)
{
    CurrentJntAngleBuffer[0] = static_cast<float>(jointFeedbackMsg->position[0]);
    CurrentJntAngleBuffer[1] = static_cast<float>(jointFeedbackMsg->position[1]);
    CurrentJntAngleBuffer[2] = static_cast<float>(jointFeedbackMsg->position[2]);
    CurrentJntAngleBuffer[3] = static_cast<float>(jointFeedbackMsg->position[3]);
    CurrentJntAngleBuffer[4] = static_cast<float>(jointFeedbackMsg->position[4]);
    CurrentJntAngleBuffer[5] = static_cast<float>(jointFeedbackMsg->position[5]);
    CurrentJntAngleBuffer[6] = static_cast<float>(jointFeedbackMsg->position[6]);
}

int ServoControl::getLatestJointAngle()
{
    for(int i =0;i<7;i++)
        CurrentJntAngle[i] = CurrentJntAngleBuffer[i];
    return 1;
}

int ServoControl::getLatestCartesianPose()
{
    for(int i =0;i<6;i++)
        fPosedeltaX[i] = fPosedeltaXBuffer[i];
    return 1;
}

void ServoControl::reconfigureCallback(visual_servo_test::visual_servo_testConfig &config, uint32_t level)
{
    operationMode = config.bool_param;
    isVisualServo_ = config.use_visual_servo;
    isManualControl_ = config.manual_contro;
    if(isVisualServo_) ROS_INFO("Using Visual Servo");
    if(isManualControl_) ROS_INFO("Using Maunal control");

}

int ServoControl::getTransformTargetToCamera(ros::Time& timeStamp, float (&fTempMatrix4)[4][4])
{
//    Eigen::Quaterniond q;

    try {
        visualServoTransformListener_.waitForTransform("/link7", "/desire_pose",timeStamp, ros::Duration(1.0));
        visualServoTransformListener_.lookupTransform("/link7","/desire_pose", timeStamp, targetToCameraTransform_);
        ROS_INFO("catch a visual target transform");
//        Eigen::Quaterniond q;
        int a =q.x();
        double px = targetToCameraTransform_.getOrigin().x();
        double py = targetToCameraTransform_.getOrigin().y();
        double pz = targetToCameraTransform_.getOrigin().z();
        q.w() = targetToCameraTransform_.getRotation().w();
        q.x() = targetToCameraTransform_.getRotation().x();
        q.y() = targetToCameraTransform_.getRotation().y();
        q.z() = targetToCameraTransform_.getRotation().z();
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        cout<<"px = "<<px<<"py = "<<py<<"pz = "<<pz<<endl;
        cout<<"Rotation matrix is "<<endl<<R<<endl;
        for(int k=0;k<3;k++){
            for(int j=0;j<3;j++)
                fTempMatrix4[k][j] = static_cast<float>(R(k,j));
        }
        fTempMatrix4[0][3] = px;
        fTempMatrix4[1][3] = py;
        fTempMatrix4[2][3] = pz;
        fTempMatrix4[3][3] = 1;
        ROS_INFO("update a visual target transform");
//        if(getLatestJointAngle())
//            ROS_INFO("updating latest Joint Angles");
//        if(initialFlag==0)
//        {
//            initialFlag = 1;
//            Rbt_fkine(CurrentJntAngle,D_H,f07matrix4);
//            Rbt_MulMtrx(4, 4, 4, f07matrix4[0], fTempMatrix4[0], f0tmatrix4[0]);
//        }
        return 1;

    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
//        ros::Duration(1.0).sleep();
        return 0;
    }
}
void ServoControl::transformFixedTargetToBaseSend()
{
    if(resetFlag){
        try {
            listener.waitForTransform("/base_link","/desire_pose", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/base_link","/desire_pose", ros::Time(0), transformTargetToBase);
            ROS_INFO("catch a visual target transform");
            fixedTargetTransform.setOrigin(transformTargetToBase.getOrigin());
            fixedTargetTransform.setRotation(transformTargetToBase.getRotation());
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
    //        ros::Duration(1.0).sleep();
    //                return 0;
        }
    }
    broadcaster.sendTransform(tf::StampedTransform(transformTargetToBase, ros::Time::now(),"/base_link","/visual_target"));

}

void ServoControl::homingCommandCallback(const std_msgs::Bool::ConstPtr& isHomingMsg)
{
    isHoming_ = 0;
    if(isHomingMsg->data)
        isHoming_ = 1;
}
int ServoControl::homing()
{
    int homingFlag =0;
    getLatestJointAngle();
    while (!homingFlag) {
        for(int i = 0;i<250;i++){
            ros::Duration(0.02).sleep();
            for(int j = 0;j<7;j++)
                jointCommand.position[j] = CurrentJntAngle[j] + (initialJointAngle[j] - CurrentJntAngle[j])*i/250;
            jointCommandPub_.publish(jointCommand);
            if(isEmergencyStop)
                i = 250;
        }
        homingFlag = 1;
    }
    return 1;
}
void ServoControl::emergencyStopCallback(const std_msgs::Bool::ConstPtr& isEmergencyStopMsg)
{
    isEmergencyStop = 0;
    if(isEmergencyStopMsg->data)
        isEmergencyStop = 1;
}

void ServoControl::resetCommandCallback(const std_msgs::Bool::ConstPtr& resetFlagMsg)
{
    resetFlag = 0;
    if(resetFlagMsg->data)
        resetFlag = 1;
}

void ServoControl::directJointCommandCallback(const sensor_msgs::JointState::ConstPtr& directJointCommandMsg)
{
    directJointMoveFlag_ = 1;
    for(int j = 0;j<4;j++)
        jointCommand.position[j] = CurrentJntAngle[j] + directJointCommandMsg->position[j];
//    jointCommandPub_.publish(jointCommand);
}


}
