#include "global.h"
#include "stdio.h"
#include "stdlib.h"
#include "ros/ros.h"
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

using namespace std;

unsigned char iCapFlag;
float fTempMatrix4[4][4]={
	{1.0000,   0.0000 ,   0.0000,    0.2342},
	{0.0000,   1.0000 ,  0.0000 ,  -0.0000},
	{0.0000 ,   0.0000 ,   1.0000,    0.3054},
	{0,0,0,1.0000}
};
float fPosedeltaX[6]={0.01, 0.01, 0.01, 0, 0, PI/6};
float fPosedeltaXBuffer[6]={0.01, 0.01, 0.01, 0, 0, PI/6};
float f07matrix4[4][4],f0tmatrix4[4][4],f70matrix4[4][4];
float CurrentJntAngle[JNT_NUM]={0*PI/180.0, 90*PI/180.0,  0*PI/180.0, -90*PI/180,  0*PI/180,   -90*PI/180, 0*PI/180};
float CurrentJntAngleBuffer[JNT_NUM]={0*PI/180.0, 90*PI/180.0,  0*PI/180.0, -90*PI/180,  0*PI/180,   -90*PI/180, 0*PI/180};

float CurrentJntRate[JNT_NUM];
float fJntAngleDesired[JNT_NUM];
float fJntRateDesired[JNT_NUM];
float fJntAccDesired[JNT_NUM];
bool operationMode, isVisualServo, isManualControl;
int visualServoFlag = 0;
int remoteControlFlag = 0;

void reconfigureCallback(visual_servo_test::visual_servo_testConfig &config, uint32_t level)
{
    operationMode = config.bool_param;
    isVisualServo = config.use_visual_servo;
    isManualControl = config.manual_contro;
    if(isVisualServo) ROS_INFO("Using Visual Servo");
    if(isManualControl) ROS_INFO("Using Maunal control");

}

void cartesianDiffPoseCallback(const visual_servo_test::CartesianPoseConstPtr& cartesianPoseMsg)
{
    fPosedeltaXBuffer[0] = static_cast<float>(cartesianPoseMsg->x);
    fPosedeltaXBuffer[1] = static_cast<float>(cartesianPoseMsg->y);
    fPosedeltaXBuffer[2] = static_cast<float>(cartesianPoseMsg->z);
    fPosedeltaXBuffer[3] = static_cast<float>(cartesianPoseMsg->roll);
    fPosedeltaXBuffer[4] = static_cast<float>(cartesianPoseMsg->pitch);
    fPosedeltaXBuffer[5] = static_cast<float>(cartesianPoseMsg->yaw);

}

void jointFeedbackCallback(const sensor_msgs::JointStateConstPtr& jointFeedbackMsg)
{
    CurrentJntAngleBuffer[0] = static_cast<float>(jointFeedbackMsg->position[0]);
    CurrentJntAngleBuffer[1] = static_cast<float>(jointFeedbackMsg->position[1]);
    CurrentJntAngleBuffer[2] = static_cast<float>(jointFeedbackMsg->position[2]);
    CurrentJntAngleBuffer[3] = static_cast<float>(jointFeedbackMsg->position[3]);
    CurrentJntAngleBuffer[4] = static_cast<float>(jointFeedbackMsg->position[4]);
    CurrentJntAngleBuffer[5] = static_cast<float>(jointFeedbackMsg->position[5]);
    CurrentJntAngleBuffer[6] = static_cast<float>(jointFeedbackMsg->position[6]);
}

int getLatestJointAngle()
{
    for(int i =0;i<7;i++)
        CurrentJntAngle[i] = CurrentJntAngleBuffer[i];
    return 1;
}

int getLatestCartesianPose()
{
    for(int i =0;i<6;i++)
        fPosedeltaX[i] = fPosedeltaXBuffer[i];
    return 1;
}

void desiredPoseCallback(const geometry_msgs::TransformStampedConstPtr endEffectorPoseMsg)
{
    fTempMatrix4[0][3] = endEffectorPoseMsg->transform.translation.x;
    tf::Transform transform;

}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"visual_servo_node");
    ros::NodeHandle nh_;
    //ros::Subscriber jointFeedbackSub = nh_.subscribe("jointstate_feedback", 1, jointFeedbackCallback);
    ros::Subscriber endEffectorPoseSub = nh_.subscribe("end_effector_pose", 1, desiredPoseCallback);
    ros::Subscriber cartesianDiffPoseSub = nh_.subscribe("cartesian_diff_pose",1,cartesianDiffPoseCallback);
    ros::Publisher jointCommandPub = nh_.advertise<sensor_msgs::JointState>("joint_command",1);


    dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig> server;
    dynamic_reconfigure::Server<visual_servo_test::visual_servo_testConfig>::CallbackType callbackFunction;
    callbackFunction = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(callbackFunction);

    sensor_msgs::JointState jointCommand;

    tf::TransformListener desireTransformOfEndListener, visualServoTransformListener;
    tf::StampedTransform desireTransform, targetToCameraTransform;
    Eigen::Quaterniond q;
//    tfScalar* matrixOneRow;
//    int visualServoFlag = 0;
//    int remoteControlFlag = 0;
    int initialFlag = 0;
    int i;
//    Rbt_fkine(CurrentJntAngle,D_H,f07matrix4);
//    Rbt_MulMtrx(4, 4, 4, f07matrix4[0], fTempMatrix4[0], f0tmatrix4[0]);
    ros::Rate rate(10);
    while(ros::ok())
	{
        if(isVisualServo)
        {
            try {
                visualServoTransformListener.lookupTransform("/desire_pose","/end_effector", ros::Time(0), targetToCameraTransform);
                ROS_INFO("catch a visual target transform");
                double px = targetToCameraTransform.getOrigin().x();
                double py = targetToCameraTransform.getOrigin().y();
                double pz = targetToCameraTransform.getOrigin().z();
                q.w() = targetToCameraTransform.getRotation().w();
                q.x() = targetToCameraTransform.getRotation().x();
                q.y() = targetToCameraTransform.getRotation().y();
                q.z() = targetToCameraTransform.getRotation().z();
                Eigen::Matrix3d R = q.normalized().toRotationMatrix();
                cout<<"Rotation matrix is "<<endl<<R<<endl;

                visualServoFlag = 1;
                for(int k=0;k<3;k++){
                    for(int j=0;j<3;j++)
                        fTempMatrix4[k][j] = static_cast<float>(R(k,j));
                }
                fTempMatrix4[0][3] = px;
                fTempMatrix4[1][3] = py;
                fTempMatrix4[2][3] = pz;
                fTempMatrix4[3][3] = 1;
                ROS_INFO("update a visual target transform");
                if(getLatestJointAngle())
                    ROS_INFO("updating latest Joint Angles");
                if(initialFlag==0)
                {
                    initialFlag = 1;
                    Rbt_fkine(CurrentJntAngle,D_H,f07matrix4);
                    Rbt_MulMtrx(4, 4, 4, f07matrix4[0], fTempMatrix4[0], f0tmatrix4[0]);
                }

            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        if(isManualControl)
        {
            ROS_INFO("wait for updaing a cartesian pose");
            if(getLatestCartesianPose())
            {
                remoteControlFlag = 1;
                ROS_INFO("updated a new cartesian pose diff");
            }
        }

        if(isVisualServo&&visualServoFlag==1){
            ROS_INFO("Start visual servo");
            while (!iCapFlag) {
                iCapFlag=nfVisualServoPlanPBVS(fTempMatrix4, CurrentJntAngle, CurrentJntRate, fJntAngleDesired, fJntRateDesired, fJntAccDesired);
                for(i=0;i<JNT_NUM;i++)
                    CurrentJntAngle[i]=fJntAngleDesired[i];
                Rbt_fkine(CurrentJntAngle,D_H,f07matrix4);
                Rbt_InvMtrx( f07matrix4[0], f70matrix4[0], 4 );
                Rbt_MulMtrx(4, 4, 4, f70matrix4[0], f0tmatrix4[0], fTempMatrix4[0]);
                for(i=0;i<JNT_NUM;i++)
                {
                    printf("%f ",fJntAngleDesired[i]*180/PI);
                    jointCommand.position.push_back(fJntAngleDesired[i]);
                }
                printf("\n");
                jointCommandPub.publish(jointCommand);
            }
            ROS_INFO("reach a target pose once");
            iCapFlag = 0;
            visualServoFlag = 0;
        }
        if(isManualControl&&remoteControlFlag==1)
        {
            ROS_INFO("Move to manual give pose");
//            float fPosedeltaX[6]={0.01, 0.01, 0.01, 0, 0, PI/6};
            if(nfremotecontrol(fPosedeltaX, CurrentJntAngle,fJntAngleDesired))
            {
                ROS_INFO("Calculate desired Joint Angles");
                for(i=0;i<JNT_NUM;i++)
                    {
                        printf("%f ",fJntAngleDesired[i]*180/PI);
                    }
                    printf("\n%d\n",iCapFlag);
            }
            else{
                ROS_WARN("Joint Angle Exceed the Limits");
            }
            remoteControlFlag = 0;
        }
        ros::spinOnce();
        rate.sleep();
	}
    return 0;
//	system("pause");
}
