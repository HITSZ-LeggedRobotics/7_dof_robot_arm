#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fixed_target_tf_publisher");
    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform fixedTargetTransform;
    tf::StampedTransform transformTargetToBase;
    ros::Rate loop_rate(10);
    int getTarget = 0;

    while (ros::ok())
    {

        if(!getTarget){
            try {
                listener.waitForTransform("/base_link","/desire_pose", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("/base_link","/desire_pose", ros::Time(0), transformTargetToBase);
                ROS_INFO("catch a visual target transform");
                fixedTargetTransform.setOrigin(transformTargetToBase.getOrigin());
                fixedTargetTransform.setRotation(transformTargetToBase.getRotation());
                getTarget = 1;
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
        //        ros::Duration(1.0).sleep();
//                return 0;
            }
        }
        if(getTarget)
            broadcaster.sendTransform(tf::StampedTransform(transformTargetToBase, ros::Time::now(),"/base_link","/visual_target"));




        loop_rate.sleep();

    }

    return 0;
}
