# 7_dof_robot_arm
7 Dof Robot Arm for Gazebo Simulation
# Launch Gazebo and Rviz
roslaunch arm_hand arm_hand_empty_world.launch
publish Sensor_msgs/Jointstates name "/joint_command" to control position of 7 joints
