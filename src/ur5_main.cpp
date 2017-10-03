/*
 * ur5_main.cpp
 *
 *  Created on: Sep 27, 2017
 *      Author: haitham
 */
#include<iostream>
#include "ur5_core_pkg/UR5Core.h"

int main( int argc, char** argv )
{
ros::init(argc, argv, "ur5_core");
UR5Core ur5_robot;
printf("main 0\n");

geometry_msgs::Pose testPose;
testPose.position.x = 0.4;
testPose.position.y = 0.4;
testPose.position.z = 0.1;
testPose.orientation.x = 0;
testPose.orientation.y = 0;
testPose.orientation.z = 0;
testPose.orientation.w =1;

if(ur5_robot.inverse(testPose)>0)
	ur5_robot.sendJointCommands(ur5_robot.xJoint);
else
	printf("Bad \n");

	while(ros::ok())ros::spin();
}

