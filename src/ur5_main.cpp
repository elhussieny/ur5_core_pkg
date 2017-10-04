/*
 * ur5_main.cpp
 *
 *  Created on: Sep 27, 2017
 *      Author: haitham
 */
#include<iostream>
#include "ur5_core_pkg/UR5Core.h"
//********************************************************************************************************//
   void GUICallbackPose(ur5_core_pkg::UR5GUIConfig &config, uint32_t level, UR5Core* ur5obj ) {
	  printf("Pose Request: (%.3f, %.3f, %.3f),  (%.3f, %.3f, %.3f; %.3f) \n",
	            config.ur5_ee_x, config.ur5_ee_y, config.ur5_ee_z,
				config.ur5_ee_qx, config.ur5_ee_qy, config.ur5_ee_qz, config.ur5_ee_qw);

	  geometry_msgs::Pose testPose;
	  testPose.position.x = config.ur5_ee_x;
	  testPose.position.y = config.ur5_ee_y;
	  testPose.position.z = config.ur5_ee_z;
	  testPose.orientation.x = config.ur5_ee_qx;
	  testPose.orientation.y = config.ur5_ee_qy;
	  testPose.orientation.z = config.ur5_ee_qz;
	  testPose.orientation.w =config.ur5_ee_qw;
	  if(ur5obj->inverse(testPose))
		  ur5obj->sendJointCommands(ur5obj->xJoint);
	  else
	  printf("Bad \n");
	}
//---------------------------------------------------------------------------------------

int main( int argc, char** argv )
{
ros::init(argc, argv, "ur5_core");
UR5Core ur5_robot;
printf("main 0\n");




dynamic_reconfigure::Server<ur5_core_pkg::UR5GUIConfig> server;
	    dynamic_reconfigure::Server<ur5_core_pkg::UR5GUIConfig>::CallbackType ur5;
	    ur5 =  boost::bind(&GUICallbackPose, _1, _2,&ur5_robot);
	    server.setCallback(ur5);

	while(ros::ok())ros::spin();
}

