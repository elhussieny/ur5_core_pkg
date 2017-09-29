/*
 * UR5Core.h
 *
 *  Created on: Sep 27, 2017
 *      Author: haitham
 */

#ifndef UR5_CORE_PKG_INCLUDE_UR5CORE_H_
#define UR5_CORE_PKG_INCLUDE_UR5CORE_H_

#include <ros/ros.h>
#include <math.h>
#include "std_msgs/String.h"
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>


#define PI 3.14159265359

using namespace std;

//-------------------------------------------------------------------------------------------------//
class UR5Core {

private:
	float UR5Joints[7];
		// DH Parameters for the UR5. Refer to png file
		double D1 = 0.08916;
		double D3 = 0.425;
		double D4 = 0.10915;
		double D5 = 0.39225;
		double D7 = 0.09456;
		ros::Publisher UR5JointPublisher[6];

		const double UR5JointLimits[7] = {
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0};


public:
		std_msgs::Float64 xJoint[7];

	UR5Core();
	bool findInverseKinematics(const geometry_msgs::Pose & desiredPose);
	bool findInverseKinematics2(const geometry_msgs::Pose & desiredPose);

	void sendJointCommands(std_msgs::Float64* xJoint);


};




#endif /* UR5_CORE_PKG_INCLUDE_UR5CORE_H_ */
