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
#include <stdio.h>
#include "std_msgs/String.h"
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include <ur5_core_pkg/UR5GUIConfig.h>
#include <boost/thread.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>


using namespace boost;

#define PI 3.14159265359

using namespace std;
using namespace Eigen;

//-------------------------------------------------------------------------------------------------//
class UR5Core {

private:
	float UR5Joints[7];
		// DH Parameters for the UR5. Refer to png file
    const double d0 =  0.0;
    const double d1 =  0.089159;
    const double d2 =  0;
    const double d3 =  0;
    const double d4 =  -0.10915;
    const double d5 =  0.09465;
    const double d6 =  -0.0823;

    const double a0 =  0;
    const double a1 =  0;
    const double a2 = 0.42500;
    const double a3 = 0.39225;
    const double a4 =  0;
    const double a5 =  0;
    const double a6 =  0;

    const double alpha0 =  0;
    const double alpha1 =  PI/2;
    const double alpha2 =  0;
    const double alpha3 =  0;
    const double alpha4 =  PI/2;
    const double alpha5 = -PI/2;
    const double alpha6 =  PI;


    const double ZERO_THRESH = 0.00000001;


		ros::Publisher UR5JointPublisher[6];
		ros::Subscriber ur5_pose_subscriber;
		const double UR5JointLimits[7] = {
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0,
		175.0*PI / 180.0};
		int SIGN(double x) {
		            return (x > 0) - (x < 0);
		}

		double wrapTo2PI(double angle);
		Eigen::MatrixXd findDHTable(double* q);
		Eigen::Matrix4d findHTransform(double theta, double d, double a, double alpha);

public:
		std_msgs::Float64 xJoint[7];

	UR5Core();
	bool findInverseKinematics(const geometry_msgs::Pose & desiredPose);
	bool findInverseKinematics2(const geometry_msgs::Pose & desiredPose);
    bool inverse(const geometry_msgs::Pose & desiredPose);
	void sendJointCommands(std_msgs::Float64* xJoint);
	void UR5DesiredPoseCallback(const geometry_msgs::PosePtr& intendedPose);



};




#endif /* UR5_CORE_PKG_INCLUDE_UR5CORE_H_ */
