/*
 * UR5Core.cpp
 *
 *  Created on: Sep 27, 2017
 *      Author: haitham
 */

#include "ur5_core_pkg/UR5Core.h"

	UR5Core::UR5Core(){	// TODO Auto-generated constructor stub
		ros::NodeHandle nh_;
		char jointTopicString[30];// = "/UR5/joints"; // rostopic that will send the ROS pose
		for(int j=0; j<6; j++)
		{
			sprintf(jointTopicString, "/UR5/joints%d", j+1);
			this->UR5JointPublisher[j] = nh_.advertise<std_msgs::Float64>(jointTopicString,1,false);
		//	this->UR5JointPublisher[j].publish(0);
		}
		printf("------UR5 ROS Interface------\n");
		printf("By: Haitham El-Hussieny \n");

		printf("-----------------------------\n");
//-------------
sleep(1);
	}

//********************************************************************************************************//

	bool UR5Core::findInverseKinematics(const geometry_msgs::Pose & desiredPose){
		 bool output = true;
			// Auxiliary variables
				double mod_pW, mod_pWxy, c2, s2, c3, s3;
					Eigen::Vector3d p(desiredPose.position.x,desiredPose.position.y, desiredPose.position.z-D1);
					Eigen::Quaterniond R = Eigen::Quaterniond(desiredPose.orientation.x,desiredPose.orientation.y,desiredPose.orientation.z,desiredPose.orientation.w);

		            Eigen::Vector3d pW = p-(D7 *R.toRotationMatrix().col(2));
						// Calculate wrist position
		            UR5Joints[0] = -atan2(pW[1], pW[0]);
								mod_pW = pow(pW.norm(),2); // Pwx^2+Pwy^2+Pwz^2
		            	//mod_pW = pow(pW[0],2) + pow(pW[1],2) + pow(pW[2]-D1,2);
							c3 = (mod_pW - D3*D3 - D5*D5)/(2*D3*D5);
							// If c3>1, there is no solution for IKT
							if (c3>1){printf("NOT --- REACHABLE! c=%.3f \n",c3);output= false;
							for(int j=0;j<6;j++) UR5Joints[j]=0;
							return output;
							}
							//
							UR5Joints[2] = 0.0;
								s3 = -sqrt(1 - c3*c3);
								UR5Joints[3] = atan2(s3, c3) + PI / 2;
								//
							//

								mod_pWxy = sqrt(pW[0] * pW[0] + pW[1] * pW[1]);
								s2 = ((D3 + D5*c3)*pW[2] - D5*s3*mod_pWxy) / mod_pW;
								c2 = ((D3 + D5*c3)*mod_pWxy + D5*s3*pW[2]) / mod_pW;
								UR5Joints[1] = atan2(s2, c2);
								//
								//    					// Calculate orientation (angles of the wrist joints)
								Eigen::Matrix3d T01; T01 << cos(UR5Joints[0]), 0.0, sin(UR5Joints[0]), sin(UR5Joints[0]), 0.0, -cos(UR5Joints[0]), 0.0, 1.0, 0.0;
								Eigen::Matrix3d T12; T12 << cos(UR5Joints[1]), -sin(UR5Joints[1]), 0.0, sin(UR5Joints[1]), cos(UR5Joints[1]), 0.0, 0.0, 0.0, 1.0;
								Eigen::Matrix3d T23; T23 << cos(UR5Joints[3]), 0.0, sin(UR5Joints[3]), sin(UR5Joints[3]), 0.0, -cos(UR5Joints[3]), 0.0, 1.0, 0.0;
								//
								Eigen::Matrix3d pose03 = T01*T12*T23;
								Eigen::Matrix3d  pose36 = pose03.inverse() * (R.toRotationMatrix());
								//
								UR5Joints[4] = atan2(pose36(1, 2), pose36(0, 2));
								UR5Joints[5] = atan2(sqrt(pose36(0, 2)*pose36(0, 2) + pose36(1, 2)*pose36(1, 2)), pose36(2, 2));
								UR5Joints[6] = atan2(pose36(2, 1), -pose36(2, 0));
								//
								//    					//Adjust to robot from IK coordinates (keeping joint coord. within the interval [-pi,pi])
								UR5Joints[1] < -PI / 2 ? UR5Joints[1] += 3 * PI / 2 : UR5Joints[1] -= PI / 2;
								UR5Joints[3] < -PI / 2 ? UR5Joints[3] += 3 * PI / 2 : UR5Joints[3] -= PI / 2;
								UR5Joints[6] <     0 ? UR5Joints[6] += PI : UR5Joints[6] -= PI;

								UR5Joints[1] = -UR5Joints[1]; //Correcting for the RobotRotation
								UR5Joints[5] = -UR5Joints[5]; //Correcting for the RobotRotation

								//
								for (int i = 0; i < 7; i++){
									xJoint[i].data =(float) UR5Joints[i]; // for ROS V-REP
									if (fabs(UR5Joints[i]) > UR5JointLimits[i]){
										output = false;
										UR5Joints[i] > 0 ? UR5Joints[i] = UR5JointLimits[i] : UR5Joints[i] = -UR5JointLimits[i];
										printf("Warning!!! IK gives values out of bounds for joint %d \n", i);
									}
									//
									printf("Joint [%d]: %.3f \n", i, UR5Joints[i]);


								}




								return output;
	}
	bool UR5Core::findInverseKinematics2(const geometry_msgs::Pose & desiredPose){
			 bool output = true;
				// Auxiliary variables
					double mod_pW, mod_pWxy, c2, s2, c3, s3, rpW, alpha2;
						Eigen::Vector3d p(desiredPose.position.x,desiredPose.position.y, desiredPose.position.z);
						Eigen::Quaterniond R = Eigen::Quaterniond(desiredPose.orientation.x,desiredPose.orientation.y,desiredPose.orientation.z,desiredPose.orientation.w);
						// 1- Calculate wrist position:
			            Eigen::Vector3d pW = p-(D7 *R.toRotationMatrix().col(2)); // wrist point
			            rpW = sqrt(pow(pW[0],2) + pow(pW[1],2));
			            alpha2 = acos(D4/rpW);
			            UR5Joints[0] = atan2(pW[1], pW[0]) + alpha2 + PI/2;
							/*		mod_pW = pow(pW.norm(),2); // Pwx^2+Pwy^2+Pwz^2
			            	//mod_pW = pow(pW[0],2) + pow(pW[1],2) + pow(pW[2]-D1,2);
								c3 = (mod_pW - D3*D3 - D5*D5)/(2*D3*D5);
								// If c3>1, there is no solution for IKT
								if (c3>1){printf("NOT --- REACHABLE! c=%.3f \n",c3);output= false;
								for(int j=0;j<6;j++) UR5Joints[j]=0;
								return output;
								}
								//
								UR5Joints[2] = 0.0;
									s3 = -sqrt(1 - c3*c3);
									UR5Joints[3] = atan2(s3, c3) + PI / 2;
									//
								//

									mod_pWxy = sqrt(pW[0] * pW[0] + pW[1] * pW[1]);
									s2 = ((D3 + D5*c3)*pW[2] - D5*s3*mod_pWxy) / mod_pW;
									c2 = ((D3 + D5*c3)*mod_pWxy + D5*s3*pW[2]) / mod_pW;
									UR5Joints[1] = atan2(s2, c2);
									//
									//    					// Calculate orientation (angles of the wrist joints)
									Eigen::Matrix3d T01; T01 << cos(UR5Joints[0]), 0.0, sin(UR5Joints[0]), sin(UR5Joints[0]), 0.0, -cos(UR5Joints[0]), 0.0, 1.0, 0.0;
									Eigen::Matrix3d T12; T12 << cos(UR5Joints[1]), -sin(UR5Joints[1]), 0.0, sin(UR5Joints[1]), cos(UR5Joints[1]), 0.0, 0.0, 0.0, 1.0;
									Eigen::Matrix3d T23; T23 << cos(UR5Joints[3]), 0.0, sin(UR5Joints[3]), sin(UR5Joints[3]), 0.0, -cos(UR5Joints[3]), 0.0, 1.0, 0.0;
									//
									Eigen::Matrix3d pose03 = T01*T12*T23;
									Eigen::Matrix3d  pose36 = pose03.inverse() * (R.toRotationMatrix());
									//
									UR5Joints[4] = atan2(pose36(1, 2), pose36(0, 2));
									UR5Joints[5] = atan2(sqrt(pose36(0, 2)*pose36(0, 2) + pose36(1, 2)*pose36(1, 2)), pose36(2, 2));
									UR5Joints[6] = atan2(pose36(2, 1), -pose36(2, 0));
									//
									//    					//Adjust to robot from IK coordinates (keeping joint coord. within the interval [-pi,pi])
									UR5Joints[1] < -PI / 2 ? UR5Joints[1] += 3 * PI / 2 : UR5Joints[1] -= PI / 2;
									UR5Joints[3] < -PI / 2 ? UR5Joints[3] += 3 * PI / 2 : UR5Joints[3] -= PI / 2;
									UR5Joints[6] <     0 ? UR5Joints[6] += PI : UR5Joints[6] -= PI;

									UR5Joints[1] = -UR5Joints[1]; //Correcting for the RobotRotation
									UR5Joints[5] = -UR5Joints[5]; //Correcting for the RobotRotation

									//
									for (int i = 0; i < 7; i++){
										xJoint[i].data =(float) UR5Joints[i]; // for ROS V-REP
										if (fabs(UR5Joints[i]) > UR5JointLimits[i]){
											output = false;
											UR5Joints[i] > 0 ? UR5Joints[i] = UR5JointLimits[i] : UR5Joints[i] = -UR5JointLimits[i];
											printf("Warning!!! IK gives values out of bounds for joint %d \n", i);
										}
										//
										printf("Joint [%d]: %.3f \n", i, UR5Joints[i]);


									}
*/

			            xJoint[0].data =(float) UR5Joints[0];
			            printf("Joint [%d]: %.3f \n", 0, UR5Joints[0]);
									return true;
		}
	//********************************************************************************************************//
void UR5Core::sendJointCommands(std_msgs::Float64* xJoint){
	UR5JointPublisher[0].publish(xJoint[0]);
/*	UR5JointPublisher[1].publish(xJoint[1]);
	UR5JointPublisher[2].publish(xJoint[3]);
	UR5JointPublisher[3].publish(xJoint[4]);
	UR5JointPublisher[4].publish(xJoint[5]);
	UR5JointPublisher[5].publish(xJoint[6]);*/
	printf("Joints Sent Successfully! \n");

}
