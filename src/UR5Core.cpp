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
		this->ur5_pose_subscriber = nh_.subscribe("/UR5/desiredPose",1,&UR5Core::UR5DesiredPoseCallback,this);
sleep(1);
	}


	//********************************************************************************************************//
	void UR5Core::UR5DesiredPoseCallback(const geometry_msgs::PosePtr& intendedPose)
	{

		// For bounded reigon (For Safety)
	/*	intendedPose->position.x = BOUND(intendedPose->position.x,KUKA_X_MIN,KUKA_X_MAX);
		intendedPose->position.y = BOUND(intendedPose->position.y,KUKA_Y_MIN,KUKA_Y_MAX);
		intendedPose->position.z = BOUND(intendedPose->position.z,KUKA_Z_MIN,KUKA_Z_MAX);
	*/
		printf("Pose:[%.3f,%.3f,%.3f] \n",intendedPose->position.x,intendedPose->position.y,intendedPose->position.z);


	}

	bool UR5Core::findInverseKinematics(const geometry_msgs::Pose & desiredPose){
		 bool output = true;
	/*		// Auxiliary variables
				double mod_pW, mod_pWxy, c2, s2, c3, s3;
					Eigen::Vector3d p(desiredPose.position.x,desiredPose.position.y, desiredPose.position.z-D1);
					Eigen::Quaterniond R = Eigen::Quaterniond(desiredPose.orientation.x,desiredPose.orientation.y,desiredPose.orientation.z,desiredPose.orientation.w);

		            Eigen::Vector3d pW = p-(D7 *R.toRotationMatrix().col(2));
						// Calculate wrist position
		            UR5Joints[0] = -atan2(pW[1], pW[0]);
								mod_pW = pow(pW.norm(),2); // Pwx^2+Pwy^2+Pwz^2
		            	//mod_pW = pow(pW[0],2) + pow(pW[1],2) + pow(pW[2]-D1,2);
							c3 = (mod_pW - D3*D3 - D5*D5)/(2*D3*D5);
							// If c3>1, there is no solution  for IKT
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
								return output;
	}

	//********************************************************************************************************//


	bool UR5Core::findInverseKinematics2(const geometry_msgs::Pose & desiredPose){
			 bool output = true;
				// Auxiliary variables
					double mod_pW, mod_pWxy, c2, s2, c3, s3, rpW, alpha2;
						Eigen::Vector3d p(desiredPose.position.y,desiredPose.position.x, desiredPose.position.z);
						Eigen::Quaterniond R = Eigen::Quaterniond(desiredPose.orientation.x,desiredPose.orientation.y,desiredPose.orientation.z,desiredPose.orientation.w);

						// 1- Calculate wrist position:
			            Eigen::Vector3d pW = p-(d6 *R.toRotationMatrix().col(2)); // wrist point
						rpW = sqrt(pow(pW[0],2) + pow(pW[1],2));
						if(d4>pow(rpW,2))
						{
							printf("R = %.3f, D4= %.3f \n",rpW, d4);

							printf("No solution\n");
							return false;
						}

						//2- q1
			            alpha2 = acos(d4/rpW);
			            UR5Joints[0] =(float) (atan2(pW[1], pW[0]) + alpha2 + PI/2);
			            //3- q5
			            UR5Joints[4] = (float)acos(((p[0]*sin(UR5Joints[0]) - (p[1]*cos(UR5Joints[0])) - d4))/d6);
			            //4- q6
			            double t1 = (-R.toRotationMatrix().col(1)[0] * sin(UR5Joints[0])) + (R.toRotationMatrix().col(1)[1] *cos(UR5Joints[0]));
			            double t2 = (-R.toRotationMatrix().col(0)[0] * sin(UR5Joints[0])) + (R.toRotationMatrix().col(0)[1] *cos(UR5Joints[0]));
			            UR5Joints[5] = atan2(SIGN(sin(UR5Joints[4]))*t1, -SIGN(sin(UR5Joints[4]))*t2);


			            double c6 = cos(UR5Joints[5]), s6 = sin(UR5Joints[5]);
			            double c5 = cos(UR5Joints[4]), s5 = sin(UR5Joints[4]);
			            double c1 = cos(UR5Joints[0]), s1 = sin(UR5Joints[0]);

			            double T00 = R.toRotationMatrix().col(0)[0];
			            double T10 = R.toRotationMatrix().col(0)[1];
			            double T20 = R.toRotationMatrix().col(0)[2];

			            double T01 = R.toRotationMatrix().col(1)[0];
			            double T11 = R.toRotationMatrix().col(1)[1];
			            double T21 = R.toRotationMatrix().col(1)[2];

			            double T02 = R.toRotationMatrix().col(2)[0];
			            double T12 = R.toRotationMatrix().col(2)[1];
			            double T22 = R.toRotationMatrix().col(2)[2];

			            double T03 = p[0]; double T13 = p[1]; double T23 = p[2];





			                                double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
			                                double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
			                                double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + T03*c1 + T13*s1;
			                                double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

			                                c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
			                                if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
			                                    c3 = SIGN(c3);
			                                else if(fabs(c3) > 1.0) {
			                                    printf("NO SOLUTION C3 larger than 1 \n");
			                                    return false;
			                                }
			                                double arccos = acos(c3);
			                                UR5Joints[1] = arccos;
			                                double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
			                                s3 = sin(arccos);
			                                double A = (a2 + a3*c3), B = a3*s3;
			                                UR5Joints[2] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
			                              //  q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
			                                double c23_0 = cos(UR5Joints[1]+UR5Joints[2]);
			                                double s23_0 = sin(UR5Joints[1]+UR5Joints[2]);
			                          //      double c23_1 = cos(q2[1]+q3[1]);
			                           //     double s23_1 = sin(q2[1]+q3[1]);
			                                UR5Joints[3] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
			       //     q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);


			                                for (int i = 0; i < 6; i++){
                         										xJoint[i].data =(float) UR5Joints[i]; // for ROS V-REP
                             										if (fabs(UR5Joints[i]) > UR5JointLimits[i]){
                              											output = false;
                              											UR5Joints[i] > 0 ? UR5Joints[i] = UR5JointLimits[i] : UR5Joints[i] = -UR5JointLimits[i];
                       											printf("Warning!!! IK gives values out of bounds for joint %d \n", i);
                             										}
                             										printf("Joint [%d]: %.3f \n", i, UR5Joints[i]);

			                                }
									return true;
		}
	//********************************************************************************************************//


	int UR5Core::inverse(const geometry_msgs::Pose & desiredPose) {

		double mod_pW, mod_pWxy, c2, s2, c3, s3, rpW, alpha2;
		Eigen::Vector3d p(desiredPose.position.x,desiredPose.position.y, desiredPose.position.z);
		Eigen::Quaterniond R = Eigen::Quaterniond(desiredPose.orientation.x,desiredPose.orientation.y,desiredPose.orientation.z,desiredPose.orientation.w);


	        int num_sols = 0;


            double T00 = 1;//R.toRotationMatrix().col(0)[0];
            double T10 = R.toRotationMatrix().col(0)[1];
            double T20 = R.toRotationMatrix().col(0)[2];

            double T01 = R.toRotationMatrix().col(1)[0];
            double T11 = 1;//R.toRotationMatrix().col(1)[1];
            double T21 = R.toRotationMatrix().col(1)[2];

            double T02 = R.toRotationMatrix().col(2)[0];
            double T12 = R.toRotationMatrix().col(2)[1];
            double T22 = 1;//R.toRotationMatrix().col(2)[2];
            double T03 = p[0]; double T13 = p[1]; double T23 = p[2];
            printf("----------------- \n");

            printf("[%.2f, %.2f, %.2f, %.2f] \n",T00, T01, T02, T03 );
            printf("[%.2f, %.2f, %.2f, %.2f] \n",T10, T11, T12, T13 );
            printf("[%.2f, %.2f, %.2f, %.2f] \n",T20, T21, T22, T23 );
            printf("----------------- \n");


	        ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
	        double q1[2];
	        {
	            double A = d6*T12 - T13;
	            double B = d6*T02 - T03;
	            double R = A*A + B*B;
	            if(fabs(A) < ZERO_THRESH) {
	                double div;
	                if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
	                    div = -SIGN(d4)*SIGN(B);
	                else
	                    div = -d4/B;
	                double arcsin = asin(div);
	                if(fabs(arcsin) < ZERO_THRESH)
	                    arcsin = 0.0;
	                if(arcsin < 0.0)
	                    q1[0] = arcsin + 2.0*PI;
	                else
	                    q1[0] = arcsin;
	                q1[1] = PI - arcsin;
	            }
	            else if(fabs(B) < ZERO_THRESH) {
	                double div;
	                if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
	                    div = SIGN(d4)*SIGN(A);
	                else
	                    div = d4/A;
	                double arccos = acos(div);
	                q1[0] = arccos;
	                q1[1] = 2.0*PI - arccos;
	            }
	            else if(d4*d4 > R) {
	                return num_sols;
	            }
	            else {
	                double arccos = acos(d4 / sqrt(R)) ;
	                double arctan = atan2(-B, A);
	                double pos = arccos + arctan;
	                double neg = -arccos + arctan;
	                if(fabs(pos) < ZERO_THRESH)
	                    pos = 0.0;
	                if(fabs(neg) < ZERO_THRESH)
	                    neg = 0.0;
	                if(pos >= 0.0)
	                    q1[0] = pos;
	                else
	                    q1[0] = 2.0*PI + pos;
	                if(neg >= 0.0)
	                    q1[1] = neg;
	                else
	                    q1[1] = 2.0*PI + neg;
	            }
	        }
	        ////////////////////////////////////////////////////////////////////////////////

	        ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
	        double q5[2][2];
	        {
	            for(int i=0;i<2;i++) {
	                double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
	                double div;
	                if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
	                    div = SIGN(numer) * SIGN(d6);
	                else
	                    div = numer / d6;
	                double arccos = acos(div);
	                q5[i][0] = arccos;
	                q5[i][1] = 2.0*PI - arccos;
	            }
	        }
	        ////////////////////////////////////////////////////////////////////////////////

	        {
	            for(int i=0;i<2;i++) {
	                for(int j=0;j<2;j++) {
	                    double c1 = cos(q1[i]), s1 = sin(q1[i]);
	                    double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
	                    double q6;
	                    ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
	                    if(fabs(s5) < ZERO_THRESH)
	                        q6 = 0;
	                    else {
	                        q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1),
	                                   SIGN(s5)*(T00*s1 - T10*c1));
	                        if(fabs(q6) < ZERO_THRESH)
	                            q6 = 0.0;
	                        if(q6 < 0.0)
	                            q6 += 2.0*PI;
	                    }
	                    ////////////////////////////////////////////////////////////////////////////////

	                    double q2[2], q3[2], q4[2];
	                    ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
	                    double c6 = cos(q6), s6 = sin(q6);
	                    double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
	                    double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
	                    double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +
	                    T03*c1 + T13*s1;
	                    double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

	                    double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
	                    if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
	                        c3 = SIGN(c3);
	                    else if(fabs(c3) > 1.0) {
	                        // TODO NO SOLUTION
	                        continue;
	                    }
	                    double arccos = acos(c3);
	                    q3[0] = arccos;
	                    q3[1] = 2.0*PI - arccos;
	                    double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
	                    double s3 = sin(arccos);
	                    double A = (a2 + a3*c3), B = a3*s3;
	                    q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
	                    q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
	                    double c23_0 = cos(q2[0]+q3[0]);
	                    double s23_0 = sin(q2[0]+q3[0]);
	                    double c23_1 = cos(q2[1]+q3[1]);
	                    double s23_1 = sin(q2[1]+q3[1]);
	                    q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
	                    q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
	                    ////////////////////////////////////////////////////////////////////////////////
	                    for(int k=0;k<1;k++) {
	                        if(fabs(q2[k]) < ZERO_THRESH)
	                            q2[k] = 0.0;
	                        else if(q2[k] < 0.0) q2[k] += 2.0*PI;
	                        if(fabs(q4[k]) < ZERO_THRESH)
	                            q4[k] = 0.0;
	                        else if(q4[k] < 0.0) q4[k] += 2.0*PI;

	                        xJoint[0].data =(float) q1[i];    xJoint[1].data =(float) q2[k];
	                        xJoint[2].data =(float) q3[k];    xJoint[3].data =(float) q4[k];
	                        xJoint[4].data =(float)q5[i][j]; xJoint[5].data =(float) q6;
	                        num_sols++;
	                    }

	                }
	            }
	        }
	        return num_sols;
	}



	void UR5Core::sendJointCommands(std_msgs::Float64* xJoint){
		for (int i = 0; i < 6; i++)printf("Joint [%d]: %.3f \n", i, xJoint[i].data);


	UR5JointPublisher[0].publish(xJoint[0]);
	UR5JointPublisher[1].publish(xJoint[1]);
	UR5JointPublisher[2].publish(xJoint[2]);
	UR5JointPublisher[3].publish(xJoint[3]);
	UR5JointPublisher[4].publish(xJoint[4]);
	UR5JointPublisher[5].publish(xJoint[5]);
	printf("Joints Sent Successfully! \n");

}
