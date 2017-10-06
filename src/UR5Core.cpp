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


	bool UR5Core::inverse(const geometry_msgs::Pose & desiredPose) {
			bool output = true;
			Eigen::Vector3d p(desiredPose.position.x,desiredPose.position.y, desiredPose.position.z);
			Eigen::Quaterniond R = Eigen::Quaterniond(desiredPose.orientation.w,desiredPose.orientation.x,desiredPose.orientation.y,desiredPose.orientation.z);
			//printf("Quaternion scalar:%.3f, Vector:[%.3f, %.3f, %.3f] \n", R.w(), R.x(), R.y(),R.z());
			double beta=0, alpha=0, theta_1[4], theta_5[2], theta0=PI, theta1=0, theta5=0, theta6=0;

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
            printf("----------------- \n");

            printf("[%.2f, %.2f, %.2f, %.2f] \n",T00, T01, T02, T03 );
            printf("[%.2f, %.2f, %.2f, %.2f] \n",T10, T11, T12, T13 );
            printf("[%.2f, %.2f, %.2f, %.2f] \n",T20, T21, T22, T23 );
            printf("----------------- \n");

            double q[6];
////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
				Eigen::Vector3d pW = p + (R.toRotationMatrix().col(2)*d6);
				printf("pW: [%.3f, %.3f, %.3f] \n", pW[0], pW[1], pW[2]);
				double pWn = sqrt(pW[0]*pW[0]+ pW[1]*pW[1]);
				printf("pWn: %.3f \n",pWn);

            	if(fabs(pWn) < ZERO_THRESH){ // it means the value is equal 0
            		ROS_ERROR("Infinite solutions may exist for theta1");
            		output = false;
            		return output;
            	}

            	else if(fabs(d4)>fabs(pWn)) {
            	            		ROS_ERROR("No solution exists!");
            	            		output = false; return output;
            	            	}
            	else{
            		// Standard solution set
            		    if (fabs( (d4/pWn) - 1) < ZERO_THRESH)
            		        // asin(d4/R) =  pi/2
            		        beta =  PI/2;
            		    else if (fabs( (d4/pWn) + 1) < ZERO_THRESH)
            		        // asin(d4/R) = -pi/2
            		        beta = -PI/2;
            		    else
            		        // asin(d4/R)
            		        beta = asin(d4/pWn);

            		    alpha = atan2(pW[1],pW[0]);
            		       theta_1[0] = alpha + beta;
            		       theta_1[1] = alpha - beta;
            		       theta_1[2] = alpha + beta + theta0;
            		       theta_1[3] = alpha - beta + theta0;
            		       theta1 = theta_1[2];
            		       printf("Joint before [%d]: %.3f \n", 0, theta1);
            		       UR5Joints[0] = wrapTo2PI(theta1);

            	}
////////////////////////////// joint (q5) //////////////////////////////
            	   // Difference along the rotated y-direction
            	    double num = (p[0]*sin(-(theta0+theta1)) + p[1]*cos(-(theta0+theta1)))-(-d4);
            	    double den = -d6;
            	    // Solution near 0 and 2*pi
            	    if (fabs((num/den) - 1 ) < ZERO_THRESH)
            	    		{
            	        theta_5[0]   = 0;
            	        theta_5[1] = 2*PI;
            	    		}
            	        // Solution near -pi and pi
            	    else if (fabs((num/den) + 1) < ZERO_THRESH)
            	    {
            	        theta_5[0] = PI;
            	        theta_5[1] = -PI;
            	    }
            	    else
            	        if ((fabs(p[0]) < ZERO_THRESH) && (fabs(p[1]) < ZERO_THRESH))
						{
            	            // SPECIAL CASE: End-effector is directly above the base frame
            	            if ((fabs(T03) < ZERO_THRESH) && (fabs(T13) < ZERO_THRESH) && (fabs(T23) < ZERO_THRESH))
            	            {
            	                //TODO - there may be other solutions
            	                theta_5[0]   =  PI/2;
            	                theta_5[1] = -PI/2; //  there may be another solution?
            	            }

            	            else if ((fabs(T03) < ZERO_THRESH) && (fabs(T13) < ZERO_THRESH) && (fabs(T23+1) < ZERO_THRESH))
            	            {
            	                //TODO - there may be other solutions
            	                theta_5[0]   = -PI/2;
            	                theta_5[1] =  PI/2; // there may be another solution?
            	            }
            	            else
            	            {
            	                // UNKNOWN SOLUTION
            	                ROS_ERROR("Unknown solution for theta5.");output = false; return output;
            	            }
						}
            	        else if (fabs(num) > fabs(den))
            	        {
            	        		ROS_ERROR("No valid solution for theta5.");output = false; return output;
            	        }
            	        else
            	        {
            	            // Inverse Cosine Solution
            	            theta_5[0]   =  acos( num/den );
            	            theta_5[1] = -acos( num/den );
            	        }
            	    theta5 = theta_5[1];
            	    UR5Joints[4] = wrapTo2PI(theta5);
////////////////////////////// joint (q6) //////////////////////////////
            	    if (fabs(sin(theta5)) < ZERO_THRESH)
            	    {
            	            // Axes for \theta_2, \theta_3, \theta_4, and \theta_5 are aligned
            	            // Infinite Solutions
            	            ROS_ERROR("Infinite solutions exist for theta6.");output = false; return output;
            	    }
            	    else
            	    {
            	            // Standard solution
            	            num = -(T01*sin( -(theta0+theta1)) + T11*cos(-(theta0+theta1)))/sin(theta5);
            	            den =  (T00*sin( -(theta0+theta1)) + T10*cos(-(theta0+theta1)))/sin(theta5);

            	            theta6 = atan2(num,den);
            	            UR5Joints[5] = wrapTo2PI(theta6);

            	    }
////////////////////////////// joint (q6) //////////////////////////////
            	    %% Calculate \theta_2, \theta_3, and \theta_4
            	    n = numel(theta1);
            	    for i = 1:n
            	        % Calculate H2e (H^2_e)
            	        % -> Shorthand used:
            	        %       HAB = H^A_B (Frame B relative to Frame A)
            	        % -> General form:
            	        %       H_b2a = H^a_b (Frame a relative to Frame b)
            	        DH_q = UR_DHtable(urMod,[theta1(i); 0; 0; 0; theta5(i); theta6(i)]);
            	        H01 = DH(DH_q(1,1),DH_q(1,2),DH_q(1,3),DH_q(1,4));
            	        H12 = DH(DH_q(2,1),DH_q(2,2),DH_q(2,3),DH_q(2,4));
            	        H02 = H01*H12;
            	        H2e = invSE(H02)*H_e2o; % End-effector Frame relative to Frame 2

            	        % Calculate H25 (H^2_5) & p25
            	        H56 = DH(DH_q(6,1),DH_q(6,2),DH_q(6,3),DH_q(6,4));
            	        H6e = DH(DH_q(7,1),DH_q(7,2),DH_q(7,3),DH_q(7,4));
            	        H5e = H56*H6e;
            	        H25 = H2e*invSE(H5e); % Frame 5 relative to Frame 2
            	        p25 = H25(1:3,4);     % Origin of Frame 5 relative to Frame 2

            	        R(i) = sqrt( p25(1)^2 + p25(2)^2 );
            	        %(a2^2 + R(i)^2 - a3^2)
            	        %(2*a2*R(i))
            	        if R(i) <= a2 + a3 && (a2^2 + R(i)^2 - a3^2) <= (2*a2*R(i))
            	            % These initial calculations assume that the +z-axis for joint 2,
            	            % 3, and 4 are aligned. The step following this calculation
            	            % accounts for the possibility of changing the z-direction.

            	            % Calculate \theta_2
            	            alpha(i) = sgnTheta2*atan2(p25(2),p25(1));
            	            beta(i) = acos( (a2^2 + R(i)^2 - a3^2)/(2*a2*R(i)) );
            	            theta2(i)   = alpha(i) + beta(i); % elbow-down solution
            	            theta2(i+n) = alpha(i) - beta(i); % elbow-up solution
            	            % Calculate \theta_3
            	            gamma(i) = acos( (a2^2 + a3^2 - R(i)^2)/(2*a2*a3) );
            	            theta3(i)   = -(pi-gamma(i)); % elbow-down solution
            	            theta3(i+n) =  (pi-gamma(i)); % elbow-up solution
            	            % Calculate \theta_4
            	            angSum = sgnTheta2*atan2(H25(2,1),H25(1,1));
            	            theta4(i)   = angSum - theta2(i)   - theta3(i);
            	            theta4(i+n) = angSum - theta2(i+n) - theta3(i+n);

            	            % Account for possible differences in the +z-axis between joint 2,
            	            % 3, and 4.
            	            if sgnTheta2 ~= sgnTheta3
            	                theta3(i)   = -theta3(i);
            	                theta3(i+n) = -theta3(i+n);
            	            end
            	            if sgnTheta2 ~= sgnTheta4
            	                theta4(i)   = -theta4(i);
            	                theta4(i+n) = -theta4(i+n);
            	            end
            	        else
            	            % No Solution Exists
            	            theta2(i)   = nan;
            	            theta2(i+n) = nan;
            	            theta3(i)   = nan;
            	            theta3(i+n) = nan;
            	            theta4(i)   = nan;
            	            theta4(i+n) = nan;
            	        end
            	    end

            	    % Repeat solutions for the additional solutions of \theta_2, \theta_3, & \theta_4
            	    theta0 = repmat(theta0,1,2);
            	    theta1 = repmat(theta1,1,2);
            	    theta5 = repmat(theta5,1,2);
            	    theta6 = repmat(theta6,1,2);

            	    %% Show thetas
            	    theta2 = wrapTo2Pi(theta2);
            	    theta3 = wrapTo2Pi(theta3);
            	    theta4 = wrapTo2Pi(theta4);






//////////////////////////////////////////////////////////////////////////
            	//for (int i = 0; i < 1; i++){
            		xJoint[0].data =(float) UR5Joints[0]; // for ROS V-REP
            	    xJoint[4].data =(float) UR5Joints[4];
            	    xJoint[5].data =(float) UR5Joints[5];


            		printf("Joint[%d]: %.3f \n", 1, UR5Joints[0]);
            		printf("Joint[%d]: %.3f \n", 5, UR5Joints[4]);
            		printf("Joint[%d]: %.3f \n", 6, UR5Joints[5]);


            	//}

            	return output;
	}

	//********************************************************************************************************//
	void UR5Core::sendJointCommands(std_msgs::Float64* xJoint){
	//	for (int i = 0; i < 6; i++)printf("Joint [%d]: %.3f \n", i, xJoint[i].data);
	UR5JointPublisher[0].publish(xJoint[0]);
	//UR5JointPublisher[1].publish(xJoint[1]);
	//UR5JointPublisher[2].publish(xJoint[2]);
	//UR5JointPublisher[3].publish(xJoint[3]);
	UR5JointPublisher[4].publish(xJoint[4]);
	UR5JointPublisher[5].publish(xJoint[5]);
	printf("Joints Sent Successfully! \n");

}

	double UR5Core::wrapTo2PI(double angle){
		bool was_neg = angle < 0;
				angle = fmod(angle,(2.0 * M_PI));
				   if (was_neg) angle += (2.0 * M_PI);
		return angle;
	}



