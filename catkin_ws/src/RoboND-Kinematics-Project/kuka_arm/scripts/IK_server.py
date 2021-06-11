#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')    
        #   
        # Create Modified DH parameters
        DH_Table = {alpha0: 0, 	    a0: 0, 		d1: 0.75, 	q1: q1,
		            alpha1: -pi/2., a1: 0.35,	d2: 0, 		q2: -pi/2. + q2,
		            alpha2: 0, 	    a2: 1.25, 	d3: 0, 		q3: q3,
		            alpha3: -pi/2,  a3: -0.054, d4: 1.5, 	q4: q4,
		            alpha4: pi/2, 	a4: 0, 		d5: 0, 		q5: q5,
		            alpha5: -pi/2., a5: 0, 		d6: 0, 		q6: q6,
		            alpha6: 0, 	    a6: 0, 		d7: 0.303, 	q7: 0}
        #
        # Define Modified DH Transformation matrix
        def DH_tfmat(alpha, a, d, q):
            TF = Matrix([
                    [cos(q), 		       -sin(q), 		     0, 		     a],
	     		    [sin(q)*cos(alpha), 	cos(q)*cos(alpha), 	-sin(alpha), 	-sin(alpha)*d],
	     		    [sin(q)* sin(alpha), 	cos(q)*sin(alpha), 	 cos(alpha), 	 cos(alpha)*d],
	     		    [0,			            0,			         0,		         1]
		        ])
            return TF

        #
        # Create individual transformation matrices
        T0_1 = DH_tfmat(alpha0, a0, d1, q1).subs(DH_Table)
    	T1_2 = DH_tfmat(alpha1, a1, d2, q2).subs(DH_Table)
    	T2_3 = DH_tfmat(alpha2, a2, d3, q3).subs(DH_Table)
    	T3_4 = DH_tfmat(alpha3, a3, d4, q4).subs(DH_Table)
    	T4_5 = DH_tfmat(alpha4, a4, d5, q5).subs(DH_Table)
    	T5_6 = DH_tfmat(alpha5, a5, d6, q6).subs(DH_Table)
    	T6_EE = DH_tfmat(alpha6, a6, d7, q7).subs(DH_Table)

        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
        #
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            EE = Matrix([[px],[py],[pz]])
            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            r, p , y = symbols('r p y')

            ROT_x = Matrix([[1, 0 ,      0],
                            [0, cos(r), -sin(r)],
                            [0, sin(r), cos(r)]]) 

            ROT_y = Matrix([[cos(p), 	0 , 	sin(p)],
                            [0, 		1, 	    0],
                            [-sin(p), 	0, 	    cos(p)]]) 

            ROT_z = Matrix([[cos(y), -sin(y), 0],
                            [sin(y), cos(y), 0],
                            [0, 0, 1]]) 
                            
            ROT_xyz = ROT_z * ROT_y * ROT_x
            ROT_corr = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
            ROT_final = ROT_xyz * ROT_corr

            ROT_final = ROT_final.subs({'r': roll, 'p': pitch, 'y': yaw})

            
            WC = EE - 0.303*(ROT_final[:,2]) #dG = 0.303
            #
            # Calculate joint angles using Geometric IK method
            # theta1
            theta1 = atan2(WC[1], WC[0])

            # theta2 and theta3
            A = 1.501
            B = sqrt((sqrt(WC[0]**2 + WC[1]**2) - 0.35)**2 + (WC[2]-0.75)**2)
            C = 1.25

            a = acos((B * B + C * C - A * A) / (2 * B * C))
            b = acos((A * A + C * C - B * B) / (2 * A * C))
            c = acos((A * A + B * B - C * C ) / (2 * A * B))

            theta2 = pi/2 - a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi/2 - b - 0.036

            #
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})
            R3_6 = R0_3.inv(method = "LU") * ROT_final
            #

            # theta4, theta5 and theta6
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
        

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
