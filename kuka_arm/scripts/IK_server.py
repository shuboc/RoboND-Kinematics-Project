#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
        # Initialize service response
        joint_trajectory_list = []

        # Define DH param symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


        # Modified DH params
        s = {alpha0: 0,     a0: 0,      d1: 0.75,
             alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
             alpha2: 0,     a2: 1.25,   d3: 0,
             alpha3: -pi/2, a3: -0.054, d4: 1.5,
             alpha4: pi/2,  a4: 0,      d5: 0,
             alpha5: -pi/2, a5: 0,      d6: 0,
             alpha6: 0,     a6: 0,      d7: 0.303, q7: 0
        }


        # Define Modified DH Transformation matrix
        def create_ht_from_dh_params(alpha, a, d, q):
            return Matrix([[cos(q),                      -sin(q),           0,             a],
                           [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                           [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                           [                0,                 0,           0,             1]])

        def rotate_x(angle):
            return Matrix([[ 1,              0,        0],
                           [ 0,        cos(angle), -sin(angle)],
                           [ 0,        sin(angle),  cos(angle)]])

        def rotate_y(angle):
            return Matrix([[ cos(angle), 0, sin(angle)],
                           [          0, 1,          0],
                           [-sin(angle), 0, cos(angle)]])

        def rotate_z(angle):
            return Matrix([[cos(angle), -sin(angle), 0],
                           [sin(angle),  cos(angle), 0],
                           [         0,           0, 1]])


        # Create individual transformation matrices
        T0_1 = create_ht_from_dh_params(alpha0, a0, d1, q1)
        T0_1 = T0_1.subs(s)

        T1_2 = create_ht_from_dh_params(alpha1, a1, d2, q2)
        T1_2 = T1_2.subs(s)

        T2_3 = create_ht_from_dh_params(alpha2, a2, d3, q3)
        T2_3 = T2_3.subs(s)

        T3_4 = create_ht_from_dh_params(alpha3, a3, d4, q4)
        T3_4 = T3_4.subs(s)

        T4_5 = create_ht_from_dh_params(alpha4, a4, d5, q5)
        T4_5 = T4_5.subs(s)

        T5_6 = create_ht_from_dh_params(alpha5, a5, d6, q6)
        T5_6 = T5_6.subs(s)

        T6_G = create_ht_from_dh_params(alpha6, a6, d7, q7)
        T6_G = T6_G.subs(s)

        R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                  [sin(pi),  cos(pi), 0, 0],
                  [         0,           0, 1, 0],
                  [         0,           0, 0, 1]])

        R_y = Matrix([[ cos(-pi/2), 0, sin(-pi/2), 0],
                  [             0, 1,             0, 0],
                  [-sin(-pi/2), 0, cos(-pi/2), 0],
                  [             0, 0,             0, 1]])

        R_corr = R_z * R_y

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

            # Calculate joint angles using Geometric IK method
            R_z = rotate_z(yaw)
            R_y = rotate_y(pitch)
            R_x = rotate_x(roll)

            # Calculate transform from base to gripper frame
            # using extrinsic transform with rpy angles.
            # A correction matrix accounts for the angle difference between ROS frame and
            # gripper frame in DH convention, which is pi/2 about y and -pi about z
            R_rpy = simplify(R_z * R_y * R_x * rotate_y(pi/2) * rotate_z(-pi))

            #print("R_rpy = ", R_rpy)

            wx = px - d7 * R_rpy[0,2]
            wy = py - d7 * R_rpy[1,2]
            wz = pz - d7 * R_rpy[2,2]

            wx = wx.subs(s)
            wy = wy.subs(s)
            wz = wz.subs(s)
            print "WC =", wx, wy, wz

            x = (sqrt(wx * wx + wy * wy) - 0.35)
            z = wz - 0.75
            delta = atan2(0.054, 1.5)
            #print 'x =', x
            #print 'z =', z
            #print 'delta =', delta
            A = sqrt(0.054 * 0.054 + 1.5 * 1.5)
            B = sqrt(x * x + z * z)
            C = 1.25

            # Solve first 3 joint angles by applying cosine-law
            qq1 = atan2(wy, wx)
            qq2 = pi/2 - acos((B*B + C*C - A*A) / (2 * B * C)) - atan2(z, x)
            qq3 = pi/2 - acos((A*A + C*C - B*B) / (2 * A * C)) - delta

            print 'qq1 = ', qq1
            print 'qq2 =', qq2
            print 'qq3 =', qq3

            # Solve rotation matrix by applying the first 3 joint angles
            # to the overall transform
            T0_3 = T0_1 * T1_2 * T2_3
            T0_3 = T0_3.subs({q1: qq1, q2: qq2, q3: qq3})
            R0_3 = T0_3[0:3, 0:3]
            R3_6 = R0_3.inv('LU') * R_rpy

            # Solve the last 3 joint variables by solving euler angles from the
            # rotation matrix
            qq4 = atan2(R3_6[2,2], -R3_6[0,2])
            qq5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            qq6 = atan2(-R3_6[1,1], R3_6[1,0])

            print 'qq4 =', qq4
            print 'qq5 =', qq5
            print 'qq6 =', qq6

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            theta1 = qq1
            theta2 = qq2
            theta3 = qq3
            theta4 = qq4
            theta5 = qq5
            theta6 = qq6

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
