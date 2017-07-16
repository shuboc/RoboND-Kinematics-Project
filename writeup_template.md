## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[dh-reference-frames]: ./misc_images/dh-reference-frames.JPG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![DH Reference Frames][dh-reference-frames]

The DH table is as follows:

i   | alpha[i-1] | a[i-1] | d[i]  | q[i]
--- | ---------- | ------ | ----- | ---
1   | 0          | 0      | 0.75  | q1
2   | -pi/2      | 0.35   | 0     | q2 - pi/2
3   | 0          | 1.25   | 0     | q3
4   | -pi/2      | -0.054 | 1.5   | q4
5   | pi/2       | 0      | 0     | q5
6   | -pi/2      | 0      | 0     | q6
G   | 0          | 0      | 0.2305| 0

The link lengths and offsets can be derived by looking into the urdf file:

* d1 = (z offset of joint 1) + (z offset of joint 2) = 0.33 + 0.42 = 0.75
* a1 = x offset of joint 2 = 0.35
* a2 = z offset of joint 3 = 1.25
* a3 = z offset of joint 4 = -0.054
* d4 = (x offset of joint 4) + (x offset of joint 5) = 0.96 + 0.54 = 1.5
* dG = (x offset of joint 6) + (x offset of the gripper joint) = 0.193 + 0.0375 = 0.2305

alpha[i-1] is the angle between Z[i-1] and Z[i] measured about X[i-1].

* (Z0, Z1), (Z2, Z3) and (Z6, ZG) are coincident, so alpha[0] = alpha[2] = alpha[6] = 0.
* (Z1, Z2), (Z3, Z4), (Z4, Z5), (Z5, Z6) are perpendicular, so alpha[1], alpha[3], alpha[4], alpha[5] are +- pi/2; the sign is determined in the right-hand sense.

q[i] is the angle between X[i-1] and X[i] measured about Z[i]. For a revolute joint, q[i] is simply the angle of the joint. However, Z1 and Z2 are not parallel in the zero configuration, so q2 should be given an offset of -pi/2.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


