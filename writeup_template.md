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
[ht_ind]: ./misc_images/ht_ind.png
[ht_corr]: ./misc_images/ht_corr.png
[ht_total]: ./misc_images/ht_total.png
[ht_3_6]: ./misc_images/ht_3_6.png
[dh-transform-matrix]: ./misc_images/dh-transform-matrix.png
[ext_rot_matrix]: ./misc_images/ext_rot_matrix.png
[wc]: ./misc_images/wc.png
[q1]: ./misc_images/q1.JPG
[q2q3]: ./misc_images/q2q3.JPG

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
G   | 0          | 0      | 0.303 | 0

The link lengths and offsets can be derived by looking into the urdf file:

* d1 = (z offset of joint 1) + (z offset of joint 2) = 0.33 + 0.42 = 0.75
* a1 = x offset of joint 2 = 0.35
* a2 = z offset of joint 3 = 1.25
* a3 = z offset of joint 4 = -0.054
* d4 = (x offset of joint 4) + (x offset of joint 5) = 0.96 + 0.54 = 1.5
* dG = (x offset of joint 6) + (x offset of the gripper joint) = 0.193 + 0.11 = 0.303

alpha[i-1] is the angle between Z[i-1] and Z[i] measured about X[i-1].

* (Z0, Z1), (Z2, Z3) and (Z6, ZG) are coincident, so alpha[0] = alpha[2] = alpha[6] = 0.
* (Z1, Z2), (Z3, Z4), (Z4, Z5), (Z5, Z6) are perpendicular, so alpha[1], alpha[3], alpha[4], alpha[5] are +- pi/2; the sign is determined in the right-hand sense.

q[i] is the angle between X[i-1] and X[i] measured about Z[i]. For a revolute joint, q[i] is simply the angle of the joint. However, Z1 and Z2 are not parallel in the zero configuration, so q2 should be given an offset of -pi/2.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Given the DH table, the individual transform can be determined as follows:

![DH Transform Matrix][dh-transform-matrix]

Therefore, individual transforms can be represented with the following python code, take T0_1 for example:

```
s = {alpha0: 0,     a0: 0,      d1: 0.75,
     alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
     alpha2: 0,     a2: 1.25,   d3: 0,
     alpha3: -pi/2, a3: -0.054, d4: 1.5,
     alpha4: pi/2,  a4: 0,      d5: 0,
     alpha5: -pi/2, a5: 0,      d6: 0,
     alpha6: 0,     a6: 0,      d7: 0.303, q7: 0
}

T0_1 = Matrix([[cos(q1),                        -sin(q1),            0,              a0],
               [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                  0,                   0,            0,               1]])

T0_1 = T0_1.subs(s)
```

The individual transform is given as follows:

![Individual HT][ht_ind]

Next, the total transform between `base_link` and `gripper_link` can be derived by multiplying all transforms and a correction matrix, which consists of an 180-degree rotation about z-axis and an -90-degree rotation about y-axis:

![Correction HT][ht_corr]

Thus, the total matrix is:

`T_total = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G * T_corr`

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The first step is to find the overall rotation matrix from `base_link` to `gripper_link`, i.e., `Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr`.

Assume the overall rotation angles about X, Y and Z axis are `gamma`, `beta` and `alpha`, respectively. These angles can be obtained from RPY angles in ROS. In fact, the roll angle from ROS is `gamma`, pitch angle is `beta`, and yaw angle is `alpha`. Finally, by applying extrinsic rotation, the overall rotation matrix can be calculated as follows:

![Extrinsic Rotation Matrix][ext_rot_matrix]

```
// TODO: Apply a correction matrix maybe?
```

**Position**

Given the total transform matrix, the position of WC can be calculated as follows:

![Position of Wrist Center][wc]

The relation between position of WC and `q1` can be illustrated as follows:

![theta1][q1]

Thus,

```
theta1 = atan2(wy, wx)
```

The relation between position of WC and `q2` and `q3` can be illustrated as follows:

![theta2 and theta3][q2q3]

To simplify explanation, the origin of the illustrattion is at O2. Also, I define several symbols for easier computation:

```
x = sqrt(wx^2 + wy^2)
z = wz - d1
delta = arctan(0.054/0.96)
```

From the above figure, 

```
a = pi/2 - q2 - atan2(z/x)
c = pi - (pi/2 + delta - q3) = pi/2 - delta + q3
```

Applying Cosine Laws to both equations will get:

```
cos(pi/2 - q2 - atan2(z/x)) = (B^2 + C^2 - A^2) / (2 * B * C)
cos(pi/2 - delta + q3) = (A^2 + C^2 - B^2) / (2 * A * C)
```

Thus, `q2` and `q3` can be calculated as follows:

```
q2 = arcsin((B^2 + C^2 - A^2) / (2 * B * C)) - atan2(z/x)
q3 = arcsin((A^2 + C^2 - B^2) / (2 * A * C)) - delta
```

```
TODO: select the best solution for q2 and q3 by observing the active workspace of the arm.
```

**Orientation**

Note that the IK problem has been decoupled into position and orientation problems, respectively, we can focus on the orientation problem now that the position has been found by solving `q1`, `q2` and `q3`.

Since the overall matrix is also the multiplication of all individual DH transforms, the following equation holds true:

`R0_6 = R0_1 * R1_2 * R2_3 * R3_4 * R4_5 * R5_6 = Rrpy`,

where `Rrpy` is the overall rotation matrix between `base_link` and `gripper_link`, which has been calculated as above, and `R0_3 = R0_1 * R1_2 * R2_3` can be calculated by applying `q1`, `q2` and `q3`.

Thus, the rotation matrix can be calculated as follows:

`R3_6 = inv(R0_3) * R0_6`.

Furthermore, `R3_6 = R3_4 * R4_5 * R5_6`. It can be obtained by taking the upperleft part from `T3_6`:

![HT_3_6][ht_3_6]

Assume `R3_6 = [[r11 r12 r13], [r21 r22 r23], [r31 r32 r33]]`, 

```
cos(q5) = r23

sin(q5) * cos(q6) = r21

-cos(q4) * sin(q5) = r13
```

=>

```
q5 = arccos(r23)

q6 = r21 / sqrt(1 - r23 * r23)

q4 = -r13 / sqrt(1 - r23 * r23)
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


