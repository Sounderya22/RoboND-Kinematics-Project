## Project: Kinematics Pick & Place

---
[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Kinematic Analysis
#### 1. Deriving the DH parameters by analyzing the URDF file
By analyzing the kr210.urdf.xacro file, we can get the xyz and rpy value of every joint in the arm. From that, we can calculate the DH parameters. 

DH Parameters  
* α<sub>i-1</sub>(twist angle) = angle between Z<sub>i-1</sub> and Z<sub>i</sub> measured about X<sub>i-1</sub>  
* a<sub>i-1</sub>(link length) = distance between Z<sub>i-1</sub> and Z<sub>i</sub> measured along X<sub>i-1</sub> which is perpendicular to both Z<sub>i-1</sub> and Z<sub>i</sub>
* d<sub>i</sub>(link offset) = signed distance from X<sub>i-1</sub> to X<sub>i</sub> measured along Z<sub>i</sub>  
* θ<sub>i</sub>(joint angle) = angle from X<sub>i-1</sub> and X<sub>i</sub> measured about Z<sub>i</sub>  

For example, from this part of the URDF file,
```xml
<joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
```  
From the origin tag, 
```xml
<origin xyz="0 0 0" rpy="0 0 0"/>
```
we can see that for the fixed_base joint, **xyz = "0 0 0"** and **rpy = "0 0 0"**  
Similarly, from the URDF file, we can get the xyz and rpy values of all the joints of the arm  

**Relative location of joint{i-1} to joint{i}**
Joint Name | Parent Link | Child Link | x(m) | y(m) | z(m) | roll | pitch | yaw |
--- | --- | --- | --- | --- | --- | --- | --- | --- | 
joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0 |
joint_2 | link_1 | link_2 | 0.35 | 0 | 0.42 | 0 | 0 | 0 |
joint_3 | link_2 | link_3 | 0 | 0 | 1.25 | 0 | 0 | 0 |
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0 |
joint_5 | link_4 | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0 |
joint_6 | link_5 | link_6 | 0.193 | 0 | 0 | 0 | 0 | 0 |
gripper-joint | link_6 | gripper_link | 0.11 | 0 | 0 | 0 | 0 | 0 |  

From this table, we can make a figure of the arm from which we can calculate the DH parameters

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


