[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[theta2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

[dh-space]: ./misc_images/dh-space.png

[dh-f0t1]: ./misc_images/f0t1.png
[dh-f1t2]: ./misc_images/f1t2.png
[dh-f2t3]: ./misc_images/f2t3.png
[dh-f3tg]: ./misc_images/f3tg.png

[theta1]: ./misc_images/theta1.jpg
[theta2]: ./misc_images/theta2.jpg
[theta3]: ./misc_images/theta3.jpg

[DHtransform]: ./misc_images/DHtransform.png
[transformation]: ./misc_images/transformation.png
[homogeneous_matrix_result]: ./misc_images/homogeneous_matrix_result.png

[EEtoWC]: ./misc_images/EEtoWC.png
[R3to6]: ./misc_images/R3to6.png
[euler]: ./misc_images/euler.png

[test1]: ./misc_images/test1.png
[test2]: ./misc_images/test2.png


## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.



---

### Installation

* Clone this repository to your home directory:
 
```sh
$ git clone https://github.com/Jungduri/RoboND-Kinematics-Project.git ~/catkin_ws 
```

* Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

* Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
* Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

* Add following to your .bashrc file

```sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models
source ~/catkin_ws/devel/setup.bash
```

* For demo mode, change demo flag as `true` in `inverse_kenematics.launch` file under `~/catkin_ws/src/kuka_arm/launch/`

* For test mode, change demo flag as `False` in `inverse_kenematics.launch` file under `~/catkin_ws/src/kuka_arm/launch/`

* To run project, go to `~/catkin_ws/src/RoboND_Kinematics-project/kuka_arm/scripts`.

```sh
$ sudo chmod u+x safe_spawner.sh
$ ./safe_spawner.sh
```

* To run IK server, go to `~/catkin_ws/src/RoboND_Kinematics-project/kuka_arm/scripts`.

```sh
$ python IK_server.py
```


### Kinematic Analysis
#### Forward kinematic analysis of Kuka KR210 robot and derive its DH parameters.

* Get the x,y,z coordinate each joint from urdf file  

Joint | x | y | z | rotation 
--- | --- | --- | --- | ---
0 | 0 | 0 | 0 | -
1 | 0 | 0 | 0.33 | z
2 | 0.35 | 0 | 0.75 | y
3 | 0.35 | 0 | 2 | y
4 | 1.31 | 0 | 1.946 | x
5 | 1.85 | 0 | 1.946 | y
6 | 2.043 | 0 | 1.946 | x

* Define the origin to get DH-parameter

Origin should be follow the rolls:

1. z-axis > rotation axis
2. minimize redundant origin
3. base(0) is global origin
4. gripper origin define physically 


![dh-space][dh-space]

* Get DH-parameter
    - From 0 to 1

    ![no image][dh-f0t1]

        From origin 0 to 1, only link offset exist. 

        d1 = 0.75

    - From 1 to 2

    ![no image][dh-f1t2]

        From origin 1 to 2, both x and z axes change. There is twist and joint angle.
        And link length also exist.
        
        a1 =  0.35
        
        alpha1 = -90deg
        
        theta = theta2 - 90deg

    - From 2 to 3

    ![no image][dh-f2t3]

        From origin 2 to 3, there is an only link length.
    
    a2 = 1.25

    ![no image][dh-f3tg]

    - From 3 to 4
    
        From origin 3 to 4, link offset, length, twist occur.

        alpha3 = -pi/2

        a3 =  -0.054

        d4 = 1.5

    - From 4 to 5

        From origin 4 to 5, only twist angle.

        alpha4 = pi/2

    - From 5 to 6

        From origin 5 to 6, only twist angle.    

        alpha5 = -pi/2

    - From 6 to gripper

        From origin 6 to G, twist angle and link offset.

        d7 = 0.303

        Origin | alpha(i) | a(i) | d(i+1) | q(i+1)   
        --- | ---   | ---   | ---   | ---
        0>1 | 0     | 0     | 0.75  | q1 
        1>2 | -pi/2 | 0.35  | 0     | q2 - pi/2
        2>3 | 0     | 1.25  | 0     | q3
        3>4 | -pi/2 | -0.054| 1.5   | q4
        4>5 | pi/2  | 0     | 0     | q5
        5>6 | -pi/2 | 0     | 0     | q6
        6>G | 0     | 0     | 0.303 | q7

* (code) Define DH parameters.
```sh
DH_Table = {alpha0:         0, a0: 0,       d1: 0.75,   q1: q1,
            alpha1:  -pi / 2., a1: 0.35,    d2: 0,      q2: q2 - pi / 2.,
            alpha2:         0, a2: 1.25,    d3: 0,      q3: q3,
            alpha3:  -pi / 2., a3: -0.054,  d4: 1.50,   q4: q4,
            alpha4:   pi / 2., a4: 0,       d5: 0,      q5: q5,
            alpha5:  -pi / 2., a5: 0,       d6: 0,      q6: q6,
            alpha6:         0, a6: 0,       d7: 0.303,  q7: 0}
```


#### Individual transform matrices about each joint using the DH table.


To define transformation joint by joint, I must use homogeneous transformation. DH convention uses four individual transforms.

![no image][DHtransform]

Using DH paramteres and above expension, I can calculate homogeneous transformation matrix joint to joint.

![no image][transformation]



* (code) Homogeneous transformation

```sh
# Define Homogeuous tranformation function
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[              cos(q),             -sin(q),           0,               a],
                 [ sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                 [ sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                 [                   0,                   0,           0,               1]])
    return TF

```

* (code) Create ubduvudyak transformation matrices.

```sh
# Define each of TF
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

```

* (code) Finally, from base to end effector by multiplying each matrices.

```sh
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

![no image][homogeneous_matrix_result]

#### Inverse kinematics

Frist of all, end-effector needs to be rotation. This is because definition of EE, in URDF and DH convention is different.

```sh
        r, p, y = symbols('r p y')

        ROT_x = Matrix([[1,      0,       0],
                        [0, cos(r), -sin(r)],
                        [0, sin(r),  cos(r)]])

        ROT_y = Matrix([[cos(p), 0, sin(p)],
                        [    0,  1,       0],
                        [-sin(p), 0,  cos(p)]])

        ROT_z = Matrix([[cos(y), -sin(y), 0],
                        [sin(y),  cos(y), 0],
                        [     0,       0, 1]])

        ROT_EE = ROT_z * ROT_y * ROT_x

        Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
        ROT_EE = ROT_EE * Rot_Error
        ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

```

Get EE position(Px, Py, Pz) and roation(Roll, Pitch, Yaw). 

```sh
# Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # roll, pitch, yaw = end-effector orientation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            EE = Matrix([[px],
                         [py],
                         [pz]])
```

From EE orientation to waist center orientation, only diffence is z directional length.  

![no images][EEtoWC]

**d** is en-effector length from DH table. (=d7)

```sh
            WC = EE - 0.303 * ROT_EE[: ,2]
```

Now start to calculate the each joint theta value. 

Ti_j stands for homogeneous transformation from i to j joint. 


* Joint 1
![no image][theta1]
Let see the joint diagram in z_1 axis direction. Then it shown like the figure above. As joint 1 rotate, z value of waist center never change which means z onto the ground plane. This is because, as seen in the urdf file, joint 1 rotate via z global axis so it only change x,y coordinate of waist center.
Therefore, theta1 simple able to get as follows:
```
theta1 = atan2(WC[1], WC[0])
```

* Joint 2 & 3
![no image][theta2]

The joint 2 decoupling problem is far more difficult than joint1. Let see step by step.

Let see the joint diagram in z_2 axis direction. If you think about side_a, y directional movement isn't happen because there is any axis to make y axis rotation. So side_a is just `side_a = 1.501`.

Side_c, side_c is basically same value with `a_3=1.25`.

Side_b, is most complicate part of it. Due to the joint 1, y axis rotation occur. It directly leads to length of joint 1 to WC(l) is consists of both `wc_x, wc_y, wc_z`. Totally, `side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))`.

Finally, using cosine law we can get `theta2`. `theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)`.

![no image][theta3]

gamma is angle made by origin 3 and WC. So by referencing to DH parameter a3 and d4, gamma is calculated. `theta3 = pi / 2 - angle_b - atan2(0.0054, 1.5)`

```sh
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

            theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi / 2 - angle_b - 0.036 ## atan2(0.054, 1.5)
```

* Joint 4, 5, 6

Joint 4, 5, 6 is RRR manipulator. Therefore if we know the rotation matrix **R3_6**, then simply I can get each joint theta value.

**R0_6 = R0_1 R1_2 R2_3 R3_4 R4_5 R5_6** 

Ri_j = Ti_j[0:3,0:3]

R0_3 = R0_1 R1_2 R2_3 

R3_6 = R3_4 R4_5 R5_6

![no-image][R3to6]

```sh
            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            R3_6 = R0_3.transpose() * ROT_EE # transpose is same with inverse matirx in rotation matrix.
```

![no-image][euler]

RPY[0:3,0:3] in above is same with R3_6 in my code.

theta = theta5 = atan2(y,x) = atan2(-r31, sqrt(r11*r11+r21*r21)) (rij = RPY[i,j])

**psi = theta4 =**atan2(r32,r33)

**phi = theta6 =**atna2(r21, r11)

In case of theta5>pi greater than pi, we divide 2 case to avoid inverse function problem.

```sh
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2]))  

            if (theta5 > pi) :
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1],-R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1],R3_6[1,0])
```

And added to the forward kinematics codes to identity errors.

```sh
    FK = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
```


#### IK_debug.py output

```sh
Total run time to calculate joint angles from pose is 1.1253 seconds

Wrist error for x position is: 0.00000046
Wrist error for y position is: 0.00000032
Wrist error for z position is: 0.00000545
Overall wrist offset is: 0.00000548 units

Theta 1 error is: 0.00093770
Theta 2 error is: 0.00181024
Theta 3 error is: 0.00205031
Theta 4 error is: 0.00172067
Theta 5 error is: 0.00197873
Theta 6 error is: 0.00251871

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have multiple positions. It is best to add your forward kinematics to            
confirm whether your code is working or not**
 

End effector error for x position is: 0.00002010
End effector error for y position is: 0.00001531
End effector error for z position is: 0.00002660
Overall end effector offset is: 0.00003668 units 
```

###Project implementation

![no-image][test1]

![no-image][test2]

* Copy the code from `IK_debug.py` to `IK_server.py`.
* For test mode, change demo flag as `False` in `inverse_kenematics.launch` file under `~/catkin_ws/src/kuka_arm/launch/`
* Run `safe_spawner.sh`.
* Run `IK_server.py` in different terminal. 

###Future improvements

* Optimize path trajectory. Currently path planning made by project make when it began. But I guess more proper planning must be concerned. For example, in some cases, when it reached to the bin, normally arm goes up and reach to the bin. 

* Recognize can location using camera. In the project 3, I learn how to handle point camera data. Use it, and do some more fun things.

* To speed up code, use class all symbols and functions. 