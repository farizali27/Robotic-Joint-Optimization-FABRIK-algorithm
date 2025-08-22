# Robotic Arm Joint Optimization using FABRIK Algorithm
<br>

## Overview:
This Python script simulates the inverse kinematics of a robotic arm using the FABRIK (Forward And Backward Reaching Inverse Kinematics) algorithm. The script calculates the joint angles required to reach a specified target position.
<br>

## Assumptions
1. The base joint is at the origin (0,0,0)

2. I have assumed that the joints can only rotate in the x-y plane. Thus I have eliminated the z-coordinate of the target position assuming it to be zero.

3. I have taken tolerance value for fabrik algorithm to be 0.1 . The tolerance value can be adjusted depending on whether we want greater accuracy or time efficiency.
<br>

## Classes
1. Link Class:
Defines the characteristics of a link in the robotic arm. Each link has attributes such as length and joint angle.

2. RoboticArm Class:
Initializes the robotic arm with its links and a target position.
Provides methods to calculate joint angles, joint positions, check reachability, and perform forward and backward passes.
<br>

## Description/Flow of Code
1. ### main() function :-
    Takes initial joint angles(in degrees) and coordinates of target as input from the user.It then creates three link objects by passing the corresponding link lengths and joint angles. Creates object of RoboticArm class by passing the link objects and target as arguments.  
     get_points() function is then called to get the coordinates of the joints.It then checks the reachability of the target by calling the check_reachabliity() function. If it is reachable then the fabrik() function is called to implement the fabrik algorithm. Otherwise it prints "no" and program ends.

2. ### get_points() :-
   This function uses forward kinematics (by multiplying link lengths with rotation matrices) to calculate the initial coordinates of the joints. 

3. ### check_reachability():-
   It returns true if the distance of target from base joint(origin) is less or equal to the sum of link lenghts (40 in this case), otherwise returns false.

4. ### fabrik():-
   It is a recursive function that applies the fabrik theorem . The base condition is the comparision of the distance between end-effector and the target position. If it is less than the tolerance value the function ends.
   In each iteration/recursive call the function calls forward_pass() and updates the joint coordinates and then passes the updated points in the backward_pass() function to get the final coordinates after each iteration.
   The function calls the get_angles() function to get the joint angles and prints it.

5. ### forward_pass():-
    This funciton first sets the end-effector coordinates to be at the target position.Then for each of the previous joint p<sub>i</sub> it sets their new position along the vector p<sub>i+1</sub>-p<sub>i</sub> at a distance equal to the link length form p<sub>i+1</sub> . It then returns theses points to the fabrik function.

6. ### backward_pass():-
      This function sets the base joint coordinates at the origin. Then for each next joint p<sub>i</sub> it sets their new position along the vector p<sub>i</sub>-p<sub>i-1</sub> at a distance equal to the link length form p<sub>i-1</sub>. It then returns the points to the fabrik function.

7. ### get_angles() :-
      This function takes two vectors (v1,v2) along the two links between which we have to find the angle. It then calculates the angle by using the relation v1 dot v2 = cos(angle)|v1| |v2|.
      If the cross product of v1 and v2 is positive this means the rotaion is in counter-clockwise direction and the angle will be positive , otherwise it will be negative.
      It then returns the angles to the fabrik() function.   
