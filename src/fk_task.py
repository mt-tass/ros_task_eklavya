#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''

TODO: Implement a ROS 2 node that reads joint angles and gripper states from angles.txt
and publishes them to the /forward_position_controller/commands topic.

Instructions:
1. Read the joint angles and gripper state from the file 'angles.txt' located in the package's share directory.
   Each line in the file contains: theta_base, theta_shoulder, theta_elbow, gripper_open.
2. Publish the joint angles and gripper state to the topic '/forward_position_controller/commands'.
   The message should contain 5 values in radians: [theta_base, theta_shoulder, theta_elbow, gripper_right, gripper_left].
   Set gripper_right and gripper_left to 0.8 if gripper_open is 1, else 0.0.
3. Add a delay of 2 seconds between publishing each set of joint values.

Your implementation goes below.

'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray 
from rclpy import qos
import math
import sys
import numpy as np

def get_coordinates(theta , d , alpha , a ):
    for index in range(len(theta)):
        theta[index] = math.radians(theta[index])

    transformation_matrix_initail = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]

    #transformation matrix multiplication T=T_initial * T_calc
    for index in range(len(theta)):

        row1 = [math.cos(theta[index]), -math.cos(alpha[index]) * math.sin(theta[index]), \
        		math.sin(alpha[index]) * math.sin(theta[index]), a[index] * math.cos(theta[index])]

        row2 = [math.sin(theta[index]), math.cos(alpha[index]) * math.cos(theta[index]), \
        		-math.sin(alpha[index]) * math.cos(theta[index]), a[index] * math.sin(theta[index])]

        row3 = [0, math.sin(alpha[index]), math.cos(alpha[index]), d[index]]

        row4 = [0,0,0,1]

        transformation_matrix_calculated = np.matrix([row1, row2, row3, row4])
        transformation_matrix_initail = transformation_matrix_initail * transformation_matrix_calculated

    return transformation_matrix_initail

d = [12, 0, 0, 10]
alpha = [math.pi/2,0, math.pi/2, 0]
a = [0, 7, 0, 0]
theta = []
gripper = []
#reading theta from angles.txt
with open("/ros_task_eklavya/ros_task/angles.txt",'r') as file:
    for line in file:
        part = line.split(',')
        thetas = [float(p) for p in part[:-1]] + [0]
        gripper.append(int(part[-1]))
        theta.append(thetas)

print(theta)
output_file = open("end_effector_position.txt" ,  "w")
i = 0
def forward_kinematics_publisher():
    global node
    global i
    if i >= len(theta):
        output_file.close()
        return

    Joints = node.create_publisher(Float64MultiArray, '/forward_position_controller/commands',qos_profile=qos.qos_profile_parameter_events)
    final_matrix_transformed = get_coordinates(theta[i] , d , alpha , a)
      
    output_file.write(f"{final_matrix_transformed[0,3]:0.5f},{final_matrix_transformed[1,3]:0.5f},{final_matrix_transformed[2,3]:0.5f}\n")
    print(f"{final_matrix_transformed[0,3]:0.5f},{final_matrix_transformed[1,3]:0.5f},{final_matrix_transformed[2,3]:0.5f}")
    joint = Float64MultiArray()
    joint.data = [0.0,0.0,0.0,0.0,0.0]
    if 0.0 <= theta[i][0] <= 180.0 and 0.0 <= theta[i][1] <= 180.0 and 0.0 <= theta[i][2] <= 180.0:
        joint.data[0] =(theta[i][0])
        joint.data[1] = (theta[i][1])
        joint.data[2] = (theta[i][2])
        if gripper:
            joint.data[3] = 0.8
            joint.data[4] = 0.8
        else:
            joint.data[3] = 0.0
            joint.data[4] = 0.0

        
        print("\ntheta_base = ", joint.data[0], "\n", "theta_shoulder = " , joint.data[1], "\n", "theta_elbow = " , joint.data[2], "\n", "Gripper Open = " , gripper[i], "\n")
        Joints.publish(joint)
    else:
        print("theta values out of range")
    
    i += 1
      
if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    global node
    node = Node('forward_kinematics_publisher')
    node.create_timer(2.0, forward_kinematics_publisher)
    rclpy.spin(node)
    rclpy.shutdown()
    
      
    
    





