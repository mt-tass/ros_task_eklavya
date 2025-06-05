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
