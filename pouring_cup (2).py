#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from iiwa_msgs.msg import JointPosition
import time
import tf
import numpy as np
import random

class iiwaRobot(object):
    def __init__(self):
        rospy.init_node('iiwa_node', anonymous=True, log_level=rospy.INFO)
        self.joint_position = JointPosition()
        self.pose = PoseStamped()
        self.tf_listener = tf.TransformListener()
        self.joint_pos_publisher = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=1)
        self.carte_pose_publisher = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)

    def move_joint(self, joint_position=JointPosition(), commit=False):
        assert joint_position._type == 'iiwa_msgs/JointPosition'
        self.joint_pos_publisher.publish(joint_position)
        rospy.logwarn("iiwa is moving to joint position:\n{}".format(joint_position))

    def move_carte(self, pose=PoseStamped(), commit=False):
        assert pose._type == 'geometry_msgs/PoseStamped'
        self.carte_pose_publisher.publish(pose)
        rospy.logwarn("iiwa is moving to cartesian pose:\n{}".format(pose))\

iiwa = iiwaRobot()

# Define joint perch position
JOINT_PERCH = JointPosition()
JOINT_PERCH.position.a1 = -28/180 * np.pi
JOINT_PERCH.position.a2 = 82/180 * np.pi
JOINT_PERCH.position.a3 = -36/180 * np.pi
JOINT_PERCH.position.a4 = -68/180 * np.pi
JOINT_PERCH.position.a5 = -27/180 * np.pi
JOINT_PERCH.position.a6 = -81/180 * np.pi
JOINT_PERCH.position.a7 = 46/180 * np.pi

#Define quaternion that makes end effector parallel to ground
QUAT = Quaternion()
QUAT.x = 0
QUAT.y = 0.707
QUAT.z = 0
QUAT.w = 0.707

#Define quaternion that makes end effector perpendicular to ground
QUAT1 = Quaternion()
QUAT1.x = -0.653
QUAT1.y = 0.271
QUAT1.z = -0.653
QUAT1.w = 0.271

QUAT2 = Quaternion()
QUAT2.x = -0.707
QUAT2.y = -0
QUAT2.z = 0.707
QUAT2.w = 0

# Define cartesian points for correct trajectory
C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20 = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x = 0.72
C1.x, C1.y, C1.z = x, -0.5, 0.1
C2.x, C2.y, C2.z = x, -0.46, 0.15
C3.x, C3.y, C3.z = x, -0.42, 0.2
C4.x, C4.y, C4.z = x, -0.38, 0.25
C5.x, C5.y, C5.z = x, -0.34, 0.3
C6.x, C6.y, C6.z = x, -0.3, 0.35
C7.x, C7.y, C7.z = x, -0.225, 0.3125
C8.x, C8.y, C8.z = x, -0.15, 0.275
C9.x, C9.y, C9.z = x, -0.075, 0.2375
C10.x, C10.y, C10.z = x, 0, 0.2
C11.x, C11.y, C11.z = x, -0.05, 0.2
C12.x, C12.y, C12.z = x, -C9.y, C9.z
C13.x, C13.y, C13.z = x, -C8.y, C8.z
C14.x, C14.y, C14.z = x, -C7.y, C7.z
C15.x, C15.y, C15.z = x, -C6.y, C6.z
C16.x, C16.y, C16.z = x, -C5.y, C5.z
C17.x, C17.y, C17.z = x, -C4.y, C4.z
C18.x, C18.y, C18.z = x, -C3.y, C3.z
C19.x, C19.y, C19.z = x, -C2.y, C2.z
C20.x, C20.y, C20.z = x, -C1.y, C1.z

reset = Point()
reset.x, reset.y, reset.z = x, -C1.y, 0.4

correct_pos = [C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20]

def correct():
    count = 0
    sleep = 2
    for cartesian_pos in correct_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #At the last point, turn the end effector joint to pour out the contents of the cup
        if count == 10:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        else:
        	iiwa.pose.pose.orientation = QUAT
        	sleep = 2
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(sleep)
        count = count + 1
    iiwa.pose.pose.position = reset
    iiwa.move_carte(iiwa.pose, commit = True)
    time.sleep(sleep)
    
#Define cartesian points for sub-optimal trajectory
#This trajectory takes a much higher than necessary path over the obstacle
C1a, C2a, C3a, C4a, C5a, C6a, C7a, C8a, C9a, C10a, C11a, C12a, C13a, C14a, C15a, C16a, C17a, C18a, C19a, C20a = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x = 0.72
C1a.x, C1a.y, C1a.z = x, -0.5, 0.1
C2a.x, C2a.y, C2a.z = x, -0.46, 0.2
C3a.x, C3a.y, C3a.z = x, -0.42, 0.3
C4a.x, C4a.y, C4a.z = x, -0.38, 0.4
C5a.x, C5a.y, C5a.z = x, -0.34, 0.5
C6a.x, C6a.y, C6a.z = x, -0.3, 0.6
C7a.x, C7a.y, C7a.z = x, -0.225, 0.52
C8a.x, C8a.y, C8a.z = x, -0.15, 0.44
C9a.x, C9a.y, C9a.z = x, -0.075, 0.36
C10a.x, C10a.y, C10a.z = x, 0, 0.28
C11a.x, C11a.y, C11a.z = x, -0.05, 0.2
C12a.x, C12a.y, C12a.z = x, -C9a.y, C9a.z
C13a.x, C13a.y, C13a.z = x, -C8a.y, C8a.z
C14a.x, C14a.y, C14a.z = x, -C7a.y, C7a.z
C15a.x, C15a.y, C15a.z = x, -C6a.y, C6a.z
C16a.x, C16a.y, C16a.z = x, -C5a.y, C5a.z
C17a.x, C17a.y, C17a.z = x, -C4a.y, C4a.z
C18a.x, C18a.y, C18a.z = x, -C3a.y, C3a.z
C19a.x, C19a.y, C19a.z = x, -C2a.y, C2a.z
C20a.x, C20a.y, C20a.z = x, -C1a.y, C1a.z

sub_opt_pos = [C1a, C2a, C3a, C4a, C5a, C6a, C7a, C8a, C9a, C10a, C11a, C12a, C13a, C14a, C15a, C16a, C17a, C18a, C19a, C20a]

def sub_opt():
    count = 0
    sleep = 2
    for cartesian_pos in sub_opt_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #At the last point, turn the end effector joint to pour out the contents of the cup
        if count == 10:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        else:
        	iiwa.pose.pose.orientation = QUAT
        	sleep = 2
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(sleep)
        count = count + 1
    time.sleep(5)
    iiwa.pose.pose.position = reset
    iiwa.move_carte(iiwa.pose, commit = True)
    time.sleep(sleep)

# Define cartesian points for collision trajectory
C1b, C2b, C3b, C4b, C5b, C6b, C7b, C8b, C9b, C10b, C11b, C12b, C13b, C14b, C15b, C16b, C17b = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x = 0.72
C1b.x, C1b.y, C1b.z = x, -0.5, 0.1
C2b.x, C2b.y, C2b.z = x, -0.46, 0.136
C3b.x, C3b.y, C3b.z = x, -0.42, 0.172
C4b.x, C4b.y, C4b.z = x, -0.38, 0.208
C5b.x, C5b.y, C5b.z = x, -0.34, 0.244
C6b.x, C6b.y, C6b.z = x, -0.3, 0.28
C7b.x, C7b.y, C7b.z = x, -0.225, 0.24
C8b.x, C8b.y, C8b.z = x, -0.15, 0.2
C9b.x, C9b.y, C9b.z = x, -C9.y, C9.z
C10b.x, C10b.y, C10b.z = x, -C8.y, C8.z
C11b.x, C11b.y, C11b.z = x, -C7.y, C7.z
C12b.x, C12b.y, C12b.z = x, -C6.y, C6.z
C13b.x, C13b.y, C13b.z = x, -C5.y, C5.z
C14b.x, C14b.y, C14b.z = x, -C4.y, C4.z
C15b.x, C15b.y, C15b.z = x, -C3.y, C3.z
C16b.x, C16b.y, C16b.z = x, -C2.y, C2.z
C17b.x, C17b.y, C17b.z = x, -C1.y, C1.z

collision1_pos = [C1b, C2b, C3b, C4b, C5b, C6b, C7b, C8b, C9b, C10b, C11b, C12b, C13b, C14b, C15b, C16b, C17b]

def collision1():
    count = 0
    sleep = 2
    for cartesian_pos in collision1_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #At the last point, turn the end effector joint to pour out the contents of the cup
        if count == 7:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        else:
        	iiwa.pose.pose.orientation = QUAT
        	sleep = 2
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(sleep)
        count = count + 1
    time.sleep(5)
    iiwa.pose.pose.position = reset
    iiwa.move_carte(iiwa.pose, commit = True)
    time.sleep(sleep)


C1c, C2c, C3c, C4c, C5c, C6c, C7c, C8c, C9c, C10c, C11c, C12c, C13c, C14c, C15c, C16c, C17c = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x = 0.72
C1c.x, C1c.y, C1c.z = x, C1.y, C1.z
C2c.x, C2c.y, C2c.z = x, C2.y, C2.z
C3c.x, C3c.y, C3c.z = x, C3.y, C3.z
C4c.x, C4c.y, C4c.z = x, C4.y, C4.z
C5c.x, C5c.y, C5c.z = x, C5.y, C5.z
C6c.x, C6c.y, C6c.z = x, C6.y, C6.z
C7c.x, C7c.y, C7c.z = x, C7.y, C7.z
C8c.x, C8c.y, C8c.z = x, C8.y, C8.z
C9c.x, C9c.y, C9c.z = x, -C9.y, C9.z
C10c.x, C10c.y, C10c.z = x, 0.15, 0.2
C11c.x, C11c.y, C11c.z = x, 0.225, 0.24
C12c.x, C12c.y, C12c.z = x, 0.3, 0.28
C13c.x, C13c.y, C13c.z = x, 0.34, 0.244
C14c.x, C14c.y, C14c.z = x, 0.38, 0.208
C15c.x, C15c.y, C15c.z = x, 0.42, 0.172
C16c.x, C16c.y, C16c.z = x, 0.46, 0.136
C17c.x, C17c.y, C17c.z = x, 0.5, 0.1 

collision2_pos = [C1c, C2c, C3c, C4c, C5c, C6c, C7c, C8c, C9c, C10c, C11c, C12c, C13c, C14c, C15c, C16c, C17c]



def collision2():
    count = 0
    sleep = 2
    for cartesian_pos in collision2_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #At the last point, turn the end effector joint to pour out the contents of the cup
        if count == 8:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        else:
        	iiwa.pose.pose.orientation = QUAT
        	sleep = 2
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(sleep)
        count = count + 1
    time.sleep(5)
    iiwa.pose.pose.position = reset
    iiwa.move_carte(iiwa.pose, commit = True)
    time.sleep(sleep)

def early_pour():
    count = 0
    sleep = 2
    for cartesian_pos in correct_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #At the last point, turn the end effector joint to pour out the contents of the cup
        if count == 4 or count == 10:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        else:
        	iiwa.pose.pose.orientation = QUAT
        	sleep = 2
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(sleep)
        count = count + 1
    time.sleep(5)
    iiwa.pose.pose.position = reset
    iiwa.move_carte(iiwa.pose, commit = True)
    time.sleep(2)

C1d, C2d, C3d, C4d, C5d, C6d, C7d, C8d, C9d, C10d, C11d, C12d, C13d, C14d, C15d, C16d, C17d, C18d, C19d, C20d, C21d, C22d, C23d, C24d, C25d, C26d, C27d = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x = 0.72
C1d.x, C1d.y, C1d.z = -x, C1.y, C1.z
C2d.x, C2d.y, C2d.z = -x, C2.y, C2.z
C3d.x, C3d.y, C3d.z = -x, C3.y, C3.z
C4d.x, C4d.y, C4d.z = -x, C4.y, C4.z
C5d.x, C5d.y, C5d.z = -x, C5.y, C5.z
C6d.x, C6d.y, C6d.z = -x, C6.y, C6.z
C7d.x, C7d.y, C7d.z = -x, C7.y, C7.z
C8d.x, C8d.y, C8d.z = C1d.x, C1d.y, C1d.z
C9d.x, C9d.y, C9d.z = x, C2.y, C2.z
C10d.x, C10d.y, C10d.z = x, C3.y, C3.z
C11d.x, C11d.y, C11d.z = x, C4.y, C4.z
C12d.x, C12d.y, C12d.z = x, C5.y, C5.z
C13d.x, C13d.y, C13d.z = x, C6.y, C6.z
C14d.x, C14d.y, C14d.z = x, C7.y, C7.z
C15d.x, C15d.y, C15d.z = x, C8.y, C8.z
C16d.x, C16d.y, C16d.z = x, C9.y, C9.z
C17d.x, C17d.y, C17d.z = x, C10.y, C10.z
C18d.x, C18d.y, C18d.z = x, C11.y, C11.z
C19d.x, C19d.y, C19d.z = x, C12.y, C12.z
C20d.x, C20d.y, C20d.z = x, C13.y, C13.z
C21d.x, C21d.y, C21d.z = x, C14.y, C14.z
C22d.x, C22d.y, C22d.z = x, C15.y, C15.z
C23d.x, C23d.y, C23d.z = x, C16.y, C16.z
C24d.x, C24d.y, C24d.z = x, C17.y, C17.z
C25d.x, C25d.y, C25d.z = x, C18.y, C18.z
C26d.x, C26d.y, C26d.z = x, C19.y, C19.z
C27d.x, C27d.y, C27d.z = x, C20.y, C20.z


backwards_pos = [C1d, C2d, C3d, C4d, C5d, C6d, C7d, C8d, C9d, C10d, C11d, C12d, C13d, C14d, C15d, C16d, C17d, C18d, C19d, C20d, C21d, C22d, C23d, C24d, C25d, C26d, C27d]

def backwards():
    count = 0
    sleep = 2
    for cartesian_pos in backwards_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #At the last point, turn the end effector joint to pour out the contents of the cup
        if count == 17:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        elif count == 0:
        	iiwa.pose.pose.orientation = QUAT2
        	sleep = 7
        elif count > 0 and count < 8:
        	iiwa.pose.pose.orientation = QUAT2
        	sleep = 2
        else:
        	iiwa.pose.pose.orientation = QUAT
        	sleep = 2
        	if count == 8:
        		sleep = 7
        		
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(sleep)
        count = count + 1
    time.sleep(5)
    iiwa.pose.pose.position = reset
    iiwa.move_carte(iiwa.pose, commit = True)
    time.sleep(2)

trajectories = [correct, sub_opt, collision1, collision2, early_pour, backwards]

#Wake KUKA Up
time.sleep(2)
# Move robot to joint perch
iiwa.move_joint(JOINT_PERCH, commit=True)
time.sleep(5)
        
backwards()
"""
while len(trajectories) > 0:
	traj_to_run = random.choice(trajectories)
	traj_to_run()
	trajectories.remove(traj_to_run)
	time.sleep(5)
	iiwa.move_joint(JOINT_PERCH, commit=True)
	time.sleep(5)
"""
	
iiwa.move_joint(JOINT_PERCH, commit=True)
time.sleep(5)
print("Finished!")

