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
JOINT_PERCH.position.a1 = 0/180 * np.pi
JOINT_PERCH.position.a2 = 37/180 * np.pi
JOINT_PERCH.position.a3 = np.pi/6 * 0
JOINT_PERCH.position.a4 = -97/180 * np.pi
JOINT_PERCH.position.a5 = 0/180 * np.pi
JOINT_PERCH.position.a6 = -45/180 * np.pi
JOINT_PERCH.position.a7 = 0/180 * np.pi

#Define quaternion that makes end effector parallel to ground
QUAT = Quaternion()
QUAT.x = 0
QUAT.y = 0.707
QUAT.z = 0
QUAT.w = 0.707

#Define quaternion that makes end effector perpendicular to ground with marker pointing in positive z direction
QUAT1 = Quaternion()
QUAT1.x = 0
QUAT1.y = 1
QUAT1.z = 0
QUAT1.w = 0

# Define cartesian points for correct trajectory
C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13 = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x = 0.75
x1 = 0.84
x2 = 0.84
C1.x, C1.y, C1.z = 0.6, -0.1, 0.3
C2.x, C2.y, C2.z = x, -0.1, 0.3
C3.x, C3.y, C3.z = x, -0.1, 0.5
C4.x, C4.y, C4.z = x, 0.1, 0.5
C5.x, C5.y, C5.z = x, 0.1, 0.3
C6.x, C6.y, C6.z = x, -0.1, 0.3
C7.x, C7.y, C7.z = 0.5, -0.1, 0.25
C8.x, C8.y, C8.z = x1, -0.05, 0.25
C9.x, C9.y, C9.z = x1, -0.05, 0.45
C10.x, C10.y, C10.z = x2, 0.15, 0.45
C11.x, C11.y, C11.z = x1, 0.15, 0.25
C12.x, C12.y, C12.z = x2, -0.05, 0.25
C13.x, C13.y, C13.z = 0.5, C1.y, C1.z

correct_pos = [C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12]

def correct():
    count = 0
    sleep = 2
    for cartesian_pos in correct_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #Rotate the end effector to eraser side
        if count >= 7:
        	iiwa.pose.pose.orientation = QUAT
        else:
        	iiwa.pose.pose.orientation = QUAT1
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(5)
        count = count + 1
    time.sleep(5)
    
#Define cartesian points for sub optimal trajectory
#The robot draws the square twice before erasing
C1a, C2a, C3a, C4a, C5a, C6a, C7a, C8a, C9a, C10a, C11a, C12a, C13a, C14a, C15a, C16a, C17a = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x = 0.65
C1a.x, C1a.y, C1a.z = x, -0.1, 0.2
C2a.x, C2a.y, C2a.z = x, -0.1, 0.4
C3a.x, C3a.y, C3a.z = x, 0.1, 0.4
C4a.x, C4a.y, C4a.z = x, 0.1, 0.2
C5a.x, C5a.y, C5a.z = x, -0.1, 0.2
C6a.x, C6a.y, C6a.z = x, C1.y, C1.z
C7a.x, C7a.y, C7a.z = x, C2.y, C2.z
C8a.x, C8a.y, C8a.z = x, C3.y, C3.z
C9a.x, C9a.y, C9a.z = x, C4.y, C4.z
C10a.x, C10a.y, C10a.z = x, C5.y, C5.z
C11a.x, C11a.y, C11a.z = 0.5, -0.2, 0.2
C12a.x, C12a.y, C12a.z = x, C1.y, C1.z
C13a.x, C13a.y, C13a.z = x, C2.y, C2.z
C14a.x, C14a.y, C14a.z = x, C3.y, C3.z
C15a.x, C15a.y, C15a.z = x, C4.y, C4.z
C16a.x, C16a.y, C16a.z = x, C5.y, C5.z
C17a.x, C17a.y, C17a.z = x, C1.y, C1.z

sub_opt_pos = [C1a, C2a, C3a, C4a, C5a, C6a, C7a, C8a, C9a, C10a, C11a, C12a, C13a, C14a, C15a, C16a, C17a]

def sub_opt():
    count = 0
    sleep = 2
    for cartesian_pos in sub_opt_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #Rotate the end effector to eraser side
        if count >= 11:
        	iiwa.pose.pose.orientation = QUAT
        else:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(sleep)
        count = count + 1
    time.sleep(5)

#Define cartesian points for too high trajectory
#Drawn square is above the bounded box    
C1b, C2b, C3b, C4b, C5b, C6b, C7b, C8b, C9b, C10b, C11b, C12b = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x = 0.65
C1b.x, C1b.y, C1b.z = x, -0.1, 0.2
C2b.x, C2b.y, C2b.z = x, -0.1, 0.4
C3b.x, C3b.y, C3b.z = x, 0.1, 0.4
C4b.x, C4b.y, C4b.z = x, 0.1, 0.2
C5b.x, C5b.y, C5b.z = x, -0.1, 0.2
C6b.x, C6b.y, C6b.z = 0.5, -0.1, 0.2
C7b.x, C7b.y, C7b.z = x, C1b.y, C1b.z
C8b.x, C8b.y, C8b.z = x, C2b.y, C2b.z
C9b.x, C9b.y, C9b.z = x, C3b.y, C3b.z
C10b.x, C10b.y, C10b.z = x, C4b.y, C4b.z
C11b.x, C11b.y, C11b.z = x, C5b.y, C5b.z
C12b.x, C12b.y, C12b.z = x, C1b.y, C1b.z

too_high_pos = [C1b, C2b, C3b, C4b, C5b, C6b, C7b, C8b, C9b, C10b, C11b, C12b]

def too_high():
    count = 0
    sleep = 2
    for cartesian_pos in too_high_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #Rotate the end effector to eraser side
        if count >= 6:
        	iiwa.pose.pose.orientation = QUAT
        else:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(5)
        count = count + 1
    time.sleep(5)

#Define cartesian points for not close trajectory
#Robot is not drawing on the board but in open space    
C1c, C2c, C3c, C4c, C5c, C6c, C7c, C8c, C9c, C10c, C11c, C12c = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x_c = 0.6
C1c.x, C1c.y, C1c.z = x_c, -0.1, 0.2
C2c.x, C2c.y, C2c.z = x_c, -0.1, 0.4
C3c.x, C3c.y, C3c.z = x_c, 0.1, 0.4
C4c.x, C4c.y, C4c.z = x_c, 0.1, 0.2
C5c.x, C5c.y, C5c.z = x_c, -0.1, 0.2
C6c.x, C6c.y, C6c.z = 0.5, -0.1, 0.2
C7c.x, C7c.y, C7c.z = x_c, C1c.y, C1c.z
C8c.x, C8c.y, C8c.z = x_c, C2c.y, C2c.z
C9c.x, C9c.y, C9c.z = x_c, C3c.y, C3c.z
C10c.x, C10c.y, C10c.z = x_c, C4c.y, C4c.z
C11c.x, C11c.y, C11c.z = x_c, C5c.y, C5c.z
C12c.x, C12c.y, C12c.z = x_c, C1c.y, C1c.z

not_close_pos = [C1c, C2c, C3c, C4c, C5c, C6c, C7c, C8c, C9c, C10c, C11c, C12c]

def not_close():
    count = 0
    sleep = 2
    for cartesian_pos in not_close_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #Rotate the end effector to eraser side
        if count >= 6:
        	iiwa.pose.pose.orientation = QUAT
        else:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(sleep)
        count = count + 1
    time.sleep(5)

#Define cartesian points for wrong shape trajectory
#Robot draws a star instead of a square
C1d, C2d, C3d, C4d, C5d, C6d, C7d, C8d, C9d, C10d, C11d, C12d, C13d, C14d, reset_d = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
x = 0.65
C1d.x, C1d.y, C1d.z = x, -0.3, 0.4
C2d.x, C2d.y, C2d.z = x, 0.3, 0.4
C3d.x, C3d.y, C3d.z = x, -0.2, 0.2
C4d.x, C4d.y, C4d.z = x, 0, 0.6
C5d.x, C5d.y, C5d.z = x, 0.2, 0.2
C6d.x, C6d.y, C6d.z = x, -0.3, 0.4
C7d.x, C7d.y, C7d.z = 0.5, C1d.y, C1d.z
C8d.x, C8d.y, C8d.z = x, C1d.y, C1d.z
C9d.x, C9d.y, C9d.z = x, C2d.y, C2d.z
C10d.x, C10d.y, C10d.z = x, C3d.y, C3d.z
C11d.x, C11d.y, C11d.z = x, C4d.y, C4d.z
C12d.x, C12d.y, C12d.z = x, C5d.y, C5d.z
C13d.x, C13d.y, C13d.z = x, C6d.y, C6d.z
C14d.x, C14d.y, C14d.z = x, C1d.y, C1d.z

wrong_shape_pos = [C1d, C2d, C3d, C4d, C5d, C6d, C7d, C8d, C9d, C10d, C11d, C12d, C13d, C14d]

def wrong_shape():
    count = 0
    sleep = 2
    for cartesian_pos in wrong_shape_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #Rotate the end effector to eraser side
        if count >= 7:
        	iiwa.pose.pose.orientation = QUAT
        else:
        	iiwa.pose.pose.orientation = QUAT1
        	sleep = 5
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(sleep)
        count = count + 1
    time.sleep(5)

trajectories = [correct, sub_opt, too_high, not_close, wrong_shape]

#Wake KUKA Up
time.sleep(2)
# Move robot to joint perch
iiwa.move_joint(JOINT_PERCH, commit=True)
time.sleep(5)
        
#Demonstrate correct trajectory
correct()

iiwa.move_joint(JOINT_PERCH, commit=True)

input("Press enter to continue")

#Loop through erroneous trajectories randomly
while len(trajectories) > 0:
	traj_to_run = random.choice(trajectories)
	traj_to_run()
	trajectories.remove(traj_to_run)
	time.sleep(5)
	iiwa.move_joint(JOINT_PERCH, commit=True)
	input("Press enter to continue")
	time.sleep(1)

print("Finished!")
