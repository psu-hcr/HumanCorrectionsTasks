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

#Define quaternion that makes end effector perpendicular to ground
QUAT1 = Quaternion()
QUAT1.x = 0
QUAT1.y = 1
QUAT1.z = 0
QUAT1.w = 0

# Define cartesian points for correct trajectory
C1, C2, C3, reset = Point(), Point(), Point(), Point()
x = 0.69
C1.x, C1.y, C1.z = x, 0, 0.25
C2.x, C2.y, C2.z = x, 0, 0.2
C3.x, C3.y, C3.z = x, 0, 0.07
#Reset keeps the peg from knocking over the box when it goes back to joint perch
reset.x, reset.y, reset.z =  x, 0, 0.25

#Putting points into list
correct_pos = [C1, C2, C3]

def correct():
    count = 0
    for cartesian_pos in correct_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #After the moving the peg into place, rotate the endeffector downwards
        if count > 0:
        	iiwa.pose.pose.orientation = QUAT1
        else:
        	iiwa.pose.pose.orientation = QUAT
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(5)
        count = count + 1
    #Raising peg out of peg box	
    iiwa.pose.pose.position = reset
    iiwa.move_carte(iiwa.pose, commit=True)
    time.sleep(5)
    #Reset to joint perch
    iiwa.move_joint(JOINT_PERCH, commit=True)


#Define cartesian points for sub-optimal trajectory
#Peg is moved to far past the box then brought back to the correct position
C1a, C2a, C3a, C4a, C5a, C6a, C7a, reset_a = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
C1a.x, C1a.y, C1a.z = x, 0, 0.25
C2a.x, C2a.y, C2a.z = 0.85, 0, 0.25
C3a.x, C3a.y, C3a.z = x, 0, 0.2
C4a.x, C4a.y, C4a.z = x, 0, 0.07
#Reset keeps the peg from knocking over the box when it goes back to joint perch
reset_a_x, reset_a_y, reset_a_z = 0.69, 0, 0.2
reset_a.x, reset_a.y, reset_a.z = reset_a_x, reset_a_y, reset_a_z

#Putting points into list
sub_opt_pos = [C1a, C2a, C3a, C4a, C5a, C6a, C7a]

def sub_opt():
    count = 0
    for cartesian_pos in sub_opt_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #After the moving the peg into place, rotate the endeffector downwards
        if count < 4:
        	iiwa.pose.pose.orientation = QUAT
        	pause = 1.8
        elif count == 4:
        	pause = 4
        	iiwa.pose.pose.orientation = QUAT1
        elif count > 4:
        	iiwa.pose.pose.orientation = QUAT1
        	pause = 1.8
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(pause)
        count = count + 1
    #Raising peg out of peg box	
    iiwa.pose.pose.position = reset_a
    iiwa.move_carte(iiwa.pose, commit=True)
    time.sleep(5)
    #Reset to joint perch
    iiwa.move_joint(JOINT_PERCH, commit=True)

# Define cartesian points for collision trajectory
#Peg hits box from the side
C1b, C2b, C3b, reset_b = Point(), Point(), Point(), Point()
xb = 0.7
C1b.x, C1b.y, C1b.z = xb, -0.4, 0.25
C2b.x, C2b.y, C2b.z = xb, -0.4, 0.1
C3b.x, C3b.y, C3b.z = xb, -0.1, 0.07
#Reset keeps the peg from knocking over the box when it goes back to joint perch
reset_b.x, reset_b.y, reset_b.z = x, -0.1, 0.2

#Putting points into list
side_collision_pos = [C1b, C2b, C3b]

def side_collision():
    count = 0
    for cartesian_pos in side_collision_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #After the moving the peg into place, rotate the endeffector downwards
        if count > 0:
        	iiwa.pose.pose.orientation = QUAT1
        else:
        	iiwa.pose.pose.orientation = QUAT
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(5)
        count = count + 1
    #Raising peg out of peg box	
    iiwa.pose.pose.position = reset_b
    iiwa.move_carte(iiwa.pose, commit=True)
    time.sleep(4)
    #Reset to joint perch
    iiwa.move_joint(JOINT_PERCH, commit=True)

# Define cartesian points for wrong hole trajectory
#Peg goes into wrong hole
C1c, C2c, C3c, reset_c = Point(), Point(), Point(), Point()
C1c.x, C1c.y, C1c.z = x, 0, 0.25
C2c.x, C2c.y, C2c.z = x, -0.06, 0.2
C3c.x, C3c.y, C3c.z = x, -0.06, 0.07
#Reset keeps the peg from knocking over the box when it goes back to joint perch
reset_c.x, reset_c.y, reset_c.z = reset_c_x, -0.06, 0.2

#Putting points into list
wrong_hole1_pos = [C1c, C2c, C3c]

def wrong_hole1():
    count = 0
    for cartesian_pos in wrong_hole1_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #After the moving the peg into place, rotate the endeffector downwards
        if count > 0:
        	iiwa.pose.pose.orientation = QUAT1
        else:
        	iiwa.pose.pose.orientation = QUAT
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(5)
        count = count + 1
    #Raising peg out of peg box	
    iiwa.pose.pose.position = reset_c
    iiwa.move_carte(iiwa.pose, commit=True)
    time.sleep(5)
    #Reset to joint perch
    iiwa.move_joint(JOINT_PERCH, commit=True)

#Define cartesian points for hitting edge of correct hole trajectory
#Peg moves to correct hole but misses very slightly
C1d, C2d, C3d, C4d, reset_d = Point(), Point(), Point(), Point(), Point()
C1d.x, C1d.y, C1d.z = x, 0.03, 0.25
C2d.x, C2d.y, C2d.z = x, 0.03, 0.2
C3d.x, C3d.y, C3d.z = x, 0.03, 0.07
#Reset keeps the peg from knocking over the box when it goes back to joint perch
reset_d.x, reset_d.y, reset_d.z = x, 0.03, 0.25

#Putting points into list
close_to_hole1_pos = [C1d, C2d, C3d]

def close_to_hole():
    count = 0
    for cartesian_pos in close_to_hole_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #After the moving the peg into place, rotate the endeffector downwards
        if count > 0:
        	iiwa.pose.pose.orientation = QUAT1
        else:
        	iiwa.pose.pose.orientation = QUAT
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(5)
        count = count + 1
    #Raising peg out of peg box	
    iiwa.pose.pose.position = reset_a
    iiwa.move_carte(iiwa.pose, commit=True)
    time.sleep(5)
    #Reset to joint perch
    iiwa.move_joint(JOINT_PERCH, commit=True)

# Define cartesian points for wrong hole trajectory
#Peg goes into wrong hole
C1e, C2e, C3e, C4e, reset_d = Point(), Point(), Point(), Point(), Point()
C1e.x, C1e.y, C1e.z = x, 0, 0.25
C2e.x, C2e.y, C2e.z = x, 0.06, 0.2
C3e.x, C3e.y, C3e.z = x, 0.06, 0.07
#Reset keeps the peg from knocking over the box when it goes back to joint perch
reset_d.x, reset_d.y, reset_d.z = x, 0, 0.25

#Putting points into list
wrong_hole2_pos = [C1e, C2e, C3e]

def wrong_hole2():
    count = 0
    for cartesian_pos in wrong_hole2_pos:
        iiwa.pose.header.frame_id = 'iiwa_link_0'
        iiwa.pose.pose.position = cartesian_pos
        #After the moving the peg into place, rotate the endeffector downwards
        if count > 0:
        	iiwa.pose.pose.orientation = QUAT1
        else:
        	iiwa.pose.pose.orientation = QUAT
        iiwa.move_carte(iiwa.pose, commit=True)
        time.sleep(5)
        count = count + 1
    #Raising peg out of peg box	
    iiwa.pose.pose.position = reset_c
    iiwa.move_carte(iiwa.pose, commit=True)
    time.sleep(5)
    #Reset to joint perch
    iiwa.move_joint(JOINT_PERCH, commit=True)

#Put erroneous trajectories into list
trajectories = [sub_opt, side_collision, wrong_hole, wrong_hole2, close_to_hole]

#Wake KUKA Up
time.sleep(5)
# Move robot to joint perch
iiwa.move_joint(JOINT_PERCH, commit=True)
time.sleep(5)

#Demonstrate correcct trajectory
wrong_hole()

"""
#Loop through erroneous trajectories randomly
while len(trajectories) > 0:
	traj_to_run = random.choice(trajectories)
	traj_to_run()
	trajectories.remove(traj_to_run)
	time.sleep(5)
	iiwa.move_joint(JOINT_PERCH, commit=True)
	time.sleep(5)
"""

print("Finished!")
