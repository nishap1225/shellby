#!/usr/bin/env python
"""
Path Planning Script for Shellby
Author: Matthew Sahim & Reena Yuan
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback
from intera_interface import gripper as robot_gripper
import tf2_ros
import math 

from moveit_msgs.msg import OrientationConstraint, PositionConstraint
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose

from path_planner import PathPlanner
from shape_msgs.msg import SolidPrimitive

from controller import Controller


# list of poses
def get_shell_poses():
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    error = True 
    while not rospy.is_shutdown() and error: 
        try: 
            trans_1 = tfBuffer.lookup_transform('base', 'ar_marker_5', rospy.Time())
            error_1 = trans_1.transform.translation
            trans_2 = tfBuffer.lookup_transform('base', 'ar_marker_6', rospy.Time())
            error_2 = trans_2.transform.translation
            trans_3 = tfBuffer.lookup_transform('base', 'ar_marker_8', rospy.Time())
            error_3 = trans_3.transform.translation
 
            error = False 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(1) 
            error = True 
            continue 

    pose_1 = PoseStamped()
    pose_1.header.frame_id = "base"
    pose_1.pose.position.x = error_1.x
    pose_1.pose.position.y = error_1.y
    pose_1.pose.position.z = error_1.z

    pose_2 = PoseStamped()
    pose_2.header.frame_id = "base"
    pose_2.pose.position.x = error_2.x
    pose_2.pose.position.y = error_2.y
    pose_2.pose.position.z = error_2.z

    pose_3 = PoseStamped()
    pose_3.header.frame_id = "base"
    pose_3.pose.position.x = error_3.x
    pose_3.pose.position.y = error_3.y
    pose_3.pose.position.z = error_3.z

    return [pose_1, pose_2, pose_3]

def get_pose(shell_str, cup_offset=[0, 0, 0]):
    print(shell_str)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    find_error = True 
    while not rospy.is_shutdown() and find_error: 
        try: 
            trans = tfBuffer.lookup_transform('base', shell_str, rospy.Time())
            error = trans.transform.translation
            find_error = False
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(1) 
            error = True 
            continue 

    pose = PoseStamped()
    pose.header.frame_id = "base"
    pose.pose.position.x = error.x + cup_offset[0]
    pose.pose.position.y = error.y + cup_offset[1]
    pose.pose.position.z = error.z + cup_offset[2]
    pose.pose.orientation.y = -1.0

    return pose


def switch_shells(shell_1_str, shell_2_str, shell_3_str, cup_offset):

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    pos_const = PositionConstraint()
    pos_const.link_name = "right_gripper";
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = (5, 5, 0)
    pos_const.constraint_region.primitives.append(box)
    pos_const.weight = 1.0


    ## get transformation from shell 1 to usb cam

    size = np.array([0.12, 0.05]) #shell size         TODO!!!!
    planner = PathPlanner("right_arm")
    right_gripper = robot_gripper.Gripper('right_gripper')
    shell_1 = get_pose(shell_1_str, cup_offset)
    shell_2 = get_pose(shell_2_str, cup_offset)
    shell_3 = get_pose(shell_3_str, cup_offset)
    shell_1_orig = shell_1
    shell_2_orig = shell_2

    ##
    # pos_const.constraint_region.primitive_poses.append(shell_1)
    ##

    shell_1_dest = PoseStamped()
    shell_1_dest.pose.position.z = shell_1.pose.position.z 
    shell_1_dest.pose.position.x = shell_1.pose.position.x - 0.2 
    shell_1_dest.pose.position.y = shell_1.pose.position.y + 0.2 

    print('Opening...')
    right_gripper.open()
    rospy.sleep(1.0)

    planner.add_cylinder_obstacle(size, 'shell_2', shell_2)
    planner.add_cylinder_obstacle(size, 'shell_3', shell_3)
    plan = planner.plan_to_pose(shell_1, [], [])
    print('going to get first shell')
    planner.execute_plan(plan[1])              #move to shell 1

    y = input() 

    right_gripper.close()
    rospy.sleep(1.0)

    print('moving first shell')
    plan = planner.plan_to_pose(shell_1_dest, [orien_const], [pos_const])
    planner.execute_plan(plan[1])              #move to mid
    right_gripper.open()
    rospy.sleep(1.0)

    shell_1 = get_pose(shell_1_str, cup_offset)         #now an obstacle in mid
    # shell_2 = get_pose(shell_2_str)
    # shell_3 = get_pose(shell_3_str)         #obstacle
    planner.add_box_obstacle(size, 'shell_1', shell_1)
    planner.remove_obstacle('shell_2')
    print('getting to shell 2')
    plan = planner.plan_to_pose(shell_2, [], [])
    planner.execute_plan(plan[1])              #move to shell 2

    y = input() 
    right_gripper.close()
    rospy.sleep(1.0)

    # shell_1_orig = PoseStamped()
    # shell_1_orig.z = shell_1.z 
    # shell_1_orig.x = shell_1.x + 0.2 
    # shell_1_orig.y = shell_1.y - 0.2
    print('moving shell 2 to shell 1s original pos')
    plan = planner.plan_to_pose(shell_1_orig, [], [])
    planner.execute_plan(plan[1])              #move to shell_1's original spot
    print('Opening...')
    right_gripper.open()
    rospy.sleep(1.0)

    planner.remove_obstacle('shell_1')
    # shell_1 = get_pose(shell_1_str)         
    shell_2 = get_pose(shell_2_str, cup_offset)
    planner.add_box_obstacle(size, 'shell_2', shell_2)
    print('getting shell 1')
    plan = planner.plan_to_pose(shell_1, [], [])
    planner.execute_plan(plan[1])              #move to mid
    print('Closing...')
    y = input() 
    right_gripper.close()
    rospy.sleep(1.0)


    plan = planner.plan_to_pose(shell_2_orig, [orien_const], [pos_const])
    print('moving shell 1 to shell 2 original place')
    planner.execute_plan(plan[1])              #move shell_1 to shell_2's original spot
    print('Opening...')
    right_gripper.open()
    rospy.sleep(1.0)


def main():
    """
    Main Script
    """

    planner = PathPlanner("right_arm")
    # Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    # Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    # Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    # Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    # controller = Controller(Kp, Kd, Ki, Kw, Limb("right"))

    planner.remove_obstacle('table')
    planner.remove_obstacle('wall')
    planner.remove_obstacle('ar_marker_5')
    planner.remove_obstacle('ar_marker_6')
    planner.remove_obstacle('ar_marker_8')
    planner.remove_obstacle('shell_1')
    planner.remove_obstacle('shell_2')
    planner.remove_obstacle('shell_3')


    # #Create a path constraint for the arm 
    
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;

    
    # right_gripper = robot_gripper.Gripper('right_gripper')
    # right_gripper.open()

    # #test


    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    error = True 
    while not rospy.is_shutdown() and error: 
        try: 
            trans_1 = tfBuffer.lookup_transform('base', 'ar_marker_6', rospy.Time())
            error = False 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(1) 
            error = True 
            continue 

    # print(trans_1)
    # print("_")
    t1 = trans_1.transform.translation


    # pose = PoseStamped()
    # pose.header.frame_id = 'base'
    # pose.pose.position.x = t1.x - 0.025 # in and out
    # pose.pose.position.y = t1.y - 0.055  # left to right
    # pose.pose.position.z = t1.z - 0.07 #- 0.125 ## down to up 
    # pose.pose.orientation.y = -1.0 
    # print("goal")
    # print(pose)

    size = np.array([.4, 1.2, 0.1])
    table_pos = PoseStamped()
    table_pos.pose.position.x = 0.662
    table_pos.pose.position.y = -0.108
    table_pos.pose.position.z =  -0.269
    # table_pose = PoseStamped(pose=Pose(position=table_pos, orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))
    table_pos.header.frame_id = "base";
    print("table")
    print(table_pos)
    planner.add_box_obstacle(size, 'table', table_pos)

    wall_size = np.array([0.1, 10.0, 10.0])
    wall_pose = PoseStamped() 
    wall_pose.pose.position.x = -0.628
    wall_pose.pose.position.y = 0.475
    wall_pose.pose.position.z = 0.536
    wall_pose.header.frame_id = "base";
    print("wall")
    print(wall_pose)
    planner.add_box_obstacle(wall_size, 'wall', wall_pose)

    # shell_2 = get_pose('ar_marker_5')
    # # shell_2.pose.position.x -= 0.03
    # # shell_2.pose.position.y -= 0.075
    # shell_2.pose.position.z -= 0.11

    # shell_3 = get_pose('ar_marker_8')
    # # shell_3.pose.position.x -= 0.03
    # # shell_3.pose.position.y -= 0.075
    # shell_3.pose.position.z -= 0.11
    # size = np.array([0.2, 0.05]) #shell size         TODO!!!!
    # planner.add_cylinder_obstacle(size, 'ar_marker_5', shell_2)
    # planner.add_cylinder_obstacle(size, 'ar_marker_8', shell_3)

    # print("ar marker 5 pose ")
    # print(shell_2) 

    # print("ar marker 8 pose")
    # print(shell_3) 

    # #pose.pose.orientation.y = -1.0
    # # print(pose)

    # plan = planner.plan_to_pose(pose, [])
    # planner.execute_plan(plan[1])              #move to mid

    # proceed = input() 
    # print("about to close ... ")
    # right_gripper.close()

    # print("dragging")
    # pos_const = PositionConstraint()
    # pos_const.header.frame_id = "base"
    # pos_const.link_name = "right_gripper";
    # box = SolidPrimitive()
    # box.type = SolidPrimitive.BOX
    # box.dimensions = (5, 5, 0.5)
    # pos_const.constraint_region.primitives.append(box)
    # pos_const.weight = 1.0

    # proceed = input() 

    # dest = Pose()
    # right_grip = tfBuffer.lookup_transform('base', 'reference/right_gripper_tip', rospy.Time())
    # print("ar marker 6")
    # print(pose)

    # print("right grip ")
    # print(right_grip)
    # dest.position.z = right_grip.transform.translation.z 
    # dest.position.x = right_grip.transform.translation.x + 0.13
    # dest.position.y = right_grip.transform.translation.y + 0.13

    # print('dest')
    # print(dest)

    # pos_const.constraint_region.primitive_poses.append(dest)

    # dest.orientation.y = -1.0 
    # dest_stamped = PoseStamped()
    # dest_stamped.pose = dest 
    # dest_stamped.header.frame_id = "base" 

    # plan = planner.plan_to_pose(dest_stamped, [], [pos_const])
    # print(plan)
    # planner.execute_plan(plan[1])              #move to mid
    # right_gripper.open()
    # rospy.sleep(1.0)

    # p = get_pose('reference/right_gripper_tip')
    # # right_grip = tfBuffer.lookup_transform('base', 'reference/right_gripper_tip', rospy.Time())

    # # shell_2 = get_pose(shell_2_str)
    # # shell_3 = get_pose(shell_3_str)         #obstacle
    # planner.add_box_obstacle(size, 'ar_marker_6', p)
    # planner.remove_obstacle('ar_marker_5')
    # print('getting to 5')

    # ar_5_pose = get_pose('ar_marker_5')
    # plan = planner.plan_to_pose(ar_5_pose, [])
    # planner.execute_plan(plan[1])              #move to shell 2
    # right_gripper.close()
    # rospy.sleep(1.0)


    switches = 0
    while switches < 1:

        shells = ['ar_marker_5', 'ar_marker_6', 'ar_marker_8']
        indices = np.random.choice(np.arange(3), size=3, replace=False)

        switch_shells(shells[indices[0]], shells[indices[1]], shells[indices[2]], [-.047, .9, -0.037])
        switches += 1


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
