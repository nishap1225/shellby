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
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse

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


def switch_shells(shell_1_str, shell_2_str, shell_3_str, planner, controller, cup_offset):

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


def set_j(limb, theta_dict):
    print("Executing" + str(theta_dict))
    limb.set_joint_position_speed(0.1)
    limb.set_joint_positions(theta_dict)

def main():
    """
    Main Script
    """

    x = [0.63, 0.80, 0.923]
    z = -0.102
    y = [0.118, -0.043, -0.201]
    cup_poses = [[], [], []]

    # main poses 
    for y_val in y: 
        pose = PoseStamped()
        pose.header.frame_id = "base"
        pose.pose.position.x = x[1]
        pose.pose.position.y = y_val 
        pose.pose.position.z = z
        pose.pose.orientation.y = -1.0
        cup_poses[1].append(pose)

    # front and back 
    for i in range(2): 
        back_pose = PoseStamped()
        back_pose.header.frame_id = "base"
        back_pose.pose.position.x = x[0] # back 
        back_pose.pose.position.y = (y[i] + y[i+1])/2 
        back_pose.pose.position.z = z
        back_pose.pose.orientation.y = -1.0
        cup_poses[2].append(back_pose)

        front_pose = PoseStamped()
        front_pose.header.frame_id = "base"
        front_pose.pose.position.x = x[2] # front 
        front_pose.pose.position.y = (y[i] + y[i+1])/2 
        front_pose.pose.position.z = z
        front_pose.pose.orientation.y = -1.0
        cup_poses[0].append(front_pose)

    planner = PathPlanner("right_arm")
    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    limb = Limb('right')

    controller = Controller(Kp, Kd, Ki, Kw, limb)
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    planner.remove_obstacle('table')
    planner.remove_obstacle('wall')
    planner.remove_obstacle('ar_marker_5')
    planner.remove_obstacle('ar_marker_6')
    planner.remove_obstacle('ar_marker_8')
    planner.remove_obstacle('shell_1')
    planner.remove_obstacle('shell_2')
    planner.remove_obstacle('shell_3')

    size = np.array([.4, 1.2, 0.1])
    table_pos = PoseStamped()
    table_pos.pose.position.x = 0.662
    table_pos.pose.position.y = -0.108
    table_pos.pose.position.z =  -0.189
    table_pos.header.frame_id = "base";
    planner.add_box_obstacle(size, 'table', table_pos)

    wall_size = np.array([0.1, 10.0, 10.0])
    wall_pose = PoseStamped() 
    wall_pose.pose.position.x = -0.748
    wall_pose.pose.position.y = 0.475
    wall_pose.pose.position.z = 0.536
    wall_pose.header.frame_id = "base";
    planner.add_box_obstacle(wall_size, 'wall', wall_pose)


    # get middle cup 
    cup_size = np.array([0.12, 0.05]) 

    planner.add_cylinder_obstacle(cup_size, 'cup_1', cup_poses[1][0])
    planner.add_cylinder_obstacle(cup_size, 'cup_2', cup_poses[1][2])
    
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.open() 
    plan = planner.plan_to_pose(cup_poses[1][1], [], [])
    print('going to get middle shell')
    #controller.execute_plan(plan[1])              #move to shell 1

    right_gripper.close()  
    # switch middle and right 
    # middle cup: goes from 1, 1 to 0, 0 
    s = cup_poses[1][1]
    e = cup_poses[0][0] 

    slope = (e.pose.position.y - s.pose.position.y)/(e.pose.position.x - s.pose.position.x)
    # y = m(x - x1) + y1
    func = lambda x: slope * (x - s.pose.position.x) + s.pose.position.y 
    

    increment = 0.005 
    p = s 
    names = limb.joint_names()

    print("DIAGONAL")
    while p.pose.position.x < e.pose.position.x and p.pose.position.y < e.pose.position.y: 
        p.pose.position.y = func(p.pose.position.x + increment)
        p.pose.position.x = p.pose.position.x + increment 
        # print(p)

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper_tip"
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        request.ik_request.pose_stamped = p 

        response = compute_ik(request)
        theta_list = [response.solution.joint_state.position[0]]
        theta_list.extend(response.solution.joint_state.position[2:8])
        theta_dict = dict(zip(names, theta_list))
        #set_j(limb, theta_dict)
        rospy.sleep(1)
        # print("___________________________")

    right_gripper.open() 

    # move up 
    print("MOVE UP")
    p = e
    while p.pose.position.z < p.pose.position.z + 0.1:
        p.pose.position.z = p.pose.position.z + increment 

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper_tip"
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        request.ik_request.pose_stamped = p 

        response = compute_ik(request)
        if response.error_code.val != 1: 
            break 
        theta_list = [response.solution.joint_state.position[0]]
        theta_list.extend(response.solution.joint_state.position[2:8])
        theta_dict = dict(zip(names, theta_list))
        #set_j(limb, theta_dict)
        rospy.sleep(1)

    # get right cup 
    print("get right cup")
    planner.remove_obstacle('cup_1')
    planner.add_cylinder_obstacle(cup_size, 'cup_3', cup_poses[0][0])
    plan = planner.plan_to_pose(cup_poses[1][0], [], [])
    print('going to get middle shell')
    # controller.execute_plan(plan[1])              #move to shell 1

    right_gripper.close() 

    # right cup goes from 1, 0 to 1, 1 
    p = cup_poses[1][0]
    e = cup_poses[1][1] 

    print("move right cup left")
    while p.pose.position.y > e.pose.position.y: 
        p.pose.position.y = p.pose.position.y - increment

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper_tip"
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        request.ik_request.pose_stamped = p 

        response = compute_ik(request)
        if response.error_code.val != 1: 
            print("ERROR")
            break 
        theta_list = [response.solution.joint_state.position[0]]
        theta_list.extend(response.solution.joint_state.position[2:8])
        theta_dict = dict(zip(names, theta_list))
        set_j(limb, theta_dict)
        rospy.sleep(1)
        # print("___________________________")
    
    # middle cup goes from 0, 0 to 1, 0 


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
