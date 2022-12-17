#!/usr/bin/env python
from intera_interface import Limb
import rospy
from intera_interface import gripper as robot_gripper
import tf2_ros

from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

INCREMENT_SIZE = 0.005 
SLEEP = 0.1 
COMPUTE_IK = rospy.ServiceProxy('compute_ik', GetPositionIK)
EPS = 0.001 

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

def point_to_pose(coords):
    pose = PoseStamped() 
    pose.pose.position.x = coords[0]
    pose.pose.position.y = coords[1]
    pose.pose.position.z = coords[2]
    pose.pose.orientation.y = -1.0
    pose.header.frame_id = "base";
    return pose

def set_j(limb, theta_dict):
    print("Executing" + str(theta_dict))
    limb.set_joint_position_speed(0.1)
    limb.set_joint_positions(theta_dict)

def get_cup_poses(x, y, z): 
    cup_poses = [[], [], []] 

    # middle row 
    for y_val in y: 
        cup_poses[1].append([x[1], y_val, z])

    # front and back 
    for i in range(2): 
        cup_poses[2].append([x[0], (y[i] + y[i+1])/2, z])
        cup_poses[0].append([x[2], (y[i] + y[i+1])/2 , z])

    return cup_poses

def move_y(s, e, limb): 
    names = limb.joint_names()
    x, z = s[0], s[2]
    
    p = point_to_pose(s)
    e_p = point_to_pose(e)

    y_diff = e[1] - s[1] 
    inc = y_diff / abs(y_diff) * INCREMENT_SIZE 
    
    if inc < 0: 
        condition = lambda a, b: a.pose.position.y > b.pose.position.y
    else: 
        condition = lambda a, b: a.pose.position.y < b.pose.position.y

    while condition(p, e_p):
        p.pose.position.y = p.pose.position.y + inc 
        p.pose.position.x = x 
        p.pose.position.z = z  

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper_tip"
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        request.ik_request.pose_stamped = p 

        response = COMPUTE_IK(request)
        if response.error_code.val != 1: 
            print("ERROR: " + str(response.error_code.val))     
            continue  

        theta_list = [response.solution.joint_state.position[0]]
        theta_list.extend(response.solution.joint_state.position[2:8])
        theta_dict = dict(zip(names, theta_list))
        set_j(limb, theta_dict)
        rospy.sleep(SLEEP)

def move_xy(s, e, limb): # in list form not pose 

    names = limb.joint_names()

    x_diff = e[0] - s[0] 
    
    if abs(x_diff) <= EPS:
        move_y(s, e, limb)
        return 

    inc = x_diff / abs(x_diff) * INCREMENT_SIZE 

    slope = (e[1] - s[1])/(e[0] - s[0])
    func = lambda x: slope * (x - s[0]) + s[1]
    z = s[2]

    p = point_to_pose(s)
    e_p = point_to_pose(e)

    condition = None 
    if inc < 0: 
        condition = lambda a, b: a.pose.position.x > b.pose.position.x 
    else: 
        condition = lambda a, b: a.pose.position.x < b.pose.position.x 

    while condition(p, e_p): 
        p.pose.position.y = func(p.pose.position.x + inc)
        p.pose.position.x = p.pose.position.x + inc 
        p.pose.position.z = z 

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper_tip"
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        request.ik_request.pose_stamped = p 

        response = COMPUTE_IK(request)
        if response.error_code.val != 1:
            print("ERROR: " + str(response.error_code.val))     
            continue  

        theta_list = [response.solution.joint_state.position[0]]
        theta_list.extend(response.solution.joint_state.position[2:8])
        theta_dict = dict(zip(names, theta_list))
        set_j(limb, theta_dict)
        rospy.sleep(SLEEP)

def move_z(s, e, limb): 
    names = limb.joint_names()
    x, y = s[0], s[1]
    
    p = point_to_pose(s)
    e_p = point_to_pose(e)

    z_diff = e[2] - s[2] 
    inc = z_diff / abs(z_diff) * INCREMENT_SIZE 
    
    if inc < 0: 
        condition = lambda a, b: a.pose.position.z > b.pose.position.z
    else: 
        condition = lambda a, b: a.pose.position.z < b.pose.position.z

    while condition(p, e_p):
        p.pose.position.z = p.pose.position.z + inc 
        p.pose.position.x = x 
        p.pose.position.y = y  

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper_tip"
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        request.ik_request.pose_stamped = p 

        response = COMPUTE_IK(request)
        if response.error_code.val != 1: 
            print("ERROR: " + str(response.error_code.val))     
            continue  

        theta_list = [response.solution.joint_state.position[0]]
        theta_list.extend(response.solution.joint_state.position[2:8])
        theta_dict = dict(zip(names, theta_list))
        set_j(limb, theta_dict)
        rospy.sleep(SLEEP)

def switch_adjacent(c1, c2, limb, cup_poses):

    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.open() 

    assert c1 == 1 or c2 == 1 
    other_num = c1 if c2 == 1 else c2 
    int_num = 0 if other_num == 0 else 1 

    middle_cup = cup_poses[1][1]
    int_cup = cup_poses[0][int_num]
    other_cup = cup_poses[1][other_num]

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]

    move_xy(tip_coord, middle_cup, limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_z(tip_coord, middle_cup, limb)

    x = input() 
    right_gripper.close() 

    # Switch  middle and right cups  
    # middle cup: goes from 1, 1 to 0, 0
    move_xy(middle_cup, int_cup, limb)

    right_gripper.open() 
    rospy.sleep(1)

    up_dest = int_cup[:]
    up_dest[2] += 0.1
    move_z(int_cup, up_dest, limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_xy(tip_coord, other_cup, limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_z(tip_coord, other_cup, limb)

    x = input() 
    right_gripper.close() 

    move_y(other_cup, middle_cup, limb)

    right_gripper.open()
    rospy.sleep(1) 

    up_dest = middle_cup[:]
    up_dest[2] += 0.1
    move_z(middle_cup, up_dest, limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_xy(tip_coord, int_cup, limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_z(tip_coord, int_cup, limb)

    x = input()

    right_gripper.close() 

    move_xy(int_cup, other_cup, limb)

    right_gripper.open() 
    rospy.sleep(1)

    up_dest = other_cup[:]
    up_dest[2] += 0.3
    move_z(other_cup, up_dest, limb)

def switch_outside(limb, cup_poses):
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.open() 

    right_cup = cup_poses[1][0]
    left_cup = cup_poses[1][2]

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]

    move_xy(tip_coord, left_cup, limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_z(tip_coord, left_cup, limb)

    x = input() 
    right_gripper.close() 

    # Switch  middle and right cups  
    # middle cup: goes from 1, 1 to 0, 0
    move_xy(left_cup, cup_poses[0][1], limb)

    move_y(cup_poses[0][1], cup_poses[0][0], limb)

    right_gripper.open() 
    rospy.sleep(1)

    up_dest = cup_poses[0][0][:]
    up_dest[2] += 0.1
    move_z(cup_poses[0][0], up_dest, limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_xy(tip_coord, right_cup, limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_z(tip_coord, right_cup, limb)

    x = input() 
    right_gripper.close() 

    move_xy(right_cup, cup_poses[2][0], limb)

    move_y(cup_poses[2][0], cup_poses[2][1], limb)

    move_xy(cup_poses[2][1], left_cup, limb)

    right_gripper.open()
    rospy.sleep(1) 

    up_dest = left_cup[:]
    up_dest[2] += 0.1
    move_z(left_cup, up_dest, limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_xy(tip_coord, cup_poses[0][0], limb)

    tip = get_pose('right_gripper_tip')
    tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    move_z(tip_coord, cup_poses[0][0], limb)

    x = input()

    right_gripper.close() 

    move_xy(cup_poses[0][0], right_cup, limb)

    right_gripper.open() 
    rospy.sleep(1)

    up_dest = other_cup[:]
    up_dest[2] += 0.3
    move_z(other_cup, up_dest, limb)

# Note: Need to push from lab computer 
def main():
    """
    Main Script
    """

    x = [0.572, 0.715, 0.852]
    z = -0.1
    y = [0.264, 0.089, -0.083]
    cup_poses = get_cup_poses(x, y, z)

    limb = Limb('right')
    for row in cup_poses: 
        print(row)

    # tip = get_pose('right_gripper_tip')
    # tip_coord = [tip.pose.position.x, tip.pose.position.y, tip.pose.position.z]
    # move_xy(tip_coord, cup_poses[1][2], limb)

    #switch_outside(limb, cup_poses)
    switch_adjacent(0, 1, limb, cup_poses)
    #switch_adjacent(1, 2, limb, cup_poses)

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
