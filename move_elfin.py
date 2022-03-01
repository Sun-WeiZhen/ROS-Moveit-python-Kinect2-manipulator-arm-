#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs
import tf

import cv2 as cv
import numpy as np

from moveit_commander import MoveGroupCommander

#from moveit_python import (MoveGroupInterface,PlanningSceneInterface,PickPlaceInterface)
import moveit_msgs.msg
from geometry_msgs.msg import Pose,PoseStamped
from copy import deepcopy
moveit_commander.roscpp_initialize(sys.argv)
cartesian = rospy.get_param('~cartesian', True)#dikaer coordinate

arm_group = moveit_commander.move_group.MoveGroupCommander("elfin_arm")#设置规划组名称
arm_group.allow_replanning(True)#允许再次规划

end_effector_link = arm_group.get_end_effector_link() #获取末端执行器名称
reference_frame = 'elfin_base_link'  # 设置参考坐标系名称
arm_group.set_pose_reference_frame(reference_frame)  # 将参考坐标系设置为 elfin_base_link

arm_group.set_goal_position_tolerance(0.001)
arm_group.set_goal_orientation_tolerance(0.001)

arm_group.set_max_acceleration_scaling_factor(0.15)#shezhi 
arm_group.set_max_velocity_scaling_factor(0.15)#

arm_group.set_start_state_to_current_state()
#pose_target = arm_group.get_current_pose().pose


def pressure_detect(press):
    #这个函数是预留给深度学习模型的
    if press>25:
        up_Z()

def detect_Net(I_US):
    #这个函数是预留给检测模型的
    #这个模型会计算出莫目标中心距离图像中心的距离，然后根据这个距离判断探头的移动
    src = I_US#代检测图像
    img = src.copy()
    src_roi = cv.imread("E:\\roi.png")#需要检测目标的模板
    roi = src_roi.copy()
    # 模板匹配
    res = cv.matchTemplate(img, roi, cv.TM_CCOEFF_NORMED)
    # 返回模板中最匹配的位置，确定左上角的坐标
    min_Val, max_Val, min_Loc, max_Loc = cv.minMaxLoc(res)
    # 使用相关系数匹配时，最大值为最佳匹配位置
    top_left = max_Loc
    roi_rows, roi_cols = roi.shape[:2]
    nx = top_left[0]+roi_cols
    ny = top_left[1]+roi_rows
    cv.rectangle(img, top_left, (nx, ny), (0, 0, 255), 3)#参数分别是：左上角坐标，右下角坐标，图像颜色，线条粗细。
    #这个是用来画检测到目标的举行框，
    mx = top_left[0]+round(roi_cols*0.5)
    my = top_left[1]+round(roi_rows*0.5)
    cv2.putText(img, (str(mx),str(my)), (mx, my), font, 0.5, (255, 255, 255), 3)
    #显示目标中心坐标。参数分别为：原图，文字，文字坐标，字体，字体大小，字体颜色，字体粗细
    size = src.shape
    jud = mx - size[0]
    jud_abs = abs(jud)
    return jud,jud_abs

    # 显示图像
    #cv.namedWindow('input_image', cv.WINDOW_AUTOSIZE)
    #cv.imshow('input_image', img)
    #cv.waitKey(0)
    #cv.destroyAllWindows()



def up_Z():
    #控制机械臂向上移动
    #z这个函数用于判断当超声探头压力过大时，
    object_position_info = pose.position
    object_orientation_info = pose.orientation
    pose_target = arm_group.get_current_pose(end_effector_link).pose
    waypoints = []
    wpose = deepcopy(pose_target)#复制机械臂末端但前位置和姿态
    #--------------------------------------------------------------------
    # 先向上移动
    #--------------------------------------------------------------------
    wpose.position.z = wpose.position.z+0.02#修改z轴方向的移动
    if cartesian:  
        waypoints.append(deepcopy(wpose))
    else:          
        arm_group.set_pose_target(wpose)  
        arm_group.go()
        rospy.sleep(1)
    #--------------------------------------------------------------------
    # 路径规划
    #--------------------------------------------------------------------
    if cartesian:
        fraction = 0.0   
	maxtries = 100   
	attempts = 0     
	arm_group.set_start_state_to_current_state()
	while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm_group.compute_cartesian_path (waypoints,0.01,0.0,True)  
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
		        
	if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
	    arm_group.execute(plan)
            rospy.loginfo("Path execution complete.")
	else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + "success after " + str(maxtries) + "attempts.")  
	rospy.sleep(1)


def down_Z():
    #控制机械臂向下移动,这边需要结合压力传感器和像素质量传感器判断是否该向下移动
    #考虑到压力，我们将末端向下移动的距离调整为每次5毫米
    object_position_info = pose.position
    object_orientation_info = pose.orientation
    pose_target = arm_group.get_current_pose(end_effector_link).pose
    waypoints = []
    wpose = deepcopy(pose_target)#复制机械臂末端但前位置和姿态
    #--------------------------------------------------------------------
    # 先向下移动
    #--------------------------------------------------------------------
    wpose.position.z = wpose.position.z-0.005#修改z轴方向的移动
    if cartesian:  
        waypoints.append(deepcopy(wpose))
    else:          
        arm_group.set_pose_target(wpose)  
        arm_group.go()
        rospy.sleep(1)
    #--------------------------------------------------------------------
    # 路径规划
    #--------------------------------------------------------------------
    if cartesian:
        fraction = 0.0   
	maxtries = 100   
	attempts = 0     
	arm_group.set_start_state_to_current_state()
	while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm_group.compute_cartesian_path (waypoints,0.01,0.0,True)  
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
		        
	if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
	    arm_group.execute(plan)
            rospy.loginfo("Path execution complete.")
	else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + "success after " + str(maxtries) + "attempts.")  
	rospy.sleep(1)

def Trans_motion(dist_x=0,dist_y=0,dist_z=0,link = end_effector_link):
    #这个函数用于控制机械臂在xyz三个轴方方向上的平移运动，
    #dist_x=0,dist_y=0,dist_z=0分别表示xyz三个轴的移动分量，一般来讲我们之控制其中一个方向的移动，其他方向的莫认为0
    #因此我们采用默认参数的变量，这样只要在调用该函数的时候改变其中一个参数就可以了
    #并且是控制直线运动的，因为使用自由曲线很可能会导致机械臂末端绕来绕去很麻烦
    #为了防止直线移动过程中碰到别人，所以在每次移动之前要将探头末端上向移动一定距离

    object_position_info = pose.position
    object_orientation_info = pose.orientation
    pose_target = arm_group.get_current_pose(link).pose
    waypoints = []
    wpose = deepcopy(pose_target)#复制机械臂末端但前位置和姿态
    #--------------------------------------------------------------------
    # 先向上移动
    #--------------------------------------------------------------------
    wpose.position.z = wpose.position.z+0.02#修改z轴方向的移动
    if cartesian:  
        waypoints.append(deepcopy(wpose))
    else:          
        arm_group.set_pose_target(wpose)  
        arm_group.go()
        rospy.sleep(1)
    #--------------------------------------------------------------------
    # 根据逻辑需求，判断机械臂末端下意识可向哪移动
    #--------------------------------------------------------------------
    wpose.position.x = wpose.position.x+dist_x#修改x轴方向的移动
    wpose.position.y = wpose.position.y+dist_y#修改y轴方向的移动
    if cartesian:
        waypoints.append(deepcopy(wpose))
    else:
        arm_group.set_pose_target(wpose)
        arm_group.go()
        rospy.sleep(1)
    #--------------------------------------------------------------------
    # 先向下移动
    #--------------------------------------------------------------------
    wpose.position.z = wpose.position.z-0.02#修改z轴方向的移动
    if cartesian:  
        waypoints.append(deepcopy(wpose))
    else:          
        arm_group.set_pose_target(wpose)  
        arm_group.go()
        rospy.sleep(1)
    #--------------------------------------------------------------------
    # 路径规划
    #--------------------------------------------------------------------
    if cartesian:
        fraction = 0.0   
	maxtries = 100   
	attempts = 0     
	arm_group.set_start_state_to_current_state()
	while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm_group.compute_cartesian_path (waypoints,0.01,0.0,True)  
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
		        
	if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
	    arm_group.execute(plan)
            rospy.loginfo("Path execution complete.")
	else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + "success after " + str(maxtries) + "attempts.")  
	rospy.sleep(1)


def Rota_motion(angle_x=0,angle_y=0,angle_z=0,angle,link = end_effector_link):
    #控制机械臂末端的旋转，使用shift_pose_target
    #采用了根上面平移一样的变量使用策略，
    #angle_x=0,angle_y=0,angle_z=0分别表示xyz三个轴的旋转分量
    #由于旋转不需要考虑高度，因此直接旋转。没有多余操作
    #除了使用PoseStamped描述位姿并规划外，还可以使用shift_pose_target实现单轴方向上的目标设置与规划；
    #第一个参数：描述机器人在六个自由度中实现哪一种运动，0，1，2，3，4，5分别表示xyz三个方向的平移与旋转。
    #第二个参数：描述机器人移动或旋转的量，单位为m或者弧度。
    #第三个参数：描述该运动针对的对象。
    #link = end_effector_link

    #--------------------------------------------------------
    #首先判断需要旋转哪个轴
    #--------------------------------------------------------
    if angle_x != 0 and angle_y = 0 and angle_z = 0:
        indes_angle = 3
    elif angle_x = 0 and angle_y != 0 and angle_z = 0:
        indes_angle = 4
    else:
        indes_angle = 5
    #--------------------------------------------------------
    #控制机械臂末端旋转
    #--------------------------------------------------------
    arm.shift_pose_target(indes_angle, angle, link)
    arm.go()
    rospy.sleep(1)

def pose_callback(pose):
    #这个函数是一系列的逻辑判断
    #判断到底该如何移动探头
    a = 4
    b = 8
    if a >5:
        Trans_motion(dist_x=0,dist_y=0,dist_z=0,link = end_effector_link)




    arm_group.clear_pose_targets()
    print("clear.............") 
    #moveit_commander.roscpp_shutdown()

def object_position_sub():
    rospy.init_node('object_position_sub_And_grasp_node',anonymous=True)
    rospy.Subscriber("/objection_position_pose",Pose,pose_callback,queue_size=1)
    rospy.spin()
if __name__ == "__main__":
    object_position_sub()





















