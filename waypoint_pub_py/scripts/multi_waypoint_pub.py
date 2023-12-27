#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

# Python
import math
import numpy as np


def generate_transforms():
    transforms_g = []
    for i in range(0, 4):
        transform = Transform()
        transform.translation.x = -2
        transform.translation.y = i*7
        transform.translation.z = 2
        transform.rotation.x = 0
        transform.rotation.y = 0
        transform.rotation.z = 0
        transform.rotation.w = 1
        transforms_g.append(transform)
    return transforms_g

# 调用函数
transforms_g = generate_transforms()


velocities = Twist()
velocities.linear.x = 1.0
velocities.linear.y = 1.0
velocities.linear.z = 1.0
velocities.angular.x = 1.0
velocities.angular.y = 1.0
velocities.angular.z = 1.0

acclerations = Twist()
acclerations.linear.x = 1.0
acclerations.linear.y = 1.0
acclerations.linear.z = 1.0
acclerations.angular.x = 1.0
acclerations.angular.y = 1.0
acclerations.angular.z = 1.0




def sub_pose(msg):
    global point
    way_point_pub = rospy.Publisher('red/position_hold/trajectory', MultiDOFJointTrajectoryPoint, queue_size=10)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # while not rospy.is_shutdown():

    position = msg.pose.pose.position
    next_pos = point.transforms[0]
    # 计算距离
    dx = position.x - next_pos.translation.x
    dy = position.y - next_pos.translation.y
    dz = position.z - next_pos.translation.z
    distance = math.sqrt(dx**2 + dy**2 + dz**2)

    hello_str = f"curr_pos is ({position.x},{position.y},{position.z}),\
        distance to ({next_pos.translation.x},{next_pos.translation.y},{next_pos.translation.z}) is {distance}"
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

    if(distance < 0.2):
        if len(transforms) > 0:
            togo_point = point.transforms.pop(0)
            rospy.loginfo("已经弹出transforms列表中的第一个元素")
            rospy.sleep(3.0)
        else:
            rospy.loginfo("transforms列表为空,无法弹出元素")

    way_point_pub.publish(point)

    rate.sleep()

def odometry_callback(msg):
    # 位姿信息在msg.pose.pose中
    global transforms_g
    global velocities
    global acclerations
    position = msg.pose.pose.position
    # rospy.loginfo("位置: x=%f, y=%f, z=%f", position.x, position.y, position.z)
    position.x = position.x -1
    position.y = position.y -1
    position.z = position.z
    # rospy.loginfo("位置: x=%f, y=%f, z=%f", position.x, position.y, position.z)

    
    
    # while not rospy.is_shutdown():
    if len(transforms_g) > 0:
        way_point = MultiDOFJointTrajectoryPoint()
        way_point.transforms = [transforms_g[0]]
        way_point.velocities = [velocities]
        way_point.accelerations = [acclerations]
        way_point.time_from_start = rospy.Duration(1.0)
    else:
        while not rospy.is_shutdown():
            rospy.loginfo("transforms列表为空,已经到终点")
            rospy.sleep(5.0)




    # 计算距离
    dx = position.x - way_point.transforms[0].translation.x
    dy = position.y - way_point.transforms[0].translation.y
    dz = position.z - way_point.transforms[0].translation.z
    distance = math.sqrt(dx**2 + dy**2 + dz**2)

    if(distance < 0.2):
        if len(transforms_g) > 0:
            pop_point = transforms_g.pop(0)
            rospy.loginfo("已经弹出transforms列表中的第一个元素")
            rospy.sleep(1.0)
            distance = 0.3
        else:
            rospy.loginfo("transforms列表为空,无法弹出元素")

    hello_str = f"curr_pos is ({position.x},{position.y},{position.z}),\
    distance to ({way_point.transforms[0].translation.x},\
    {way_point.transforms[0].translation.y},\
    {way_point.transforms[0].translation.z}) is {distance}"

    rospy.loginfo(hello_str)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub.publish(hello_str)

    way_point_pub = rospy.Publisher('red/position_hold/trajectory', MultiDOFJointTrajectoryPoint, queue_size=10)
    way_point_pub.publish(way_point)



def listener():
    rospy.init_node('odometry_listener', anonymous=True)
    rospy.Subscriber("/red/odometry", Odometry, odometry_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    # rospy.init_node('drone_pose_listener', anonymous=True)
    # rospy.Subscriber("/red/odometry", Odometry, sub_pose)
    # way_point_publisher()
    # rospy.spin()
    # try:
    #     way_point_publisher()
    # except rospy.ROSInterruptException:
    #     pass
