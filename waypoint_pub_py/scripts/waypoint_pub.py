#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf

# Python
import math
import numpy as np


transforms = Transform()
transforms.translation.x = 10.0
transforms.translation.y = 10.0
transforms.translation.z = 2.0
transforms.rotation.x = 0.0
transforms.rotation.y = 0.0
transforms.rotation.z = 0.0
transforms.rotation.w = 1.0

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

point = MultiDOFJointTrajectoryPoint()
point.transforms = [transforms]
point.velocities = [velocities]
point.accelerations = [acclerations]
point.time_from_start = rospy.Duration(1.0)



def way_point_publisher():
    way_point_pub = rospy.Publisher('red/position_hold/trajectory', MultiDOFJointTrajectoryPoint, queue_size=10)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('way_point_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        way_point_pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    try:
        way_point_publisher()
    except rospy.ROSInterruptException:
        pass
