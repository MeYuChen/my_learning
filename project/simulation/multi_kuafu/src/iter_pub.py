#!/usr/bin/env python
#
# Copyright (c) 2022, Unity-Drive Inc. All rights reserved.
# Author: Zhenyu Li lizhenyu@unity-drive.com

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

goal_1 = PoseStamped()
#goal_1.header.timestamp_sec = rospy.get_time()
goal_1.header.frame_id = ""
goal_1.pose.position.x = 0
goal_1.pose.position.y = 0
goal_1.pose.position.z = 0
goal_1.pose.orientation.x = 0
goal_1.pose.orientation.y = 0
goal_1.pose.orientation.z = 0
goal_1.pose.orientation.w = 0

goal_2 = PoseStamped()
#goal_2.header.timestamp_sec = rospy.get_time()
goal_2.header.frame_id = ""
goal_2.pose.position.x = 0
goal_2.pose.position.y = 0
goal_2.pose.position.z = 0
goal_2.pose.orientation.x = 0
goal_2.pose.orientation.y = 0
goal_2.pose.orientation.z = 0
goal_2.pose.orientation.w = 0

flag = True

def status_callback(data):
    print("Ok")
    global flag
    if data.markers[-1].text == "[FREE_PARKING]":
        rospy.sleep(2)
        if flag:
            print("pub goal 1")
            pub.publish(goal_1)
            flag = False
        else:
            print("pub goal 2")
            pub.publish(goal_2)
            flag = True
        rospy.sleep(3)



if __name__ == '__main__':
    try:
        print("start")
        rospy.init_node('iter_pub', anonymous=True)
        pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        sub = rospy.Subscriber('motion_planning_infa/rviz/current_manuever', MarkerArray, status_callback, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass