#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf 
import math
import actionlib
import std_srvs.srv
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Quaternion
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan

class Navigation:
    def __init__(self):
        rospy.Subscriber('/navigation/memorize_place', String, self.getMemorizePlaceCB)
        rospy.Subscriber('/navigation/destination', String, self.getDestinationCB)
        rospy.Subscriber('/tf', TFMessage, self.setLocationList)
        
        self.navigation_result_pub = rospy.Publisher('/navigation/result', Bool, queue_size = 1)
        
        self.location_name = 'Null'
        self.location_list = []
        self.location_pose_x = 0
        self.location_pose_y = 0
        self.location_pose_w = 0
        self.destination = 'Null'

    def getMemorizePlaceCB(self, receive_msg):
        self.location_name = receive_msg.data

    def getDestinationCB(self, receive_msg):
        self.destination = receive_msg.data

    def waitTopic(self):#------------------------------------------------------state 0
        while not rospy.is_shutdown():
            if self.location_name != 'Null':
                rospy.loginfo("Start LocationList setup")
                return 1
            if self.destination != 'Null':
                rospy.loginfo("Start navigation")
                return 2
            else :
               #rospy.loginfo("Waiting for topic...")
                return 0

    def setLocationList(self, sub_tf):#----------------------------------------state 1
        pose = sub_tf.data
        if pose.transforms[0].header.frame_id == 'odom':
            self.location_pose_x = pose.transforms[0].transform.translation.x
            self.location_pose_y = pose.transforms[0].transform.translation.y
            self.location_pose_w = pose.transforms[0].transform.rotation.z
            self.location_pose_w += 1.5 * self.location_pose_w * self.location_pose_w *self.location_pose_w
            self.location_list.append([self.location_name, self.location_pose_x, self.location_pose_y, self.location_pose_w])
            rospy.loginfo("Add" + self.location_name + "to the LocationList")
            self.location_name = 'Null'
            result = Bool()
            result.data = True
            self.navigation_result_pub.publish(result)
            return 0

    def navigateToDestination(self):#------------------------------------------state 2
        location_num = -1
        for i in range(len(self.location_list)):
            if self.destination == self.location_list[i][0]:
                location_num = i
        if location_num == -1:
            rospy.loginfo("Not found Destination")
            result = Bool()
            result.data = False
            self.navigation_result_pub.publish(result)
            return 0
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if ac.wait_for_server(rospy.Duration(5)) == 1:
            rospy,loginfo("Waiting for action client comes up...")
        goal = MoveBaseGoal
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.location_list[location_num][1]
        goal.target_pose.pose.position.y = self.location_list[location_num][2]
        q = tf.transformations.quaternion_from_euler(0, 0, self.location_list[location_num][3])
        goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        ac.send_goal(goal)
        while not rospy.is_shutdown():
            #if ac.get_state() == 1:
            #    rospy.loginfo("Got out of the obstacle")
            #    rospy.sleep(1.0)
            if ac.get_state() == 2:
                rospy.loginfo("Goal")
                self.destination = 'Null'
                result = Bool()
                result.data = True
                self.navigation_result_pub.publish(result)
                rospy.sleep(2.0)
                return 0
            if ac.get_state() == 4:
                rospy.loginfo("Buried in obstacle")
                rospy.sleep(2.0)
                return 3

    def clearCostmap(self):#-----------------------------------------------state 3
        while not rospy.is_shutdown():
            rospy.wait_for_service('move_base/clear_costmap')
            clear_costmap = rospy.ServiceProxy('move_base/clear_costmap', std_srvs.srv.Empty)
            rospy.sleep(5.0)
            clear_costmap()
            return 2

if __name__ == '__main__':
    rospy.init_node('sg_navigation', anonymous = True)
    nav = Navigation()
    state = 0
    while not rospy.is_shutdown():
        if state == 0:
           state = nav.waitTopic()
        if state == 1:
            state = nav.setLocationList()
        if state == 2:
            state = nav.navigateToDestination()
        if state == 3:
            state = nav.clearCostmap()
        rospy.spin()
