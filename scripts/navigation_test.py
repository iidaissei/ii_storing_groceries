#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool

class NavigationTest:
    def __init__(self):
        self.navigation_memorize_pub = rospy.Publisher('/navigation/memorize_place', String, queue_size = 1)
        self.navigation_command_pub = rospy.Publisher('/navigation/destination', String, queue_size = 1)

        rospy.Subscriber('/navigation/result', Bool, self.subscribeTopic)

    def pubA(self):
        while not rospy.is_shutdown():
            self.navigation_memorize_pub.publish('cupboard')
            rospy.loginfo("Published")
    
    def pubB(self):        
        while not rospy.is_shutdown():
            self.navigation_command_pub.publish('cupboard')
            rospy.loginfo("Published")

    def subscribeTopic(self, receive_msg):
        sub = receive_msg.data
        print '',sub

if __name__ == '__main__':
    rospy.init_node('navigation_test', anonymous = True)
    while not rospy.is_shutdown():
        nav_test = NavigationTest()
        print 'Please input a or b'
        result = raw_input('>> ')
        if result == 'a':
            nav_test.pubA()
        if result == 'b':
            nav_test.pubB()
