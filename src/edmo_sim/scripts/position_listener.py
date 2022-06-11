#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry


def callback(data, a):
    print("x: ", data.pose.pose.position.x)
    print("y: ", data.pose.pose.position.y)
    data_to_save = str(data.pose.pose.position.x) + "\t" + str(data.pose.pose.position.y) + "\n"
    a.write(data_to_save)

def listener():
    a = open('/home/shyngys/test_ws/goals.txt', 'w+')
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/edmo_snake/state', Odometry, callback, (a))
    rospy.spin()

if __name__ == '__main__':
    listener()