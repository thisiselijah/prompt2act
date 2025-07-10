#!/usr/bin/env python3
import rospy

def hello_world():
    rospy.init_node('hello_world_node', anonymous=True)
    rospy.loginfo("Hello World!")
    rospy.spin()  # Keeps the node running

if __name__ == '__main__':
    try:
        hello_world()
    except rospy.ROSInterruptException:
        pass