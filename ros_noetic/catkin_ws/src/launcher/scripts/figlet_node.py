#!/usr/bin/env python3

import rospy
import subprocess
import sys

def figlet_node():
    """Simple ROS node that prints project name with figlet and exits"""
    rospy.init_node('figlet_node', anonymous=False)
    
    try:
        # Clear terminal first
        subprocess.run(['clear'], check=True)
        
        # Print "prompt2act" with figlet
        subprocess.run(['figlet', 'prompt2act'], check=True)
        # rospy.loginfo("Figlet banner displayed successfully")
    except subprocess.CalledProcessError:
        rospy.logwarn("figlet command failed - is figlet installed?")
    except FileNotFoundError:
        rospy.logwarn("figlet not found - is figlet installed?")
    
    # Exit after printing (we don't need this node to keep running)
    rospy.signal_shutdown("Banner displayed")

if __name__ == '__main__':
    try:
        figlet_node()
    except rospy.ROSInterruptException:
        pass
