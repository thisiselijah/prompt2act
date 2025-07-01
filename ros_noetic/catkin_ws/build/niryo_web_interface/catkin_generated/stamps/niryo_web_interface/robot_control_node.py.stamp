#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pyniryo import NiryoRobot

niryo = NiryoRobot("192.168.x.x")  # 請改成你的 IP

print("執行自動校正...")
niryo.calibrate_auto()
print("自動校正完成！")

def callback(msg):
    command = msg.data
    rospy.loginfo(f"收到指令: {command}")

    if command == "left":
        joints = niryo.get_joints()
        joints[0] += 0.1
        niryo.move_joints(joints)
    elif command == "right":
        joints = niryo.get_joints()
        joints[0] -= 0.1
        niryo.move_joints(joints)
    elif command == "learning_mode":
        niryo.change_learning_mode(True)
    else:
        rospy.loginfo(f"未知指令: {command}")

if __name__ == '__main__':
    rospy.init_node('robot_control_node')
    rospy.Subscriber('/niryo_arm_command', String, callback)
    rospy.spin()
