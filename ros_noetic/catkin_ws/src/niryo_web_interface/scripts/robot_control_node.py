#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pyniryo import NiryoRobot
from pyniryo import PoseObject

# --- 建立與 Niryo One 的連線 ---
niryo = NiryoRobot("192.168.232.26")  # 請改成你的 Niryo IP

print("執行自動校正...")
niryo.calibrate_auto()
print("自動校正完成！")

# --- ROS 訂閱者回呼函式 ---
def callback(msg):
    command = msg.data
    rospy.loginfo(f"收到指令: {command}")

    try:
        if command == "left":
            joints = niryo.get_joints()
            joints[0] += 0.1  # 左旋轉
            niryo.move_joints(joints)

        elif command == "right":
            joints = niryo.get_joints()
            joints[0] -= 0.1  # 右旋轉
            niryo.move_joints(joints)

        elif command == "learning_mode":
            niryo.set_learning_mode(True)  # 進入 Learning Mode

        elif command == "move_to_home":
            niryo.move_to_home_pose()  # 呼叫 pyniryo 的 home 位置函式

        elif command.startswith("pick_at:"):
            # 解析指令格式
            parts = command.replace("pick_at:", "").split(",")
            if len(parts) != 2:
                raise ValueError("格式錯誤，應為 pick_at:x,y")

            x = float(parts[0])
            y = float(parts[1])
            z = 0.2  # 抓取高度（公尺）
            approach_z = z + 0.1  # 接近高度

            roll = 0
            pitch = 1.53
            yaw = 0


            rospy.loginfo(f"🚀 移動至 ({x:.2f}, {y:.2f}) 上方準備抓取")

            pose_obj = PoseObject(x,y,z,roll,pitch,yaw)

            niryo.move_pose(pose_obj)

            rospy.loginfo("✅ 抓取完成")


        else:
            rospy.logwarn(f"未知指令: {command}")

    except Exception as e:
        rospy.logerr(f"❌ 執行動作時發生錯誤: {e}")


# --- ROS Node 初始化 ---
if __name__ == '__main__':
    rospy.init_node('robot_control_node')
    rospy.Subscriber('/niryo_arm_command', String, callback)
    rospy.spin()
