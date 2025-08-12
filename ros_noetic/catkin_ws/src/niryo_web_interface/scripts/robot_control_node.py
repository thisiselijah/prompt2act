#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from pyniryo import NiryoRobot
from pyniryo import PoseObject

# --- 建立與 Niryo One 的連線 ---
niryo = NiryoRobot("10.10.10.10")  # 請改成你的 Niryo IP

print("執行自動校正...")
niryo.calibrate_auto()
print("自動校正完成！")

# --- ROS 訂閱者回呼函式 ---
def callback(msg):
    command = msg.data
    rospy.loginfo(f"收到指令: {command}")

    try:
        if command == "open":
            niryo.open_gripper()
        elif command == "close":
            niryo.close_gripper()
        elif command == "learning_mode":
            niryo.set_learning_mode(True)  # 進入 Learning Mode

        elif command == "move_to_home":
            niryo.move_to_home_pose()  # 呼叫 pyniryo 的 home 位置函式

        elif command.startswith("pick_at:"):
            # 解析指令格式
            parts = command.replace("pick_at:", "").split(",")
            if len(parts) != 3:
                raise ValueError("格式錯誤，應為 pick_at:x,y")

            x = float(parts[0])
            y = float(parts[1])
            z = 0.163  # 抓取高度（公尺）
            approach_z = z + 0.1  # 接近高度

            # 從 detected_target 取出 roll，沒有就用預設值
            roll = float(parts[2])
            pitch = 1.438
            yaw = -0.35

            rospy.loginfo(f"🚀 移動至 ({x:.2f}, {y:.2f}) 上方準備抓取，roll={roll:.3f}")
            niryo.open_gripper()

            pose_obj = PoseObject(x, y, z, roll, pitch, yaw)

            niryo.move_pose(pose_obj)

            niryo.close_gripper()

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
