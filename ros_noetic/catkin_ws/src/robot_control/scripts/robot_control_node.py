#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool, String
from robot_control.srv import RobotCommand, RobotCommandResponse
from pyniryo import NiryoRobot
from pyniryo import PoseObject

# --- Service section ---
def robot_control_service(niryo):
    """
    Create ROS service to control the robot
    """
    def handle_control_robot(req):
        """
        Service callback to control the robot based on request
        Args:
            req: Service request containing command in req.command
        Returns:
            RobotCommandResponse: success status and message
        """
        command = req.command
        rospy.loginfo(f"Received command: {command}")
        
        try:
            # === PRIMITIVE ACTIONS ===
            
            # 1. Gripper Control Actions
            if command == "open_gripper":
                niryo.open_gripper()
                rospy.loginfo("Gripper opened")
                
            elif command == "close_gripper":
                niryo.close_gripper()
                rospy.loginfo("Gripper closed")

            # 2. Movement Actions
            elif command == "get_current_pose":
                current_pose = niryo.get_pose()
                rospy.loginfo(f"Current pose: {current_pose}")
                return RobotCommandResponse(success=True, message=f"Current pose: {current_pose}")
            
            elif command == "move_to_home_and_sleep":
                niryo.go_to_sleep()
                rospy.loginfo("Moved to home position")
                
            elif command.startswith("move_to_pose:"):
                # Format: move_to_pose:x,y,z,roll,pitch,yaw
                parts = command.replace("move_to_pose:", "").split(",")
                if len(parts) != 6:
                    raise ValueError("Format error, should be move_to_pose:x,y,z,roll,pitch,yaw")

                x, y, z, roll, pitch, yaw = map(float, parts)
                pose_obj = PoseObject(x, y, z, roll, pitch, yaw)
                niryo.move_pose(pose_obj)
                rospy.loginfo(f"Moved to pose ({x:.2f}, {y:.2f}, {z:.2f})")
                
            # 3. Pick and Place Actions
            elif command.startswith("pick_at:"):
                # Format: pick_at:x,y,roll (z is fixed for table surface)
                parts = command.replace("pick_at:", "").split(",")
                if len(parts) != 3:
                    raise ValueError("Format error, should be pick_at:x,y,roll")

                x = float(parts[0])
                y = float(parts[1])
                z = 0.163  # 抓取高度（公尺）
                roll = float(parts[2])
                pitch = 1.438
                yaw = -0.35

                rospy.loginfo(f"Picking at ({x:.2f}, {y:.2f}), roll={roll:.3f}")
                niryo.open_gripper()
                pose_obj = PoseObject(x, y, z, roll, pitch, yaw)
                niryo.move_pose(pose_obj)
                niryo.close_gripper()
                rospy.loginfo("Pick completed")
                
            elif command.startswith("place_at:"):
                # Format: place_at:x,y,z,roll,pitch,yaw
                parts = command.replace("place_at:", "").split(",")
                if len(parts) != 6:
                    raise ValueError("Format error, should be place_at:x,y,z,roll,pitch,yaw")

                x, y, z, roll, pitch, yaw = map(float, parts)
                rospy.loginfo(f"Placing at ({x:.2f}, {y:.2f}, {z:.2f})")
                pose_obj = PoseObject(x, y, z, roll, pitch, yaw)
                niryo.move_pose(pose_obj)
                niryo.open_gripper()
                rospy.loginfo("Place completed")
                
            # 4. Robot State Actions
            elif command == "enable_learning_mode":
                niryo.set_learning_mode(True)
                rospy.loginfo("Learning mode enabled")
                
            elif command == "disable_learning_mode":
                niryo.set_learning_mode(False)
                rospy.loginfo("Learning mode disabled")
                
            elif command == "calibrate":
                niryo.calibrate_auto()
                rospy.loginfo("Robot calibrated")
                
            # 5. Safety Actions
            elif command == "stop":
                niryo.stop_move()
                rospy.loginfo("🛑 Robot stopped")
                
            elif command == "release_gripper":
                niryo.release_with_tool()
                rospy.loginfo("Gripper released")
                
            # 6. Check Actions (for behavior tree conditions)
            elif command == "check_calibration":
                is_calibrated = niryo.need_calibration()
                if is_calibrated:
                    rospy.loginfo("Robot is calibrated")
                else:
                    rospy.logwarn("Robot is not calibrated")
                return RobotCommandResponse(success=is_calibrated, 
                                          message=f"Calibration status: {is_calibrated}")
                
            else:
                return RobotCommandResponse(success=False, message=f"Unknown command: {command}")
            
            # If we reach here, command was successful
            return RobotCommandResponse(success=True, message=f"Command '{command}' executed successfully")

        except Exception as e:
            rospy.logerr(f"Error during robot control: {e}")
            return RobotCommandResponse(success=False, message=f"Error: {e}")

    return handle_control_robot


# --- ROS 訂閱者回呼函式 ---
def callback(msg, niryo):
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
        
def main():
    """
    Main function to initialize the ROS node and services
    """
    
    NIRYOROBOT_IP = "192.168.232.26"
    
    niryo = ''
    
    try:
        niryo = NiryoRobot(NIRYOROBOT_IP)  # Connect to the robot
    except Exception as e:
        rospy.logerr(f"Failed to connect to Niryo One at {NIRYOROBOT_IP}: {e}")
        return
    
    # 檢查並校準機器人
    if not niryo.need_calibration():
        rospy.loginfo("Niryo One is not calibrated, starting calibration...")
        niryo.calibrate_auto()
    rospy.loginfo("Calibration completed!")

    # 初始化 ROS 節點
    rospy.init_node('robot_control_node')
    
    # 創建服務
    service = rospy.Service('/niryo_arm_command_service', RobotCommand, robot_control_service(niryo))
    rospy.loginfo("Robot control service started")
    
    # 保留訂閱者作為備用
    rospy.Subscriber('/niryo_arm_command', String, callback(niryo))
    
    rospy.spin()



# --- ROS Node 初始化 ---
if __name__ == '__main__':
    main()
    
