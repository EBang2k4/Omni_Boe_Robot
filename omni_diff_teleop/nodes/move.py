#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class BoeBotController:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('boe_bot_controller', anonymous=True)
        
        # Publisher để gửi lệnh vận tốc trên topic /cmd_vel
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Khởi tạo vận tốc ban đầu
        self.linear_speed = 0.5  # Tốc độ tuyến tính ban đầu (m/s)
        self.angular_speed = 0.8  # Tốc độ góc ban đầu (rad/s) - tăng để quay nhanh hơn
        self.speed_increment = 0.1  # Bước tăng/giảm tốc độ cho q/z
        self.linear_increment = 0.2  # Bước tăng tốc độ khi tiếp tục nhấn w/s

        # Biến vận tốc
        self.move_cmd = Twist()

        # Biến để theo dõi trạng thái trước đó
        self.last_command = None

        # Thông báo khởi động
        rospy.loginfo("Điều khiển robot boe_bot:")
        rospy.loginfo("Nhập lệnh (w: Tiến, s: Lùi, x/Enter: Dừng, a: Xoay trái, d: Xoay phải, q: Tăng tốc, z: Giảm tốc, e: Thoát)")

    def control_robot(self):
        while not rospy.is_shutdown():
            # Nhận lệnh từ terminal
            command = input("Nhập lệnh: ").lower()

            # Đặt lại vận tốc
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0

            # Xử lý lệnh
            if command == 'w':  # Tiến
                if self.last_command == 'w':  # Nếu tiếp tục nhấn w
                    self.linear_speed += self.linear_increment
                    rospy.loginfo(f"Tăng tốc tiến: linear_speed={self.linear_speed}")
                self.move_cmd.linear.x = self.linear_speed
            elif command == 's':  # Lùi
                if self.last_command == 's':  # Nếu tiếp tục nhấn s
                    self.linear_speed += self.linear_increment
                    rospy.loginfo(f"Tăng tốc lùi: linear_speed={self.linear_speed}")
                self.move_cmd.linear.x = -self.linear_speed
            elif command == 'x':  # Dừng
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.0
            elif command == 'a':  # Xoay trái
                self.move_cmd.angular.z = self.angular_speed
            elif command == 'd':  # Xoay phải
                self.move_cmd.angular.z = -self.angular_speed
            elif command == 'q':  # Tăng tốc
                self.linear_speed += self.speed_increment
                self.angular_speed += self.speed_increment
                rospy.loginfo(f"Tăng tốc: linear_speed={self.linear_speed}, angular_speed={self.angular_speed}")
                continue
            elif command == 'z':  # Giảm tốc
                self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
                self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
                rospy.loginfo(f"Giảm tốc: linear_speed={self.linear_speed}, angular_speed={self.angular_speed}")
                continue
            elif command == 'e':  # Thoát
                rospy.loginfo("Đã thoát chương trình điều khiển robot.")
                break

            # Cập nhật trạng thái lệnh trước đó
            self.last_command = command

            # Xuất bản lệnh vận tốc
            for _ in range(10):  # Gửi lệnh trong 1 giây (10 Hz * 1 giây)
                self.pub.publish(self.move_cmd)
                self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = BoeBotController()
        controller.control_robot()
    except rospy.ROSInterruptException:
        pass