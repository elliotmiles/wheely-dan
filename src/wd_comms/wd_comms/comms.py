import serial
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

radius = 0.0325 # wheel radius in metres
separation = 0.202
enc_cpr = 2800.0 # encoder counts per revolution (700 on datasheet but 4x counting, so 2800)


class Comms:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser_ = serial.Serial(port, baudrate, timeout=0.01)
        time.sleep(2)  # Wait for the serial connection to initialize

    def upload(self, data):
        # first converts all list elements to strings, then joins them into one continuous string separated by commas
        comm = ",".join(map(str, data))
        self.ser_.write(f"{comm}\n".encode())

    def close(self):
        self.ser_.close()

    def read(self):
        try:
            if self.ser_.in_waiting > 0:
                return self.ser_.readline().decode(errors='ignore').strip()
            return None

        except serial.SerialException as e:
            print(f"Serial error: {e}")
            return None

        except OSError as e:
            print(f"USB disconnected: {e}")
            return None
        
class RobotState:
    def __init__(self):
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.prev_pos_left_ = None
        self.prev_pos_right_ = None


class CommsNode(Node):
    def __init__(self):
        super().__init__('comms_node')

        self.comms_ = Comms()

        self.robot_state_ = RobotState()

        self.subscription_ = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.subscription_callback,
            10
        )
        self.imu_publisher_ = self.create_publisher(Imu, '/imu', 10)

        self.odom_publisher_ = self.create_publisher(Odometry, '/odom', 10)
    
        self.timer_ = self.create_timer(0.01, self.timer_callback)
    
    def subscription_callback(self, msg):
        self.msg_list_= [msg.twist.linear.x, msg.twist.angular.z]
        self.comms_.upload(self.msg_list_)

    def timer_callback(self):
        msg = self.comms_.read()

        if msg is not None:
            if msg.startswith("IMU,"):
                imu_msg = Imu()
                imu_data = msg.split(",")  # Extract the data after "IMU," and split by commas
                imu_msg.orientation.w = float(imu_data[1])  
                imu_msg.orientation.x = float(imu_data[2]) 
                imu_msg.orientation.y = float(imu_data[3]) 
                imu_msg.orientation.z = float(imu_data[4]) 
                imu_msg.angular_velocity.x = float(imu_data[5])
                imu_msg.angular_velocity.y = float(imu_data[6]) 
                imu_msg.angular_velocity.z = float(imu_data[7]) 
                imu_msg.linear_acceleration.x = float(imu_data[8]) 
                imu_msg.linear_acceleration.y = float(imu_data[9])  
                imu_msg.linear_acceleration.z = float(imu_data[10]) 
                self.imu_publisher_.publish(imu_msg)  # Publish the IMU data

            elif msg.startswith("Odom,"):
                # parse received data
                odom_data = msg.split(",")  # Extract the data after "Odom," and split by commas
                posLeft = float(odom_data[1])  
                posRight = float(odom_data[2]) 
                velLeft = float(odom_data[3]) 
                velRight = float(odom_data[4]) 

                if self.robot_state_.prev_pos_left_ is None:
                    self.robot_state_.prev_pos_left_ = posLeft
                    self.robot_state_.prev_pos_right_ = posRight
                    return


                deltaLeftCounts = posLeft - self.robot_state_.prev_pos_left_
                deltaRightCounts = posRight - self.robot_state_.prev_pos_right_

                self.robot_state_.prev_pos_left_ = posLeft
                self.robot_state_.prev_pos_right_ = posRight

                distLeft = (deltaLeftCounts/enc_cpr) * 2 * math.pi * radius
                distRight = (deltaRightCounts/enc_cpr) * 2 * math.pi * radius
                displacement = (distLeft + distRight) / 2

                deltaTheta = (distRight - distLeft) / separation


                self.robot_state_.x_ += displacement * math.cos(self.robot_state_.theta_ + deltaTheta/2)
                self.robot_state_.y_ += displacement * math.sin(self.robot_state_.theta_ + deltaTheta/2)

                self.robot_state_.theta_ += deltaTheta

                linVel = (radius * (velLeft + velRight)) / 2 
                angVel = (radius * (velRight - velLeft)) / separation




                # publish data to ros2 topic
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"

                odom_msg.pose.pose.position.x = self.robot_state_.x_
                odom_msg.pose.pose.position.y = self.robot_state_.y_
                odom_msg.pose.pose.position.z = 0.0

                odom_msg.pose.pose.orientation.w = math.cos(self.robot_state_.theta_ / 2)
                odom_msg.pose.pose.orientation.x = 0.0
                odom_msg.pose.pose.orientation.y = 0.0
                odom_msg.pose.pose.orientation.z = math.sin(self.robot_state_.theta_ / 2)


                odom_msg.twist.twist.linear.x = linVel
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0

                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = angVel

                self.odom_publisher_.publish(odom_msg)  # Publish the Odometry data

        print(msg)  # Print the received message for debugging purposes


def main():
    rclpy.init()

    comms_node = CommsNode()

    try:
        rclpy.spin(comms_node)
    finally:
        comms_node.comms_.close()
        comms_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()