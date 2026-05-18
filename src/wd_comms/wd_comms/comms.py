import serial
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


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



class CommsNode(Node):
    def __init__(self):
        super().__init__('comms_node')
        self.comms_ = Comms()
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
            #elif msg.startswith("O,"):

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