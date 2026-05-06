import serial
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Comms:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser_ = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize

    def upload(self, data):
        # first converts all list elements to strings, then joins them into one continuous string separated by commas
        comm = ",".join(map(str, data))
        self.ser_.write(f"{comm}\n".encode())

    def close(self):
        self.ser_.close()



class CommsNode(Node):
    def __init__(self):
        super().__init__('comms_node')
        self.comms_ = Comms()
        self.subscription_ = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.subscription_callback,
            10
        )
    
    def subscription_callback(self, msg):
        self.msg_list_= [msg.linear.x, msg.angular.z]
        self.comms_.upload(self.msg_list_)

def main():
    rclpy.init()
    comms_node = CommsNode()
    rclpy.spin(comms_node)
    comms_node.comms_.close()
    comms_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()