#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from optitrack_sim.msg import OptiData         
import time
import yaml
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('optitrack_simulator')
        self.twist_publisher_ = self.create_publisher(Twist, '/optitrack/cmd_vel', 10)  
        self.receive_from_server()
        
    def receive_from_server(self):
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.bind((UDP_IP,UDP_PORT))

        while True:
            data,addr = sock.recvfrom(1024)
            decoded_data = data.decode('utf-8')
            received_object = yaml.safe_load(decoded_data)
            
            rigid_bodies = [float(i) for i in received_object['QDrone']]
            twist_msg = Twist()
            twist_msg.linear.x = rigid_bodies[0]
            twist_msg.linear.y = rigid_bodies[1]
            twist_msg.linear.z = rigid_bodies[2]
            
            twist_msg.angular.x = rigid_bodies[3]
            twist_msg.angular.y = rigid_bodies[4]
            twist_msg.angular.z = rigid_bodies[5]
            self.twist_publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()