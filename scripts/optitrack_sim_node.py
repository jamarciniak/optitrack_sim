#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from optitrack_sim.msg import OptiData 
from tf_transformations import quaternion_from_euler
import time
import yaml
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('optitrack_simulator')
        self.twist_publisher_ = self.create_publisher(Twist, '/optitrack/pose', 10)  
        self.receive_from_server()
        
    def receive_from_server(self):
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.bind((UDP_IP,UDP_PORT))

        while True:
            data,addr = sock.recvfrom(1024)
            decoded_data = data.decode('utf-8')
            
            try:
                received_object = yaml.safe_load(decoded_data)
            except:
                continue
                
            if not isinstance(received_object,dict) or 'rigid_bodies' not in received_object:
                continue
            
            if not received_object['rigid_bodies']:
                continue 
            
            
            rigid_bodies = [float(i) for i in received_object['QDrone']]
            
            if not rigid_bodies[0]:
                print("Drone not in cage!")
                continue
            
            message = Twist()
            # Position
            message.linear.x = rigid_bodies[1]
            message.linear.y = rigid_bodies[2]
            message.linear.z = rigid_bodies[3]
            # Orientation
            message.angular.x = rigid_bodies[3]
            message.angular.y = rigid_bodies[4]
            message.angular.z = rigid_bodies[5]
            self.twist_publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()