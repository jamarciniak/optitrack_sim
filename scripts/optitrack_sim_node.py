#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from optitrack_sim.msg import OptiData         
import time
import yaml



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('optitrack_simulator')
        self.publisher_ = self.create_publisher(OptiData, '/optitrack/data', 10)  
        timer_period = 0.5
        self.load_json_data()

    def timer_callback(self):
        
        msg = OptiData()                                                
        msg.rigid_bodies = self.i                                           
        msg.q_drone = [float(self.i)] * 5                                   
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing rigid_bodies: {msg.rigid_bodies} ')       
        self.get_logger().info(f'Publishing q_drone: {msg.q_drone}')       
        self.i += 1
        
    def load_json_data(self):
        file_path = '/root/tello_ros_ws/logger_QDrone.txt'

        with open(file_path,'r') as file:
            for line in file:
                data = yaml.safe_load(line)
                rigid_bodies = data['rigid_bodies']
                q_drone = [float(i) for i in data['QDrone']]
                
                msg = OptiData()                                                
                msg.rigid_bodies = rigid_bodies                         
                msg.q_drone = q_drone
                self.publisher_.publish(msg)
                time.sleep(0.5)
                
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()