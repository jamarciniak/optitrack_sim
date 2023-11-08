#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from optitrack_sim.msg import OptiData         
from tf2_py import quaternion_from_euler
import time
import yaml



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('optitrack_simulator')
        self.publisher_ = self.create_publisher(OptiData, '/optitrack/data', 10)  
        self.twist_publisher_ = self.create_publisher(Twist, '/optitrack/cmd_vel', 10)  
        timer_period = 0.5
        self.load_json_data()
        
    def load_yaml_data(self):
        file_path = '/root/tello_ros_ws/logger_QDrone.txt'
        
        # Dane to kolejno: czy obiekt jest śledzony, pozycja x, pozycja y, pozycja z, kąt roll, kąt pitch, kąt yaw. Pozycja x,y,z jest liczona od środka laboratorium (punkt 0,0,0).

        with open(file_path,'r') as file:
            for line in file:
                data = yaml.safe_load(line)
                rigid_bodies = data['rigid_bodies']
                q_drone = [float(i) for i in data['QDrone']]
                
                msg = OptiData()                                                
                msg.rigid_bodies = rigid_bodies                         
                msg.q_drone = q_drone
                self.publisher_.publish(msg)
                
                twist_msg = Twist()
                twist_msg.linear.x = rigid_bodies[0]
                twist_msg.linear.y = rigid_bodies[1]
                twist_msg.linear.z = rigid_bodies[2]
                
                quaternion = quaternion_from_euler(rigid_bodies[3],rigid_bodies[4],rigid_bodies[5])
                twist_msg.angular.x = quaternion[0]
                twist_msg.angular.x = quaternion[1]
                twist_msg.angular.x = quaternion[2]
                self.twist_publisher_.publish(twist_msg)
                
                
                time.sleep(0.5)
                
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()