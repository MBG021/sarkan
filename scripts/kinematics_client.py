#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from sarkan.srv import KinematicGoal
# Python libraries
from numpy import pi, rad2deg, around
import sys

class KinematicsClient(Node):
    def __init__(self):
        super().__init__("kinematics_client")

        self.client = self.create_client(KinematicGoal, "kinematics")
        while not self.client.wait_for_service(timeout_sec= 5.0):
            print("Waiting for an available service")
        self.srv_request = KinematicGoal.Request()

        
    
    def send_request(self, input):
        self.srv_request.x = input[0]
        self.srv_request.y = input[1]
        self.srv_request.z = input[2]
        response = self.client.call_async(self.srv_request) 
        return response

def main():
    rclpy.init()
    node = KinematicsClient()

    x_des = float(sys.argv[1])
    y_des = float(sys.argv[2])
    z_des = float(sys.argv[3])
    
    try:
        # Send request to server
        input = [x_des, y_des, z_des]
        future = node.send_request(input)
        print("\nRequest:")
        print(f"\tX: {input[0]} m")
        print(f"\tY: {input[1]} m")
        print(f"\tZ: {input[2]} m")

        # Get anwer from server
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        print(f"\nResponse:")
        print(f"\t{around((response.q), 2)}\n")
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()