#!/usr/bin/env python3
# ROS2 libraries
import rclpy
import serial
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class KinematicsSubscriber(Node):
    def __init__(self):
        super().__init__("kinematics_subscriber")

        # Crear el suscriptor para el tópico "kinematics_response"
        self.subscription = self.create_subscription(
            Float64MultiArray,
            "serial_kinematic",
            self.listener_callback,
            10)
        
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        
        self.received_data = []  # Variable para almacenar los datos recibidos

    def listener_callback(self, msg):
        #self.received_data = list(msg.data)
        self.received_data = [round(value * (180 / np.pi), 2) for value in msg.data]
        self.get_logger().info(f"Recibido: {self.received_data}")

        data_to_send = ','.join(map(str, self.received_data)) + '\n'
        self.serial_port.write(data_to_send.encode())

    def destroy_node(self):
        # Cierra el puerto serial al finalizar
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = KinematicsSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destruir el nodo y cerrar ROS 2
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()