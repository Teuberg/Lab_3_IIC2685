#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from time import sleep
import math


class Client(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name

        self.explorer_pub = self.create_publisher(Empty, "/explorer", 1)
        self.displacement_sub = self.create_subscription(Pose, "/displacement", self.displacement_cb, 1)
        
    def check_connections(self):
        while rclpy.ok():
            self.get_logger().info(f"Esperando sub de {self.name}")

            if (self.explorer_pub.get_subscription_count() > 0):
                break
            sleep(0.1)

        self.get_logger().info(f"[{self.name}] Inicializando Client explorador :p")









def main():
    rclpy.init()
    client = Client("Don logger")
    client.check_connections()

    try:
       client.spin()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()