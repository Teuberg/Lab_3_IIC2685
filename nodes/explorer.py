#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from time import sleep
import math

#Idea: que aqui el compare tome las decisiones fuertes basadas en las lecturas del lidar


class Robot(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name

        self.client_sub = self.create_subscription(Empty, "/explore", self.explore_cb, 1)
        self.displacement_pub = self.create_publisher(Pose, "/displacement", 1)

    def check_connections(self):
        #Loop para q no se bugeen los one shot
        while rclpy.ok():
            self.get_logger().info(f"Esperando sub de {self.name}")

            if (self.client.get_subscription_count() > 0):
                break
            sleep(0.1)

        self.get_logger().info(f"[{self.name}] Inicializando Robot explorador :D")









def main():
    rclpy.init()
    robot = Robot("Don explorador")
    robot.check_connections()

    try:
       robot.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()