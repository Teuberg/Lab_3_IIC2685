#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from time import sleep
from threading import Thread
from nav_msgs.msg import OccupancyGrid

#Este compare solo servira para enviar mensaje y luego recibir y luego enviar y luego recibir y asi..
#Esto que sea el puntapie para luego en la pt 3 meterle weno con la construccion del mapa

EXPLORER_CHECK_RATE = 2

class Client(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name

        self.ready_to_explore = True
        self.world_ready = 0
        self.displacement_list = []

        self.explorer_pub = self.create_publisher(Empty, "/explore", 1)
        self.displacement_sub = self.create_subscription(Pose, "/displacement", self.displacement_cb, 1)
        self.world_sub = self.create_subscription(OccupancyGrid, "/world_map", self.world_cb, 1)
        
    def check_connections(self):
        while rclpy.ok():
            self.get_logger().info(f"Esperando sub de {self.name}")

            if (self.explorer_pub.get_subscription_count() > 0):
                break
            sleep(0.1)

        self.get_logger().info(f"[{self.name}] Inicializando Client explorador :p")

    #===========================
    #Callbacks
    def world_cb(self, msg):
        self.world_ready += 1

    def displacement_cb(self, msg):
        self.displacement_list.append(msg)
        self.ready_to_explore = True

    #===========================
    #Funciones

    def auto_explorer(self):
        while(True):
            sleep(EXPLORER_CHECK_RATE)
            if(not(self.ready_to_explore)):
                continue

            self.explorer_pub.publish(Empty())
            self.get_logger().info(f"[{self.name}] Publicando /explore :p")
            self.ready_to_explore = False

        

def main():
    rclpy.init()
    client = Client("Don_client")
    client.check_connections()

    while(client.world_ready < 2):
        client.get_logger().info("Esperando mapa...")
        rclpy.spin_once(client, timeout_sec=1.0)
    Thread(target=client.auto_explorer, daemon=True).start()

    try:
       rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()