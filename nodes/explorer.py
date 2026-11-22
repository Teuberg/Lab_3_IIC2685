#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from time import sleep
import math
import random

#===========================
#Parametros de alineacion
STEP = 0.20                       
SAFE_RADIUS = 0.30               
MIN_ADVANCE = 0.05                
FOV_HALF_DEG = 28                 
SIDE_FROM_DEG = 10                
WALL_RAY_DEG = 20                 
ALIGN_TOL = math.radians(6)       
ALIGN_MAX = math.radians(30)      
TURN_BIAS = 0.7                  


class ExplorerBot(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name

        self.action_in_progress = False
        self.current_scan = LaserScan()
        self.last_move = Pose()

        #Lecturas lidar
        self.front = 0.0
        self.left = 0.0
        self.right = 0.0
        self.wall_angle = None

        self.client_sub = self.create_subscription(Empty, "/explore", self.explore_cb, 1)
        self.movement_sub = self.create_subscription(Empty, "/movement_ready", self.ready_cb, 1)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_cb, 2)

        self.displacement_pub = self.create_publisher(Pose, "/displacement", 1)
        self.controller_pub = self.create_publisher(Pose, "/move", 1)

    def check_connections(self):
        #Loop para q no se bugeen los one shot
        while rclpy.ok():
            self.get_logger().info(f"Esperando subs de {self.name}")

            if (self.displacement_pub.get_subscription_count() > 0 and self.controller_pub.get_subscription_count() > 0):
                break
            sleep(0.1)

        self.get_logger().info(f"[{self.name}] Inicializando Robot explorador :D")

    #===========================
    #Callbacks

    def ready_cb(self, msg):
        if not self.action_in_progress:
            return
        self.action_in_progress = False
        self.displacement_pub.publish(self.last_move)

    def lidar_cb(self, msg):
        self.current_scan = msg

    def explore_cb(self, msg):
        if self.action_in_progress:
            return
 
        if not self.current_scan.ranges:
            self.get_logger().warn(f"[{self.name}] No tengo scans :c")
            self.displacement_pub.publish(self.make_pose())
            return
 
        self.update_readings()
        self.last_move = self.decide_action()
        self.action_in_progress = True
        self.controller_pub.publish(self.last_move)
 
        self.get_logger().info(f"[{self.name}] front={self.front:.2f} left={self.left:.2f} right={self.right:.2f}")
        self.get_logger().info(f"Siguiente movimiento: {self.describe(self.last_move)}")

    #===========================
    #Funciones

    def angle_to_index(self, scan, deg):
        i = round((math.radians(deg) - scan.angle_min) / scan.angle_increment)
        return max(0, min(len(scan.ranges) - 1, i))
 
    def valid(self, scan, r):
        return math.isfinite(r) and r >= scan.range_min
 
    def sector_min(self, scan, deg_from, deg_to):
        i0, i1 = sorted((self.angle_to_index(scan, deg_from), self.angle_to_index(scan, deg_to)))
        vals = [r for r in scan.ranges[i0:i1 + 1] if self.valid(scan, r)]
        return min(vals) if vals else scan.range_max
 
    def estimate_wall_angle(self, scan, deg=WALL_RAY_DEG):
        r1 = scan.ranges[self.angle_to_index(scan, -deg)]
        r2 = scan.ranges[self.angle_to_index(scan, deg)]
        if not (self.valid(scan, r1) and self.valid(scan, r2)) or max(r1, r2) >= scan.range_max:
            return None
        a = math.radians(deg)
        x1, y1 = r1 * math.cos(-a), r1 * math.sin(-a)
        x2, y2 = r2 * math.cos(a), r2 * math.sin(a)
        return math.atan2(-(x2 - x1), y2 - y1)
 
    def update_readings(self):
        scan = self.current_scan
        self.front = self.sector_min(scan, -FOV_HALF_DEG, FOV_HALF_DEG)
        self.left = self.sector_min(scan, SIDE_FROM_DEG, FOV_HALF_DEG)
        self.right = self.sector_min(scan, -FOV_HALF_DEG, -SIDE_FROM_DEG)
        self.wall_angle = self.estimate_wall_angle(scan)

    def make_pose(self, x=0.0, yaw=0.0):
        p = Pose()
        p.position.x = float(x)
        p.orientation.z = math.sin(yaw / 2)
        p.orientation.w = math.cos(yaw / 2)
        return p
 
    def describe(self, p: Pose):
        yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        if abs(p.position.x) > 1e-6:
            return f"avanzar {p.position.x:.2f} m"
        return f"rotar {math.degrees(yaw):.1f}°"


    def decide_action(self) -> Pose:
        #Camino libre
        if self.front > STEP + SAFE_RADIUS:
            return self.make_pose(x=STEP)
 
        #Pared cerca pero chueca
        if self.wall_angle is not None and ALIGN_TOL < abs(self.wall_angle) < ALIGN_MAX:
            return self.make_pose(yaw=self.wall_angle)
 
        #Acercarse al murito
        if self.front - SAFE_RADIUS > MIN_ADVANCE:
            return self.make_pose(x=self.front - SAFE_RADIUS)
 
        #Bloqueado -> girar random
        better = 1 if self.left > self.right else -1
        sign = better if random.random() < TURN_BIAS else -better
        return self.make_pose(yaw=sign * math.pi / 2)
 


def main():
    rclpy.init()
    robot = ExplorerBot("Don_explorer")
    robot.check_connections()

    try:
       rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()