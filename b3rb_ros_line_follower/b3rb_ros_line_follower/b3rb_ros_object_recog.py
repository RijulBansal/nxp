import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import math
from synapse_msgs.msg import EdgeVectors
from sensor_msgs.msg import LaserScan

QOS_PROFILE_DEFAULT = 10
THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription_vectors = self.create_subscription(EdgeVectors, '/edge_vectors', self.edge_vectors_callback, QOS_PROFILE_DEFAULT)
        self.publisher_joy = self.create_publisher(Joy, '/cerebri/in/joy', QOS_PROFILE_DEFAULT)
        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, QOS_PROFILE_DEFAULT)
        self.obstacle_detected = False

    def rover_move_manual_mode(self, speed, turn):
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed, 0.0, turn]
        self.publisher_joy.publish(msg)

    def edge_vectors_callback(self, message):
        speed = 0
        turn = 0
        self.rover_move_manual_mode(speed, turn)

    def lidar_callback(self, message):
        shield_vertical = 4
        shield_horizontal = 1
        theta = math.atan(shield_vertical / shield_horizontal)

        length = float(len(message.ranges))
        ranges = message.ranges[int(length / 4): int(3 * length / 4)]
        length = float(len(ranges))

        front_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]
        side_ranges_right = ranges[0: int(length * theta / PI)]
        side_ranges_left = ranges[int(length * (PI - theta) / PI):]

        for distance in front_ranges:
            if distance < THRESHOLD_OBSTACLE_VERTICAL:
                self.obstacle_detected = True
                return

        side_ranges_left.reverse()
        for side_ranges in [side_ranges_left, side_ranges_right]:
            for distance in side_ranges:
                if distance < THRESHOLD_OBSTACLE_HORIZONTAL:
                    self.obstacle_detected = True
                    return

        self.obstacle_detected = False

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()