import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import math
from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import LaserScan

QOS_PROFILE_DEFAULT = 10
PI = math.pi
LEFT_TURN = +1.0
RIGHT_TURN = -1.0
TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription_vectors = self.create_subscription(EdgeVectors, '/edge_vectors', self.edge_vectors_callback, QOS_PROFILE_DEFAULT)
        self.publisher_joy = self.create_publisher(Joy, '/cerebri/in/joy', QOS_PROFILE_DEFAULT)
        self.subscription_traffic = self.create_subscription(TrafficStatus, '/traffic_status', self.traffic_status_callback, QOS_PROFILE_DEFAULT)
        self.traffic_status = TrafficStatus()

    def rover_move_manual_mode(self, speed, turn):
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed, 0.0, turn]
        self.publisher_joy.publish(msg)

    def edge_vectors_callback(self, message):
        speed = SPEED_MAX
        turn = TURN_MIN
        vectors = message
        half_width = vectors.image_width / 2

        if vectors.vector_count == 0:
            speed = 0  # Stop if no vectors are detected
        elif vectors.vector_count == 1:
            deviation = vectors.vector_1[1].x - vectors.vector_1[0].x
            turn = deviation / vectors.image_width
        elif vectors.vector_count == 2:
            middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
            middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
            middle_x = (middle_x_left + middle_x_right) / 2
            deviation = half_width - middle_x
            turn = deviation / half_width

        if self.traffic_status.stop_sign:
            speed = SPEED_MIN
            print("Stop sign detected")

        self.rover_move_manual_mode(speed, turn)

    def traffic_status_callback(self, message):
        self.traffic_status = message

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()