#usr/bin/python3
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose
from interfaces.srv import SetGoal
from interfaces.srv import FrontierRequest
from std_msgs.msg import Bool  # Import the Bool message type

class MainController(Node):

    def __init__(self):
        super().__init__('MainController')

        self.activate_controllers = self.create_publisher(
            Bool,
            '/controller_nodes_switch',
            10
        )

        self.activate_controllers.publish(Bool(data=True))

        # self.requestsent = False
        # self.timer = self.create_timer(0.1, self.timer_callback)

        

         
    def timer_callback(self):
        ...


def main(args=None):
    rclpy.init(args=args)

    main_controller = MainController()

    rclpy.spin(main_controller)

    main_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
