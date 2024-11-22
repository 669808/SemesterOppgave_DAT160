import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from interfaces.srv import SetGoal
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class MainController(Node):

    def __init__(self):
        super().__init__('MainController')

        self.large_fire_subscriber = self.create_subscription(
            Pose,
            '/large_fire_location',
            self.large_fire_callback,
            10
        )

        self.activate_search_tb3_0 = self.create_client(
            SetBool,
            'tb3_0/activate_search'
        )

        self.activate_search_tb3_1 = self.create_client(
            SetBool,
            'tb3_1/activate_search'
        )

        while not self.activate_search_tb3_0.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service: activate_search_tb3_0')
        while not self.activate_search_tb3_1.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service: activate_search_tb3_1')

        self.tb3_0_waiting_state_subscriber = self.create_subscription(
            Bool,
            'tb3_0/waiting_state',
            self.tb3_0_waiting_state_callback,
            10
        )

        self.tb3_1_waiting_state_subscriber = self.create_subscription(
            Bool,
            'tb3_1/waiting_state',
            self.tb3_1_waiting_state_callback,
            10
        )

        self.tb3_0_client = self.create_client(
            SetGoal,
            'tb3_0/assist_service'
        )

        self.tb3_1_client = self.create_client(
            SetGoal,
            'tb3_1/assist_service'
        )

        self.activate_controllers()

        self.active_event = False
        self.tb3_0_waitingstate = False
        self.tb3_1_waitingstate = False

        self.timer = self.create_timer(1, self.timer_callback)
        self.timer.cancel()
         
    def timer_callback(self):
        if self.active_event:
            if self.tb3_0_waitingstate and self.tb3_1_waitingstate:
                self.get_logger().info('Both robots are at the large fire')
                self.activate_controllers()
                self.active_event = False
                self.timer.cancel()

    def activate_controllers(self):
        request = SetBool.Request()
        request.data = True
        future1 = self.activate_search_tb3_0.call_async(request)
        future2 = self.activate_search_tb3_1.call_async(request)
        self.get_logger().info('Controllers have been activated')

        future1.add_done_callback(self.activate_controllers_callback)
        future2.add_done_callback(self.activate_controllers_callback)
    
    def activate_controllers_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')
        else:
            self.get_logger().info(f'Controllers activated: {response.success}')

    def large_fire_callback(self, msg):
        self.get_logger().info(f'Large fire location: {msg.position.x}, {msg.position.y}')
        pos = msg.position
        self.send_help_requests(pos)
        self.active_event = True
        self.timer.reset()

    def send_help_requests(self, position):
        request = SetGoal.Request()
        request.target_position = position
        request.target_position.y -= 0.5
        self.tb3_0_client.call_async(request)
        request.target_position.y += 1.0
        self.tb3_1_client.call_async(request)
        self.get_logger().info('Help request sent!')

    def tb3_0_waiting_state_callback(self, msg):
        self.tb3_0_waitingstate = msg.data

    def tb3_1_waiting_state_callback(self, msg):
        self.tb3_1_waitingstate = msg.data

def main(args=None):
    rclpy.init(args=args)

    main_controller = MainController()

    rclpy.spin(main_controller)

    main_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
