import rclpy 
from rclpy.node import Node
from interfaces.srv import SetGoal
from std_msgs.msg import Bool
from interfaces.srv import FrontierRequest
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class TurtleBot3(Node):
    def __init__(self):
        super().__init__('turtlebot3')

        self.declare_parameter('namespace', '')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value        
        self.get_logger().info(f'Namespace: {self.namespace}')

        self.go_to_point_client = self.create_client(
            SetGoal,
            f'{self.namespace}/go_to_point_service',
        )

        self.client_frontier_based_search = self.create_client(
            FrontierRequest, 
            'frontier_based_search'
        )

        while not self.go_to_point_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service: go_to_point_service')
        while not self.client_frontier_based_search.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for service: frontier_based_search")

        self.change_state_service = self.create_service(
            SetGoal,
            f'{self.namespace}/change_state_service',
            self.handle_service
        )

        self.request_help_publisher = self.create_publisher(
            Pose,
            '/request_help',
            10
        )

        self.subscription_goal = self.create_subscription(
            Bool,
            f'{self.namespace}/goal_reached',
            self.goal_reached,
            10
        )

        self.subscription_switch = self.create_subscription(
            Bool,
            '/controller_nodes_switch',
            self.activate,
            10
        )

        self.tb3_current_pos = None
        self.subscription_odom = self.create_subscription(
            Odometry,
            f'{self.namespace}/odom',
            self.odom_callback,
            10
        )

        self.state_dict = {
            'request_frontier',
            'go_to_frontier',
            'wait_for_help',
            'go_to_help',
        }

        self.state = 'request_frontier'

        self.next_frontier = None
        self.active_task = False
        self.pending_help_request = None

        self.get_logger().info(self.namespace + ' controller started')
        
    def activate(self, msg):
        if msg.data:
            self.get_logger().info(self.namespace + ' controller activated')
            timer_period = 0.5
            self.timer = self.create_timer(timer_period, self.timer_callback)
        else:
            self.timer.cancel()

    def timer_callback(self):
        if not self.active_task:
            if(self.state == 'request_frontier'):
                self.get_logger().info('Requesting frontier')
                x_coord = self.tb3_current_pos.x
                y_coord = self.tb3_current_pos.y
                self.send_frontier_request(x_coord, y_coord)
                self.state = 'go_to_frontier'

            elif(self.state == 'go_to_frontier'):
                self.get_logger().info('Going to frontier')
                x_coord = self.next_frontier.x
                y_coord = self.next_frontier.y
                self.send_goal(x_coord, y_coord)

            elif(self.state == 'wait_for_help'):
                self.get_logger().info('Waiting for help')

            elif(self.state == 'go_to_help'):
                self.get_logger().info('Going to help')

    def send_goal(self, x, y):
        self.active_task = True
        request = SetGoal.Request()
        request.target_position.x = x
        request.target_position.y = y
        request.move_switch = True

        self.get_logger().info(f'Sending goal: {x}, {y}')

        future = self.go_to_point_client.call_async(request)
        future.add_done_callback(self.handle_gotopoint_response)

    def handle_gotopoint_response(self, future): 
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def send_frontier_request(self, x_coord, y_coord):
        self.active_task = True
        request = FrontierRequest.Request()
        request.current_pos.x = x_coord
        request.current_pos.y = y_coord

        future = self.client_frontier_based_search.call_async(request)
        self.get_logger().info('Request sent')

        future.add_done_callback(self.handle_frontier_response)

    def handle_frontier_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.next_frontier = response.frontier
                self.get_logger().info(f'Next frontier: {self.next_frontier}')
                self.active_task = False
            else:
                self.get_logger().error('Failed to get frontier')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def goal_reached(self, msg):
        if msg.data:
            self.get_logger().info('Goal reached')
            self.active_task = False

            if self.pending_help_request:
                self.state = 'go_to_help'
                self.send_goal(self.pending_help_request[0], self.pending_help_request[1])
                self.pending_help_request = None
            else:
                self.state = 'request_frontier'

    def handle_service(self, request, response):
        self.get_logger().info('Service request received')
        response.success = True
        (x, y) = (request.target_position.x, request.target_position.y)
        self.pending_help_request = (x, y)
        return response
    
    def odom_callback(self, msg):
        self.tb3_current_pos = msg.pose.pose.position
        


def main(args=None):
    rclpy.init(args=args)

    turtlebot3 = TurtleBot3()

    rclpy.spin(turtlebot3)

    turtlebot3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

