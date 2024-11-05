#usr/bin/python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from interfaces.srv import SetGoal
from interfaces.srv import FrontierRequest


class CentralController(Node):
    def __init__(self):
        super().__init__('CentralController')

        #Create clients for moving each robot to their goals
        self.client_tb3_0 = self.create_client(
            SetGoal,
            'tb3_0_go_to_point_service',
        )
        self.client_tb3_1 = self.create_client(
            SetGoal,
            'tb3_1_go_to_point_service',
        )

        while not self.client_tb3_0.wait_for_service(timeout_sec=2.0):
            print("Waiting for service: tb3_0")
        # while not self.client_tb3_1.wait_for_service(timeout_sec=2.0):
        #     print("Waiting for service: tb3_1")
        
        self.client_frontier_based_search = self.create_client(
            FrontierRequest,
            'frontier_based_search' 
        )

        self.subscription_lidar_tb3_0 = self.create_subscription(
            LaserScan,
            'tb3_0/scan',
            self.lidar_callback_tb3_0,
            10)
        
        self.subscription_lidar_tb3_1 = self.create_subscription(
            LaserScan,
            'tb3_1/scan',
            self.lidar_callback_tb3_1,
            10)

        self.sensors_tb3_0 = {
            "front": 0,
            "left": 0,
            "right": 0,
            "rear": 0,
            "front_left": 0,
            "front_right": 0,
            "rear_left": 0,
            "rear_right": 0
        }

        self.sensors_tb3_1 = {
            "front": 0,
            "left": 0,
            "right": 0,
            "rear": 0,
            "front_left": 0,
            "front_right": 0,
            "rear_left": 0,
            "rear_right": 0
        }

        self.requestsent = False
        print('Hey')
        self.timer = self.create_timer(0.1, self.timer_callback)

    def lidar_callback_tb3_0(self, msg):
        self.sensors_tb3_0 = {
            "front": min(msg.ranges[0:1]),
            "left": min(msg.ranges[90:91]),
            "right": min(msg.ranges[270:271]),
            "rear": min(msg.ranges[180:181]),
            "front_left": min(msg.ranges[45:46]),
            "front_right": min(msg.ranges[315:316]),
            "rear_left": min(msg.ranges[135:136]),
            "rear_right": min(msg.ranges[225:226])
        }

    def lidar_callback_tb3_1(self, msg):
        self.sensors_tb3_1 = {
            "front": min(msg.ranges[0:1]),
            "left": min(msg.ranges[90:91]),
            "right": min(msg.ranges[270:271]),
            "rear": min(msg.ranges[180:181]),
            "front_left": min(msg.ranges[45:46]),
            "front_right": min(msg.ranges[315:316]),
            "rear_left": min(msg.ranges[135:136]),
            "rear_right": min(msg.ranges[225:226])
        }

    def send_frontier_request(self, lidarsensor_data, x_coord, y_coord):
        request = FrontierRequest.Request()
        request.data = lidarsensor_data
        request.current_pos.x = x_coord
        request.current_pos.y = y_coord

        future = self.client_tb3_0.call_async(request)
        future.add_done_callback(self.handle_frontier_response)

    def handle_frontier_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def send_gotopoint_request(self, switch_val, x_coord, y_coord):
        request = SetGoal.Request()
        request.move_switch = switch_val
        request.target_position.x = x_coord
        request.target_position.y = y_coord

        future = self.client_tb3_0.call_async(request)
        future.add_done_callback(self.handle_gotopoint_response)

    def handle_gotopoint_response(self, future):
        try:
            response = future.result()
            #self.arrived_at_goal = response
            self.get_logger().info(f'Service response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def timer_callback(self):
        if not self.requestsent:
            self.send_gotopoint_request(True, 1.0, -1.0)
            self.requestsent = True
            print('Request sent')


def main(args=None):
    rclpy.init(args=args)

    controller = CentralController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()