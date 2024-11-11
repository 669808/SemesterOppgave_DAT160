#usr/bin/python3
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose
from interfaces.srv import SetGoal
from interfaces.srv import FrontierRequest


class CentralController(Node):
    def __init__(self):
        super().__init__('CentralController')

        #Create clients for moving each robot to their goals
        self.client_tb3_0 = self.create_client(
            SetGoal,
            'tb3_0/go_to_point_service',
        )
        self.client_tb3_1 = self.create_client(
            SetGoal,
            'tb3_1/go_to_point_service',
        )

        while not self.client_tb3_0.wait_for_service(timeout_sec=2.0):
            print("Waiting for service: tb3_0")
        while not self.client_tb3_1.wait_for_service(timeout_sec=2.0):
            print("Waiting for service: tb3_1")
        
        # self.client_frontier_based_search = self.create_client(
        #     FrontierRequest,
        #     'frontier_based_search' 
        # )

        # while not self.client_frontier_based_search.wait_for_service(timeout_sec=2.0):
        #     print("Waiting for service: frontier_based_search")

        # self.subscription_marker_pose_tb3_0 = self.create_subscription(Pose,'tb3_0//marker_map_pose',self.marker_callback_tb3_0,10)
        # self.subscription_marker_pose_tb3_1 = self.create_subscription(Pose,'tb3_1//marker_map_pose',self.marker_callback_tb3_1,10)
        # self.subscription_marker_id_tb3_0 = self.create_subscription(Int64,'tb3_0/marker_id',self.marker_id_callback_tb3_0,10)
        # self.subscription_marker_id_tb3_1 = self.create_subscription(Int64,'tb3_1/marker_id',self.marker_id_callback_tb3_1,10)
        #self. scoring = self.create_client(SetMarkerPosition, 'scoring')

        self.requestsent = False
        self.timer = self.create_timer(0.1, self.timer_callback)

    def send_frontier_request(self, x_coord, y_coord):
        request = FrontierRequest.Request()
        request.current_pos.x = x_coord
        request.current_pos.y = y_coord

        future = self.client_frontier_based_search.call_async(request)
        print('Request sent')

        future.add_done_callback(self.handle_frontier_response)

    def handle_frontier_response(self, future):
        try:
            response = future.result()
            frontier = response.frontier
            self.get_logger().info(f'Service response: {response.success}')
            print(f'Frontier: {frontier}')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def send_gotopoint_request(self, switch_val, x_coord, y_coord, robot_id):
        request = SetGoal.Request()
        request.move_switch = switch_val
        request.target_position.x = x_coord
        request.target_position.y = y_coord

        if robot_id == 'tb3_0':
            future = self.client_tb3_0.call_async(request)
        if robot_id == 'tb3_1':
            future = self.client_tb3_1.call_async(request)

        future.add_done_callback(self.handle_gotopoint_response)

    def handle_gotopoint_response(self, future): 
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def timer_callback(self):
        if not self.requestsent:
            self.send_gotopoint_request(True, 3.0, -3.0, 'tb3_0')
            self.send_gotopoint_request(True, 3.05, 3.2, 'tb3_1')
            self.requestsent = True



def main(args=None):
    rclpy.init(args=args)

    controller = CentralController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()