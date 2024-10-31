#usr/bin/python3
import rclpy
from rclpy.node import Node

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
        
        self.client_frontier_based_search = self.create_client(
            FrontierRequest,
            'frontier_based_search' 
        )

        self.requestsent = False
        print('Hey')
        self.timer = self.create_timer(0.1, self.timer_callback)

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
            self.get_logger().info(f'Service response: {response.message}')
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
            self.send_gotopoint_request(True, 2.0, 2.0)
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