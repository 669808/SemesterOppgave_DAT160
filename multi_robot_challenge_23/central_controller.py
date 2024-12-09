#usr/bin/python3
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose
from interfaces.srv import SetGoal
from interfaces.srv import FrontierRequest
#from std_msgs.msg import Bool  # Import the Bool message type
from geometry_msgs.msg import Pose 


class CentralController(Node):

    #Unsure if my commits have been working, testing
    #Communication between a-star controller and go-to-point
    def goal_reached_callback_tb3_0(self, msg):
        """Callback for tb3_0 goal reached notification."""
        self.get_logger().info(f"tb3_0 reached its goal at position: x={msg.position.x}, y={msg.position.y}. Initiating frontier-based search.")
        self.robot_states['tb3_0']['goal_reached'] = True

        # Use the received position to send a frontier-based search request
        self.send_frontier_request(msg.position.x, msg.position.y)

    def goal_reached_callback_tb3_1(self, msg):
        """Callback for tb3_1 goal reached notification."""
        self.get_logger().info(f"tb3_1 reached its goal at position: x={msg.position.x}, y={msg.position.y}. Initiating frontier-based search.")
        self.robot_states['tb3_1']['goal_reached'] = True

        # Use the received position to send a frontier-based search request
        self.send_frontier_request(msg.position.x, msg.position.y)
    


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

        self.robot_states = {
            'tb3_0': {'goal_reached': False},
            'tb3_1': {'goal_reached': False},
        }

        while not self.client_tb3_0.wait_for_service(timeout_sec=2.0):
            print("Waiting for service: tb3_0")
        while not self.client_tb3_1.wait_for_service(timeout_sec=2.0):
            print("Waiting for service: tb3_1")

        self.create_subscription(
            Pose,
            'tb3_0/goal_reached',
            self.goal_reached_callback_tb3_0,
            10
        )
        
        self.create_subscription(
            Pose,
            'tb3_1/goal_reached',
            self.goal_reached_callback_tb3_1,
            10
        )


        self.client_frontier_based_search = self.create_client(FrontierRequest, 'frontier_based_search')
        while not self.client_frontier_based_search.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for service: frontier_based_search")


        # self.subscription_marker_pose_tb3_0 = self.create_subscription(Pose,'tb3_0//marker_map_pose',self.marker_callback_tb3_0,10)
        # self.subscription_marker_pose_tb3_1 = self.create_subscription(Pose,'tb3_1//marker_map_pose',self.marker_callback_tb3_1,10)
        # self.subscription_marker_id_tb3_0 = self.create_subscription(Int64,'tb3_0/marker_id',self.marker_id_callback_tb3_0,10)
        # self.subscription_marker_id_tb3_1 = self.create_subscription(Int64,'tb3_1/marker_id',self.marker_id_callback_tb3_1,10)
        #self. scoring = self.create_client(SetMarkerPosition, 'scoring')

        self.requestsent = False
        self.timer = self.create_timer(0.1, self.timer_callback)

        

         

    # def send_frontier_request(self, x_coord, y_coord):
    #     request = FrontierRequest.Request()
    #     request.current_pos.x = x_coord
    #     request.current_pos.y = y_coord

    #     future = self.client_frontier_based_search.call_async(request)
    #     print('Request sent')

    #     future.add_done_callback(self.handle_frontier_response)

    def send_frontier_request(self, x_coord, y_coord):
        request = FrontierRequest.Request()
        request.current_pos.x = x_coord
        request.current_pos.y = y_coord

        future = self.client_frontier_based_search.call_async(request)
        print('Request sent')

        future.add_done_callback(self.handle_frontier_response)

    # def handle_frontier_response(self, future):
    #     try:
    #         response = future.result()
    #         frontier = response.frontier
    #         self.get_logger().info(f'Service response: {response.success}')
    #         print(f'Frontier: {frontier}')

    #     except Exception as e:
    #         self.get_logger().error(f'Error: {e}')

    def handle_frontier_response(self, future):
        try:
            response = future.result()
            if response.success:
                frontier_point = response.frontier
                self.get_logger().info(f"New frontier located at: ({frontier_point.x}, {frontier_point.y})")
                # Example: Assign a new goal for tb3_0
                self.send_gotopoint_request(True, frontier_point.x, frontier_point.y, 'tb3_0')
            else:
                self.get_logger().warning("No frontier found.")
        except Exception as e:
            self.get_logger().error(f"Error handling frontier response: {e}")


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
            #self.send_frontier_request(0.0, -1.0)
            self.requestsent = True
            self.get_logger().info("Go-to-point requests sent.")

    # Check if tb3_0 has reached its goal and initiate frontier search
        if self.robot_states['tb3_0']['goal_reached']:
            self.get_logger().info("tb3_0 goal reached. Sending frontier request.")
            # Get the current position or pass a predefined one
            self.send_frontier_request(0.0, -1.0)
            self.robot_states['tb3_0']['goal_reached'] = False  # Reset state

        # Check if tb3_1 has reached its goal and initiate frontier search
        if self.robot_states['tb3_1']['goal_reached']:
            self.get_logger().info("tb3_1 goal reached. Sending frontier request.")
            # Get the current position or pass a predefined one
            self.send_frontier_request(0.0, -1.0)
            self.robot_states['tb3_1']['goal_reached'] = False  # Reset state


            # Define callbacks for when goals are reached, so that frontier based search can start. Still some quirks to the code, but I will commit to show I have done work
# def goal_reached_callback_tb3_0(self, msg):
#             if msg.data:  # Goal reached
#                 self.get_logger().info("tb3_0 reached its goal. Switching to frontier_based_search.")
#                 self.robot_states['tb3_0']['goal_reached'] = True
#                 # Send a frontier-based search request
#                 self.send_frontier_request(0.0, -1.0)  

# def goal_reached_callback_tb3_1(self, msg):
#             if msg.data:  # Goal reached
#                 self.get_logger().info("tb3_1 reached its goal. Switching to frontier_based_search.")
#                 self.robot_states['tb3_1']['goal_reached'] = True
#                 # Send a frontier-based search request
#                 self.send_frontier_request(0.0, 1.0) 


def main(args=None):
    rclpy.init(args=args)

    controller = CentralController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
