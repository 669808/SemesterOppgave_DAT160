import rclpy 

from rclpy.node import Node

from interfaces.srv import SetGoal, FrontierRequest
from std_msgs.msg import Bool, Int64
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


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

        self.large_fire_publisher = self.create_publisher(
            Pose,
            '/large_fire_location',
            10
        )

        self.waiting_state_publisher = self.create_publisher(
            Bool,
            f'{self.namespace}/waiting_state',
            10
        )

        self.activate_switch =  self.create_service(
            SetBool,
            f'{self.namespace}/activate_search',
            self.activate
        )

        self.assist_service = self.create_service(
            SetGoal,
            f'{self.namespace}/assist_service',
            self.assist_callback
        )

        self.subscription_goal = self.create_subscription(
            Bool,
            f'{self.namespace}/goal_reached',
            self.goal_reached,
            10
        )

        self.tb3_current_pos = None
        self.subscription_odom = self.create_subscription(
            Odometry,
            f'{self.namespace}/odom',
            self.odom_callback,
            10
        )

        self.marker_id = 100
        self.marker_map_position = None 
        
        self.large_fire_extinguished = False

        self.marker_pose_subscription = self.create_subscription(
            Pose,
            f"{self.namespace}/marker_map_pose",
            self.marker_pose_callback,
            10
        )

        self.marker_id_subscription = self.create_subscription(
            Int64,
            f"{self.namespace}/marker_id",
            self.marker_id_callback,
            10
        )

        # Could not get scoring working in time
        # self.reporting_client = self.create_client(
        #     SetMarkerPosition,
        #     '/set_marker_position',
        # )
        # while self.reporting_client.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().info('Waiting for service: reporting_client')

        self.state = 'request_frontier'

        self.next_frontier = None
        self.active_task = True
        self.pending_help_request = None

        self.get_logger().info(self.namespace + ' controller started')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer.cancel()
        
    def activate(self, request, response):
        if request.data:
            self.get_logger().info(self.namespace + ' controller activated')
            
            self.timer.reset()
            
            if self.state == 'wait_for_help':
                self.state = 'request_frontier'
                self.pending_help_request = None

            self.active_task = False
        else:
            self.get_logger().info(self.namespace + ' controller deactivated')
            self.timer.cancel()
        response.success = True
        return response

    def timer_callback(self):
        self.report_aruco_marker()
        if self.large_fire_is_detected():
            self.get_logger().info(f"MARKER ID = {self.marker_id}")
            fire_position = [self.marker_map_position.x, self.marker_map_position.y]
            self.publish_large_fire(fire_position[0], fire_position[1])
            self.large_fire_extinguished = True
        
        if not self.active_task:
            if(self.state == 'request_frontier'):
                self.get_logger().info('Requesting frontier')
                self.active_task = True
                x_coord = self.tb3_current_pos.x
                y_coord = self.tb3_current_pos.y
                self.send_frontier_request(x_coord, y_coord)
                self.state = 'go_to_frontier'

            elif(self.state == 'go_to_frontier'):
                self.get_logger().info('Going to frontier')
                self.active_task = True
                x_coord = self.next_frontier.x
                y_coord = self.next_frontier.y
                self.send_goal(x_coord, y_coord)

            elif(self.state == 'wait_for_help'):
                self.get_logger().info('Waiting for help')
                self.publish_waiting_state()
                self.active_task = True

            elif(self.state == 'go_to_help'):
                self.get_logger().info('Going to help')
                self.send_goal(self.pending_help_request[0], self.pending_help_request[1])
                self.publish_waiting_state()
                self.active_task = True

    def assist_callback(self, request, response):
        self.pending_help_request = [request.target_position.x, request.target_position.y]
        self.get_logger().info(f'Help requested: {self.pending_help_request}')
        response.success = True
        return response

    def send_goal(self, x, y):
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

            if self.state == 'go_to_help' and self.pending_help_request:
                self.state = 'wait_for_help'
            elif self.pending_help_request:
                self.state = 'go_to_help'
            else:
                self.state = 'request_frontier'
    
    def publish_large_fire(self, x, y):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        self.large_fire_publisher.publish(msg)
        
    
    def odom_callback(self, msg):
        self.tb3_current_pos = msg.pose.pose.position

    def publish_waiting_state(self):
        msg = Bool()
        waiting = (self.state == 'wait_for_help')
        msg.data = waiting
        self.waiting_state_publisher.publish(msg)

    def large_fire_is_detected(self):
        if self.large_fire_extinguished:
            return False
        if self.marker_id == 100:
            return False
        if self.marker_map_position == None:
            return False

        if self.marker_id == 4:
            return True
        else:
            return False
    
    def marker_pose_callback(self, msg):
        self.marker_map_position = msg.position

    def marker_id_callback(self, msg):
        self.marker_id = msg.data

    def report_aruco_marker(self):
        id = self.marker_id
        position = self.marker_map_position

        if id == 100 or position is None:
            return
        
        self.get_logger().info(f"Marker with id: {id} detected at location: x={position.x} y={position.y}")
    #     accepted = self.report_finding(id, position)
    #     self.get_logger().info(f"Accepted id({id}) status: {accepted}")

    # def report_finding(self, id, pos: Point):
    #     request = SetMarkerPosition.Request()
    #     request.marker_id = id
    #     request.marker_position = pos
    #     future = self.reporting_client.call_async(request)
    #     return future.add_done_callback(self.handle_report_response)

    # def handle_report_response(self, future):
    #     try:
    #         response = future.result()
    #         return response.accepted
    #     except Exception as e:
    #         self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)

    turtlebot3 = TurtleBot3()

    rclpy.spin(turtlebot3)

    turtlebot3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

