import rclpy 

from rclpy.node import Node

from geometry_msgs.msg import Pose, Point
from multi_robot_scoring.scoring_interfaces.srv import SetMarkerPosition
from std_msgs.msg import Int64

#This class is not used in the project. This was supposed to be a part of the project
#but since i could not get the scoring to work in time it is not included. 
class ArUcoDetection(Node):
    def __init__(self):
        super().__init__('turtlebot3')

        self.declare_parameter('namespace', '')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value        
        self.get_logger().info(f'Namespace: {self.namespace}')

        self.reporting_client = self.create_client(
            SetMarkerPosition,
            '/set_marker_position',
        )
        while self.reporting_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service: reporting_client')

        self.large_fire_publisher = self.create_publisher(
            Pose,
            '/large_fire_location',
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

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.large_fire_is_detected():
            self.publish_large_fire(self.marker_map_position.x, self.marker_map_position.y)
        
        self.report_aruco_marker()


    def marker_pose_callback(self, msg):
        self.marker_map_position = msg.position

    def marker_id_callback(self, msg):
        self.marker_id = msg.data

    def publish_large_fire(self, x, y):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        self.large_fire_publisher.publish(msg)

    def report_aruco_marker(self):
        id = self.marker_id
        position = self.marker_map_position

        if id == 100 or position is None:
            return
        
        self.get_logger().info(f"Marker with id: {id} detected at location: x={position.x} y={position.y}")
        accepted = self.report_finding(id, position)
        self.get_logger().info(f"Accepted id({id}) status: {accepted}")

    def report_finding(self, id, pos: Point):
        request = SetMarkerPosition.Request()
        request.marker_id = id
        request.marker_position = pos
        future = self.reporting_client.call_async(request)
        return future.add_done_callback(self.handle_report_response)

    def handle_report_response(self, future):
        try:
            response = future.result()
            return response.accepted
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
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
        
def main(args=None):
    rclpy.init(args=args)

    aruco = ArUcoDetection()

    rclpy.spin(aruco)

    aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
