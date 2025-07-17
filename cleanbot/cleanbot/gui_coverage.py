# Copyright (c) 2023 Open Navigation LLC
# Modified by Wasif Raza

from enum import Enum
import time
import math
from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped, Quaternion
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from opennav_coverage_msgs.action import NavigateCompleteCoverage
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

class CoverageNavigator(Node):

    def __init__(self):
        super().__init__(node_name='gui_coverage')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None
        self.is_processing = False
        self.pending_fields = []
        self.failed_fields = []
        self.completed_fields = []
        self.active_goal_id = None
        self.current_field = None
        self.is_moving = False
        self.robot_pose = None
        self.last_pose_time = None
        self.goal_start_time = None
        self.is_resetting = False

        # Declare parameters for initial pose and map boundaries
        self.declare_parameter('initial_pose.x', 0.0)
        self.declare_parameter('initial_pose.y', 0.0)
        self.declare_parameter('initial_pose.yaw', 0.0)
        self.declare_parameter('map_width', 10500.0)
        self.declare_parameter('map_height', 10500.0)

        # Log initial parameter values for debugging
        initial_x = self.get_parameter('initial_pose.x').get_parameter_value().double_value
        initial_y = self.get_parameter('initial_pose.y').get_parameter_value().double_value
        initial_yaw = self.get_parameter('initial_pose.yaw').get_parameter_value().double_value
        self.get_logger().info(f'Initial pose parameters: x={initial_x}, y={initial_y}, yaw={initial_yaw}')

        # Action clients and publishers
        self.coverage_client = ActionClient(self, NavigateCompleteCoverage, 'navigate_complete_coverage')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_costmap_client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entire_costmap')
        self.field_sub = self.create_subscription(Polygon, '/user_selected_field', self.field_callback, 10)
        self.initial_pose_sub = self.create_subscription(String, '/return_to_initial_pose', self.return_to_initial_pose, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/user_selected_field_marker', 10)
        self.status_pub = self.create_publisher(String, '/coverage_task_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Coverage Navigator initialized')

        # Timers for periodic checks
        self.create_timer(0.2, self.check_pose_updates)
        self.create_timer(0.1, self.check_goal_status)

    def destroy_node(self):
        self.coverage_client.destroy()
        self.nav_to_pose_client.destroy()
        self.field_sub.destroy()
        self.initial_pose_sub.destroy()
        self.cmd_vel_sub.destroy()
        self.pose_sub.destroy()
        super().destroy_node()

    def toPolygon(self, field):
        poly = Polygon()
        for coord in field:
            pt = Point32()
            pt.x = float(coord[0])
            pt.y = float(coord[1])
            poly.points.append(pt)
        return poly

    def publish_polygon_marker(self, field):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'polygon'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for coord in field:
            pt = Point()
            pt.x = float(coord[0])
            pt.y = float(coord[1])
            pt.z = 0.0
            marker.points.append(pt)
        if field:
            pt = Point()
            pt.x = float(field[0][0])
            pt.y = float(field[0][1])
            pt.z = 0.0
            marker.points.append(pt)
        self.marker_pub.publish(marker)

    def is_convex(self, field):
        def cross_product(p1, p2, p3):
            return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
        n = len(field)
        if n < 3:
            return False
        sign = 0
        for i in range(n):
            p1, p2, p3 = field[i], field[(i + 1) % n], field[(i + 2) % n]
            cross = cross_product(p1, p2, p3)
            if cross != 0:
                if sign == 0:
                    sign = 1 if cross > 0 else -1
                elif (cross > 0 and sign < 0) or (cross < 0 and sign > 0):
                    return False
        return True

    def calculate_field_area(self, field):
        n = len(field)
        area = 0.0
        for i in range(n):
            j = (i + 1) % n
            area += field[i][0] * field[j][1]
            area -= field[j][0] * field[i][1]
        area = abs(area) / 2.0
        return area

    def field_callback(self, msg):
        self.get_logger().info('Received new field from /user_selected_field')
        field = [[pt.x, pt.y] for pt in msg.points]
        self.get_logger().info(f'Field coordinates: {field}')
        if len(field) < 3:
            self.get_logger().error('Field must have at least 3 points')
            return
        map_width = self.get_parameter('map_width').get_parameter_value().double_value
        map_height = self.get_parameter('map_height').get_parameter_value().double_value
        for pt in field:
            if not (-map_width/2 <= pt[0] <= map_width/2 and -map_height/2 <= pt[1] <= map_height/2):
                self.get_logger().error(f'Invalid point {pt} outside map boundaries [{map_width}, {map_height}]')
                return
        if not self.is_convex(field):
            self.get_logger().error(f'Field is not convex: {field}')
            return
        if any(all(math.isclose(pt[0], f[0]) and math.isclose(pt[1], f[1]) for pt, f in zip(field, cf)) for cf in self.completed_fields):
            self.get_logger().warn(f'Field {field} already completed, ignoring')
            return
        self.pending_fields.append(field)
        self.get_logger().info(f'Queued field, queue size: {len(self.pending_fields)}')
        if not self.is_processing and not self.is_resetting:
            self.process_field()

    def cmd_vel_callback(self, msg):
        self.is_moving = msg.linear.x != 0.0 or msg.angular.z != 0.0

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
        self.last_pose_time = self.get_clock().now()
        self.get_logger().info(f'Pose update: x={self.robot_pose.x}, y={self.robot_pose.y}')

    def check_pose_updates(self):
        if self.last_pose_time is None:
            self.get_logger().warn('No pose updates received')
            return
        time_since_last_pose = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if time_since_last_pose > 3.0:
            self.get_logger().warn(f'No pose updates for {time_since_last_pose:.1f}s')
            if time_since_last_pose > 10.0 and self.is_processing:
                self.get_logger().error('Pose updates lost, stopping task')
                self.is_processing = False
                self.resetState(keep_moving=False)
                status_msg = String()
                status_msg.data = 'FAILED: Pose updates lost'
                for _ in range(5):
                    self.status_pub.publish(status_msg)
                    time.sleep(0.1)
                time.sleep(1.0)
                self.process_field()

    def check_goal_status(self):
        if self.is_resetting:
            return
        if self.result_future and not self.result_future.done():
            if self.is_goal_active():
                return
            for _ in range(3):
                rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.1)
                if self.result_future.done():
                    break
                self.get_logger().warn('Result future not ready, retrying')
                time.sleep(0.5)
            if self.result_future.result():
                self.isTaskComplete()
            else:
                self.get_logger().error('Result future completed without result')
                self.is_processing = False
                self.resetState(keep_moving=False)
                status_msg = String()
                status_msg.data = 'FAILED: No result received'
                for _ in range(5):
                    self.status_pub.publish(status_msg)
                    time.sleep(0.1)
                time.sleep(1.0)
                self.process_field()
        elif self.is_processing and not self.result_future:
            self.get_logger().warn('No result future while processing, resetting')
            self.is_processing = False
            self.resetState(keep_moving=False)
            status_msg = String()
            status_msg.data = 'FAILED: No active goal'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
                time.sleep(1.0)
                self.process_field()

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        for _ in range(100):
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.01)
        self.is_moving = False
        self.get_logger().info('Robot stopped')

    def is_goal_active(self):
        if self.goal_handle and self.active_goal_id:
            status_future = self.goal_handle.get_status()
            rclpy.spin_until_future_complete(self, status_future, timeout_sec=0.1)
            if status_future.result():
                status = status_future.result().status
                self.get_logger().info(f'Goal status: {status}')
                return status in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]
        return False

    def is_inside_polygon(self, field):
        if not self.robot_pose or not field:
            return self.is_moving
        x, y = self.robot_pose.x, self.robot_pose.y
        n = len(field)
        inside = False
        j = n - 1
        for i in range(n):
            if ((field[i][1] > y) != (field[j][1] > y)) and \
               (x < (field[j][0] - field[i][0]) * (y - field[i][1]) / (field[j][1] - field[i][1]) + field[i][0]):
                inside = not inside
            j = i
        return inside

    def return_to_initial_pose(self, msg):
        self.get_logger().info('Received request to navigate to initial pose')
        self.is_resetting = True
        self.pending_fields = []
        self.failed_fields = []
        self.current_field = None
        self.is_processing = False
        for _ in range(3):
            self.resetState(keep_moving=False)
            if not self.is_goal_active():
                break
            self.get_logger().warn('Goal cancellation failed, retrying')
            time.sleep(1.0)
        self.stop_robot()
        status_msg = String()
        status_msg.data = 'Navigating to Initial Pose'
        for _ in range(5):
            self.status_pub.publish(status_msg)
            time.sleep(0.1)
        self.clear_costmap()
        if self.navigateToInitialPose():
            self.is_processing = True
        else:
            self.get_logger().error('Failed to navigate to initial pose')
            status_msg = String()
            status_msg.data = 'Idle'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
            self.stop_robot()
        self.is_resetting = False

    def clear_costmap(self):
        if not self.clear_costmap_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Clear costmap service unavailable')
            return False
        request = ClearEntireCostmap.Request()
        future = self.clear_costmap_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is not None:
            self.get_logger().info('Costmap cleared')
            return True
        self.get_logger().error('Failed to clear costmap')
        return False

    def navigateToInitialPose(self):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server unavailable')
            status_msg = String()
            status_msg.data = 'FAILED: NavigateToPose server unavailable'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
            return False

        # Read initial pose parameters
        try:
            x = self.get_parameter('initial_pose.x').get_parameter_value().double_value
            y = self.get_parameter('initial_pose.y').get_parameter_value().double_value
            yaw = self.get_parameter('initial_pose.yaw').get_parameter_value().double_value
            map_width = self.get_parameter('map_width').get_parameter_value().double_value
            map_height = self.get_parameter('map_height').get_parameter_value().double_value
            self.get_logger().info(f'Navigating to initial pose: x={x}, y={y}, yaw={yaw}')
        except Exception as e:
            self.get_logger().error(f'Invalid initial pose parameters: {str(e)}')
            status_msg = String()
            status_msg.data = 'FAILED: Invalid initial pose parameters'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
            return False

        # Validate pose within map boundaries
        if not (-map_width/2 <= x <= map_width/2 and -map_height/2 <= y <= map_height/2):
            self.get_logger().error(f'Initial pose [x={x}, y={y}] out of bounds [width={map_width}, height={map_height}]')
            status_msg = String()
            status_msg.data = 'FAILED: Initial pose out of bounds'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
            return False

        # Create quaternion for orientation
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = quaternion

        # Send goal with retries
        for attempt in range(1, 4):
            self.get_logger().info(f'Sending NavigateToPose goal [x={x}, y={y}, yaw={yaw}], attempt {attempt}')
            try:
                send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
                start_time = self.get_clock().now()
                timeout_duration = Duration(seconds=20.0)
                while (self.get_clock().now() - start_time) < timeout_duration:
                    rclpy.spin_once(self, timeout_sec=0.001)
                    if send_goal_future.done():
                        break
                    time.sleep(0.001)

                if not send_goal_future.done():
                    self.get_logger().error(f'NavigateToPose goal sending timed out after 20s, attempt {attempt}')
                    if attempt == 3:
                        status_msg = String()
                        status_msg.data = 'FAILED: NavigateToPose timed out'
                        for _ in range(5):
                            self.status_pub.publish(status_msg)
                            time.sleep(0.1)
                        return False
                    time.sleep(2.0)  # Delay before retry
                    continue

                self.goal_handle = send_goal_future.result()
                if not self.goal_handle.accepted:
                    self.get_logger().error(f'NavigateToPose goal rejected, attempt {attempt}')
                    if attempt == 3:
                        status_msg = String()
                        status_msg.data = 'FAILED: NavigateToPose rejected'
                        for _ in range(5):
                            self.status_pub.publish(status_msg)
                            time.sleep(0.1)
                        return False
                    time.sleep(2.0)  # Delay before retry
                    continue

                self.get_logger().info('NavigateToPose goal accepted')
                self.active_goal_id = self.goal_handle.goal_id
                self.result_future = self.goal_handle.get_result_async()
                return True
            except Exception as e:
                self.get_logger().error(f'NavigateToPose goal sending exception, attempt {attempt}: {str(e)}')
                if attempt == 3:
                    status_msg = String()
                    status_msg.data = f'FAILED: NavigateToPose exception - {str(e)}'
                    for _ in range(5):
                        self.status_pub.publish(status_msg)
                        time.sleep(0.1)
                    return False
                time.sleep(2.0)  # Delay before retry
        return False

    def process_field(self):
        if not self.pending_fields or self.is_resetting:
            self.is_processing = False
            self.current_field = None
            status_msg = String()
            status_msg.data = 'Idle'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
            self.stop_robot()
            self.get_logger().info('No pending fields, set to Idle')
            return

        self.current_field = self.pending_fields[0]
        self.is_processing = True
        self.get_logger().info(f'Processing field: {self.current_field}')
        self.publish_polygon_marker(self.current_field)

        if not self.navigateCoverage(self.current_field, ""):
            self.get_logger().error(f'Field failed: {self.current_field}')
            self.failed_fields.append(self.current_field)
            self.pending_fields.pop(0)
            self.current_field = None
            self.is_processing = False
            self.stop_robot()
            status_msg = String()
            status_msg.data = 'FAILED: Field navigation failed'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
            time.sleep(1.0)
            self.process_field()
        else:
            self.get_logger().info('Coverage goal sent, awaiting result')

    def navigateCoverage(self, field, field_file):
        if not field:
            self.get_logger().error('No field provided')
            status_msg = String()
            status_msg.data = 'FAILED: No field provided'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
            return False

        if not self.coverage_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateCompleteCoverage action server unavailable')
            status_msg = String()
            status_msg.data = 'FAILED: Action server unavailable'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
            return False

        self.clear_costmap()
        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = 'map'
        goal_msg.polygons.clear()
        goal_msg.polygons.append(self.toPolygon(field))

        self.get_logger().info(f'Sending NavigateCompleteCoverage goal: {field}')
        for attempt in range(1, 4):
            try:
                send_goal_future = self.coverage_client.send_goal_async(goal_msg, self._feedbackCallback)
                self.goal_start_time = self.get_clock().now()
                start_time = self.get_clock().now()
                timeout_duration = Duration(seconds=20.0)
                while (self.get_clock().now() - start_time) < timeout_duration:
                    rclpy.spin_once(self, timeout_sec=0.001)
                    if send_goal_future.done():
                        break
                    time.sleep(0.001)

                if not send_goal_future.done():
                    self.get_logger().error(f'NavigateCompleteCoverage goal sending timed out after 20s, attempt {attempt}')
                    if attempt == 3:
                        status_msg = String()
                        status_msg.data = 'FAILED: NavigateCompleteCoverage timed out'
                        for _ in range(5):
                            self.status_pub.publish(status_msg)
                            time.sleep(0.1)
                        self.resetState()
                        return False
                    time.sleep(2.0)  # Delay before retry
                    continue

                self.goal_handle = send_goal_future.result()
                if not self.goal_handle.accepted:
                    self.get_logger().error(f'NavigateCompleteCoverage goal rejected, attempt {attempt}')
                    if attempt == 3:
                        status_msg = String()
                        status_msg.data = 'FAILED: NavigateCompleteCoverage rejected'
                        for _ in range(5):
                            self.status_pub.publish(status_msg)
                            time.sleep(0.1)
                        return False
                    time.sleep(2.0)  # Delay before retry
                    continue

                self.get_logger().info('NavigateCompleteCoverage goal accepted')
                self.active_goal_id = self.goal_handle.goal_id
                self.result_future = self.goal_handle.get_result_async()
                return True
            except Exception as e:
                self.get_logger().error(f'NavigateCompleteCoverage goal sending exception, attempt {attempt}: {str(e)}')
                if attempt == 3:
                    status_msg = String()
                    status_msg.data = f'FAILED: NavigateCompleteCoverage exception - {str(e)}'
                    for _ in range(5):
                        self.status_pub.publish(status_msg)
                        time.sleep(0.1)
                    return False
                time.sleep(2.0)  # Delay before retry
        return False

    def isTaskComplete(self):
        if not self.result_future:
            if self.pending_fields and not self.is_processing and not self.is_resetting:
                time.sleep(1.0)
                self.process_field()
            return True

        if self.goal_start_time and self.current_field:
            area = self.calculate_field_area(self.current_field)
            base_timeout = 60.0
            dynamic_timeout = base_timeout + area * 10.0
            elapsed_time = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            if elapsed_time > dynamic_timeout:
                self.get_logger().error(f'Goal timeout after {elapsed_time:.1f}s (timeout: {dynamic_timeout:.1f}s, area: {area:.2f})')
                self.failed_fields.append(self.current_field)
                self.pending_fields.pop(0)
                self.current_field = None
                self.is_processing = False
                self.resetState(keep_moving=False)
                status_msg = String()
                status_msg.data = 'FAILED: Goal timeout'
                for _ in range(5):
                    self.status_pub.publish(status_msg)
                    time.sleep(0.1)
                time.sleep(1.0)
                self.process_field()
                return True

        if not self.result_future.done():
            if self.is_goal_active():
                return False
            for _ in range(3):
                rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.1)
                if self.result_future.done():
                    break
                self.get_logger().warn('Result future not ready, waiting')
                time.sleep(0.5)

        if self.result_future.result():
            self.status = self.result_future.result().status
            result = self.getResult()
            self.get_logger().info(f'Task status: {self.status}, Result: {result}')
            status_msg = String()
            status_msg.data = result.name
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)

            if result == TaskResult.SUCCEEDED:
                if self.current_field:
                    self.completed_fields.append(self.current_field)
                self.pending_fields.pop(0)
                self.current_field = None
                self.is_processing = False
                self.resetState(keep_moving=False)
                time.sleep(1.0)
                self.process_field()
                return True
            elif result == TaskResult.FAILED or result == TaskResult.CANCELED:
                self.get_logger().error(f'Task failed with status: {result.name}')
                self.failed_fields.append(self.current_field)
                self.pending_fields.pop(0)
                self.current_field = None
                self.is_processing = False
                self.resetState(keep_moving=False)
                status_msg = String()
                status_msg.data = f'FAILED: {result.name}'
                for _ in range(5):
                    self.status_pub.publish(status_msg)
                    time.sleep(0.1)
                time.sleep(1.0)
                self.process_field()
                return True
        else:
            self.get_logger().error('Result future completed without result')
            self.is_processing = False
            self.resetState(keep_moving=False)
            status_msg = String()
            status_msg.data = 'FAILED: No result received'
            for _ in range(5):
                self.status_pub.publish(status_msg)
                time.sleep(0.1)
            time.sleep(1.0)
            self.process_field()
            return True

        return False

    def resetState(self, keep_moving=False):
        self.get_logger().info('Resetting state')
        self.is_resetting = True
        for _ in range(3):
            if self.goal_handle:
                cancel_future = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=1.0)
                if cancel_future.result():
                    self.get_logger().info('Goal cancelled')
                else:
                    self.get_logger().warn('Failed to cancel goal')
            if not self.is_goal_active():
                break
            time.sleep(1.0)
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None
        self.active_goal_id = None
        self.goal_start_time = None
        if not keep_moving:
            self.stop_robot()
        self.is_resetting = False
        self.get_logger().info('State reset complete')

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        self.get_logger().info(f'Feedback: ETA {Duration.from_msg(msg.feedback.estimated_time_remaining).nanoseconds / 1e9}s')

    def getResult(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            self.get_logger().warn(f'Unknown goal status: {self.status}')
            return TaskResult.FAILED

def main():
    rclpy.init()
    navigator = CoverageNavigator()
    try:
        while rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.0005)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down')
        navigator.stop_robot()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


