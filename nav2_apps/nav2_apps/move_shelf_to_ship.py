#!/usr/bin/env python3
import rclpy
import time
import math
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class WarehouseBotNode(Node):
    def __init__(self, navigator):
        super().__init__("warehouse_bot_node")
        self.navigator: BasicNavigator = navigator

        # member variables
        self.x = 0
        self.y = 0
        self.yaw = 0

        # node parameters
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.target_pose_values = self._target_pose_values()

        # callback groups
        self.callback_g1 = ReentrantCallbackGroup()
        self.callback_g2 = MutuallyExclusiveCallbackGroup()

        # node modification
        elevator_msg = Empty if self.use_sim_time else String

        # ros objects
        self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.subscriber_odom_callback, 10, callback_group=self.callback_g1)
        self.service_client = self.create_client(Trigger, "/approach_shelf")
        self.publisher_cmd_vel = self.create_publisher(Twist, "/robot/cmd_vel", 10)
        self.publisher_elevator_up = self.create_publisher(elevator_msg, "/elevator_up", 10)
        self.publisher_elevator_down = self.create_publisher(elevator_msg, "/elevator_down", 10)
        self.publisher_gfootprint = self.create_publisher(Polygon, "/global_costmap/footprint", 10)
        self.publisher_lfootprint = self.create_publisher(Polygon, "/local_costmap/footprint", 10)

        # navigator callback
        self.navigator_timer = self.create_timer(1, self.navigator_callback, callback_group=self.callback_g2)

    def navigator_callback(self):
        self.get_logger().info(f"Starting robot movement, value use_sim_time : {self.use_sim_time}")

        # initialize robot position (global)
        init_position = self._create_pose("init_position")
        self.navigator.setInitialPose(init_position)
        self.get_logger().info(f"Initial position marked, check rviz!")

        # wait for Nav2 services
        self.navigator.waitUntilNav2Active(localizer="amcl_node", navigator="bt_navigator_node")

        # move towards loading point
        loading_position = self._create_pose("loading_position")
        status = self._go_to_next_pose(loading_position)
        self.get_logger().info(f"Reached loading_position : {status}")

        # load shelf cart on robot
        self._load_shelf_cart()

        # move towards shipping position
        postload_position = self._create_pose("postload_position")
        status = self._go_to_next_pose(postload_position)

        shipping_position = self._create_pose("shipping_position")
        status = self._go_to_next_pose(shipping_position)
        self.get_logger().info(f"Reached shipping_position : {status}")

        # unload shelf cart from robot
        self._unload_shelf_cart()

        # move towards initial position
        init_position = self._create_pose("init_position")
        status = self._go_to_next_pose(init_position)
        self.get_logger().info(f"Reached init_position : {status}")

        # destroy thread
        self.destroy_timer(self.navigator_timer)

    def _load_shelf_cart(self):
        # check if service is available
        counter = 0
        while not self.service_client.wait_for_service(1):
            counter += 1
            self.get_logger().warn(f"waiting for server /approach_shelf : {counter}")

        # send request to publish tf
        future = self.service_client.call(request=Trigger.Request())
        self.get_logger().info(f"Status /approach_shelf : {future.success}")

        # move robot under the shelf
        if future.success:
            if self.use_sim_time:
                self._rotate_robot_to_angle(angle=90)
                self._move_robot_by_distance(direction=-1, distance=0.40)
            else:
                # self._rotate_robot_by_angle(direction=1, angle=180)
                self._rotate_robot_by_time(direction=1, duration=4.40)
                self._move_robot_by_distance(direction=-1, distance=0.3)
        else:
            return

        # load shelf from robot
        message = Empty() if self.use_sim_time else String()
        for i in range(5):
            self.publisher_elevator_up.publish(message)
            # it takes time for robot to load shelf
            time.sleep(1)

        # cart dimension (0.85 0.8 0.05) radius (0.5836)
        message = self._polygon_message_rectangle(half_side=0.40)
        self.publisher_gfootprint.publish(message)
        self.publisher_lfootprint.publish(message)

        # move robot forward
        self._move_robot_by_distance(direction=1, distance=0.5) 

    def _unload_shelf_cart(self):
        # unload shelf from robot
        message = Empty() if self.use_sim_time else String()
        for i in range(5):
            self.publisher_elevator_down.publish(message)
            # it takes time for robot to unload shelf
            time.sleep(1)

        # robot dimension radius (0.15)
        message = self._polygon_message_rectangle(half_side=0.15)
        self.publisher_gfootprint.publish(message)
        self.publisher_lfootprint.publish(message)

        # move robot forward
        self._move_robot_by_distance(direction=1, distance=0.5)

    def _move_robot_by_distance(self, direction, distance):
        # initialize variable
        traveled_distance = 0
        x_prime = self.x
        y_prime = self.y

        self.get_logger().info(f"move_robot_by_distance : {direction*distance}")
        # control logic
        while(rclpy.ok()):
            dx = self.x - x_prime
            dy = self.y - y_prime
            traveled_distance += math.sqrt(dx * dx + dy * dy)
            x_prime = self.x
            y_prime = self.y
            if traveled_distance < distance:
                self._move(direction * 0.1, 0.0)
            else:
                break
        self.get_logger().info(f"traveled_distance: {traveled_distance}")
        
        # halt robot motion
        self._move(0.0, 0.0)

    def _rotate_robot_by_time(self, direction, duration):
        # initialize variable
        start_time = time.time()

        self.get_logger().info(f"start yaw {self.yaw})")
        # control logic
        while (time.time() - start_time) < duration:
            self.get_logger().info(f"yaw {self.yaw})", throttle_duration_sec=0.5)
            self._move(0.0, direction * 0.8)
        
        self.get_logger().info(f"end yaw {self.yaw})")
        # halt robot motion
        self._move(0.0, 0.0)

    def _rotate_robot_by_angle(self, direction, angle):
        # initialize variable
        rotate_yaw = 0.0
        yaw_prime = self.yaw
        
        self.get_logger().info(f"rotate_robot_by_angle : target {angle}")
        # control logic
        while(rclpy.ok()):
            rotate_yaw += abs(self.yaw - yaw_prime)
            yaw_prime = self.yaw
            if rotate_yaw < angle:
                self.get_logger().info(f"yaw {self.yaw})", throttle_duration_sec=0.5)
                self._move(0.0, direction * 0.5)
            else:
                break
        self.get_logger().info(f"rotate_yaw: {rotate_yaw}")
        
        # halt robot motion
        self._move(0.0, 0.0)

    def _rotate_robot_to_angle(self, angle):
        self.get_logger().info(f"rotate_robot_to_angle : target {angle}")
        # control logic
        while(rclpy.ok()):
            error_yaw = angle - self.yaw
            yaw_rad = (math.pi / 180) * error_yaw
            if abs(error_yaw) > 2.5:
                self.get_logger().info(f"yaw {self.yaw})", throttle_duration_sec=0.5)
                self._move(0.0, 0.5 * yaw_rad)
            else:
                break
        self.get_logger().info(f"error_yaw: {error_yaw}")
        
        # halt robot motion
        self._move(0.0, 0.0)

    def _move(self, x, z):
        # initialize message
        message = Twist()
        
        # define linear and angular velocity
        message.linear.x = x
        message.angular.z = z
        
        # publish message
        self.publisher_cmd_vel.publish(message)

    def _polygon_message_circle(self, radius):
        # initialize message
        message = Polygon()

        # define points
        for i in range(12):
            theta = i / 12.0 * 2.0 * math.pi
            x = radius * math.cos(theta) + 3.0
            y = radius * math.sin(theta)
            message.points.append(Point32(x=x, y=y))
        
        return message

    def _polygon_message_rectangle(self, half_side):
        # initialize message
        message = Polygon()
        
        # define points
        message.points.append(Point32(x=half_side, y=half_side))
        message.points.append(Point32(x=-half_side, y=half_side))
        message.points.append(Point32(x=-half_side, y=-half_side))
        message.points.append(Point32(x=half_side, y=-half_side))
        
        return message

    def _go_to_next_pose(self, position):
        # send taks to nav2
        self.navigator.goToPose(position)

        # wait till robot complete its assigned task
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback.number_of_recoveries > 5:
                self.get_logger().error(f"Task Canceled #{feedback.number_of_recoveries} recoveries.")
                self.navigator.cancelTask()
                return False
            else:
                self.get_logger().info(f"Time : {feedback.estimated_time_remaining}", once=True)             
        
        return True

    def _create_pose(self, request):
        # fetch the correct coordinates of target
        x, y, yaw = self.target_pose_values[request]

        # create ros2 position-orientation-stamped-message (PoseStamped)
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.navigator.get_clock().now().to_msg()

        # position
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = 0.0

        # orientation
        r = self._quaternion_from_euler(0, 0, yaw)
        message.pose.orientation.x = r[0]
        message.pose.orientation.y = r[1]
        message.pose.orientation.z = r[2]
        message.pose.orientation.w = r[3]
        return message

    def _quaternion_from_euler(self, roll, pitch, yaw):
        # calculate vector components
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # calculate quaternion (x, y, z, w)
        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr  # x
        q[1] = sy * cp * sr + cy * sp * cr  # y
        q[2] = sy * cp * cr - cy * sp * sr  # z
        q[3] = cy * cp * cr + sy * sp * sr  # w
        return q

    def _target_pose_values(self):
        if self.use_sim_time:
            return {
                "init_position": (-0.05, -0.03, 0.0),
                "loading_position": (5.8, 0.0, -1.57),
                "postload_position": (5.0, 0.0, 3.14),
                "shipping_position": (0.85, -3.0, 1.57),
            }
        else:
            return {
                "init_position": (-0.05, -0.4, 0.0),
                "loading_position": (4.3, -0.4, -1.57),
                "postload_position": (2.5, -0.8, 3.14),
                "shipping_position": (0.25, -3.0, 1.57),
            }

    def subscriber_odom_callback(self, msg):
        # reading current position from /odom topic
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # reading current orientation from /odom topic
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        # convert quaternion into euler angles
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        # fix: radian to degree conversions
        self.yaw = (180 / math.pi) * yaw

        # log the odom data
        # self.get_logger().info(f"odom (x {self.x}, y{self.y}, yaw {self.yaw})", throttle_duration_sec=2.0)


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    navigator = BasicNavigator()
    node = WarehouseBotNode(navigator=navigator)

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
