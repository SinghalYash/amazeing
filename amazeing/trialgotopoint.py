#!/usr/bin/env python3
from typing import Union
import argparse
import math
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Point, Twist, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from tf2_ros import Time
from time import sleep

# Helper function to extract the sign of a number
# Any value below 0 is considered negative
def sign(value):
    if value < 0:
        return -1
    else:
        return 1


# Helper function to wrap an angle to the range [-pi, pi]
def wrap(angle):
    while abs(angle) > math.pi:
        angle = angle - 2 * math.pi * sign(angle)
    return angle


class GoToPoint(Node):

    def __init__(self):
        """
        Class constructor
        """
        # Initialize node, name it 'go_to_point'
        super().__init__("go_to_point")

        # Set the rate
        # self.rate = rospy.Rate(10)

        # Attributes to keep track of current position
        # self.px = 0.0
        # self.py = 0.0
        self.dir = 0.0

        # Attributes to keep track of the goal
        self.goal = None
        self.goal_updated = False

        # Attribute to keep track of the distance to the wall
        self.too_close = False

        # Create a TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(2.0, self.check_transform)

        # Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.speed_pub = self.create_publisher(Twist,"/cmd_vel" , 10)

        # Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # When a message is received, call self.update_odometry
        self.odom_sub = self.create_subscription(Odometry, "/odom" , self.update_odometry, qos_profile_sensor_data)

        # Tell ROS that this node subscribes to Point messages on the '/go_to_point/goal' topic
        self.point_sub = self.create_subscription(Point, "/go_to_point/goal", self.update_goal,10)

        # Check if the robot is too close to the wall
        self.lidarsub = self.create_subscription(LaserScan, "/scan" , self.check_too_close, qos_profile_sensor_data)
        
        self.pure_pursuit_sub = self.create_subscription(Path, "/pure_pursuit/path", self.update_path, qos_profile_sensor_data)

        self.LOOKAHEAD_DISTANCE = 0.10  # m
        self.pose = None
        self.path = Path()



    
    def update_path(self, msg: Path):
        self.path = msg
        self.get_logger().info(f"Received path message with {len(self.path.poses)} poses")
        for i, pose in enumerate(self.path.poses):
            self.get_logger().info(f"Waypoint {i}: ({pose.pose.position.x}, {pose.pose.position.y})")
    
    def check_transform(self):
        if self.tf_buffer.can_transform("map", "odom", Time()):
            self.timer = self.create_timer(1.0, self.update_odometry)  # Start main loop

        else:
            self.get_logger().warn("odom NOT available!")

    def update_goal(self, msg: Point):
        """
        Updates the goal point for the robot to reach.
        This method is a callback bound to a Subscriber.
        :param msg [Point] The new goal point.
        """
        self.goal = msg
        self.goal_updated = True

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        if self.too_close:
            linear_speed = 0.0
            angular_speed = 0.0
            self.get_logger().info("Too close to the wall")

        # Make a new Twist message
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        print("sent speed", twist)
        # Publish the message
        self.speed_pub.publish(twist)

    def drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line. If the speed is negative, the robot moves backwards. Negative distances will be ignored
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # Note the starting position
        start_x = self.pose.x
        start_y = self.pose.y

        # Publish the movement to the '/cmd_vel' topic
        # Continuously check if we have covered the distance
        while (self.pose.x - start_x) ** 2 + (self.pose.y - start_y) ** 2 < distance**2:
            self.send_speed(linear_speed, 0.0)
            print("speed 0")
            sleep(1/6)
            # self.rate.sleep()

        # Stop the robot
        self.send_speed(0.0, 0.0)

    def rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param aspeed [float] [rad/s] The angular speed.
        """
        # Wrap the angle to the range [-pi, pi]
        angle = wrap(angle)

        # Invert the move speed if needed
        aspeed = aspeed if angle > 0 else -aspeed

        # Publish the movement to the '/cmd_vel' topic
        # Continuously check if we have covered the angle
        start_dir = self.dir
        while abs(wrap(self.dir - start_dir) - angle) > 0.1:
            self.send_speed(0.0, aspeed)
            # if self.goal_updated:
            #     break
            # self.rate.sleep()
            sleep(1/6)

        # Stop the robot
        self.send_speed(0.0, 0.0)

    def go_to(self, goal: Point, linear_speed: float = 0.1, angular_speed: float = 0.5):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param goal [Point] The target point.
        :param linear_speed [float] [m/s] The maximum forward linear speed. Should be positive.
        :param angular_speed [float] [rad/s] The maximum angular speed. Should be positive.
        """
        # Calculate the angle to the target point
        initial_angle = math.atan2(goal.y - self.pose.y, goal.x - self.pose.x) - self.dir

        # Wrap the angle to the range [-pi, pi]
        initial_angle = wrap(initial_angle)

        # Attempt to optimize the direction of the robot
        # Instead of turning 180 degrees, we can turn 180 - angle degrees and drive backwards
        drive_dir = 1
        if abs(initial_angle) > math.pi / 2:
            initial_angle += math.pi
            drive_dir = -1

        # Execute the robot movements to reach the target pose
        self.rotate(initial_angle, angular_speed)
        self.smooth_drive(
            goal.x,
            goal.y,
            linear_speed * drive_dir,
        )

    def update_odometry(self, msg: Union[Odometry,None] = None):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        try:
            transform_base_to_map = self.tf_buffer.lookup_transform("map", "base_footprint", Time(), Duration(seconds=1))
            trans = transform_base_to_map.transform.translation
            rot = transform_base_to_map.transform.rotation
            self.pose = Pose(
                position=Point(x=trans.x, y=trans.y),
                orientation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w),
            )
#            print('pose updated')

        except tf2_ros.TransformException or tf2_ros.LookupException:
            print("didnt work")
        # trans = [0, 0]
        # rot = [0, 0, 0, 0]
        # try:
        #     (trans, rot) = self.tf_listener.lookupTransform(
        #         "/map", "/base_footprint", rospy.Time(0)
        #     )
        # except (
        #     tf.LookupException,
        #     tf.ConnectivityException,
        #     tf.ExtrapolationException,
        # ):
        #     print("Error running tf transform")
        # self.px = trans[0]
        # self.py = trans[1]

        # (roll, pitch, self.dir) = euler_from_quaternion(rot)

    def smooth_drive(self, goal_x: float, goal_y: float, linear_speed: float):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param goal_x       [float] [m]   The target x-coordinate.
        :param goal_y       [float] [m]   The target y-coordinate.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # Note the starting position
        start_x = self.pose.x
        start_y = self.pose.y

        # Calculate the distance
        distance = math.sqrt((goal_x - start_x) ** 2 + (goal_y - start_y) ** 2)

        # Publish the movement to the '/cmd_vel' topic
        # Adjust the speed smoothly over time
        # Continuously check if we have covered the distance
        current_speed = 0.0
        dist_tol = 0.01  # m
        while (
            abs(
                distance
                - math.sqrt((self.pose.x - start_x) ** 2 + (self.pose.y - start_y) ** 2)
            )
            > dist_tol
        ):
            # Calculate the remaining distance
            remaining_dist = distance - math.sqrt(
                (self.pose.x - start_x) ** 2 + (self.pose.y - start_y) ** 2
            )

            # Forward project the robot's position
            # By making this point follow the line between the start and goal points,
            # the robot will follow the line like a trailer follows a truck
            # The larger the projection distance, the smoother the movement, but the slower the correction
            proj_dist = 0.1 * sign(linear_speed)  # m
            forward_x = self.pose.x + proj_dist * math.cos(self.dir)
            forward_y = self.pose.y + proj_dist * math.sin(self.dir)
            forward_pt = np.array([forward_x, forward_y])

            # Find the distance from the forward point to the line
            # This distance is the error that we want to correct
            start_pt = np.array([start_x, start_y])
            goal_pt = np.array([goal_x, goal_y])
            heading_error = np.cross(
                goal_pt - start_pt, forward_pt - start_pt
            ) / np.linalg.norm(goal_pt - start_pt)

            # Calculate the heading correction by scaling the error
            headingP = -2.5 * math.pi  # rad/s/m
            heading_correction = headingP * heading_error

            # Initial acceleration
            # Make sure that the robot hasn't made it halfway to the target
            # Otherwise we could continue accelerating and overshoot the target
            if abs(current_speed) < abs(linear_speed) and remaining_dist > distance / 2:
                # Increase the speed by a constant acceleration
                # Pull the sign from the linear speed
                accel = 0.1  # m/s^2
                current_speed += (
                    accel * sign(linear_speed)
                )
                #* self.rate.sleep_dur.to_sec() 
                self.send_speed(current_speed, heading_correction)
                sleep(1/6)

            else:
                # Simple P controller to adjust the speed
                # Note that we have to integrate the sign of linear speed into the controller
                kP = 0.75  # 1/m
                desired_speed = kP * remaining_dist * sign(linear_speed)

                # Cap the desired speed to the maximum speed
                # Extract the sign of the desired speed to preserve the direction
                if abs(desired_speed) > abs(linear_speed):
                    desired_speed = abs(linear_speed) * sign(desired_speed)

                # Send over the speed
                # If the desired speed is higher than the maximum speed, send the maximum speed
                self.send_speed(desired_speed, heading_correction)
                sleep(1/6)

            # if self.goal_updated:
            #     break

        # Stop the robot
        self.send_speed(0.0, 0.0)

    def check_too_close(self, msg: LaserScan):
        # Remove the ranges that are smaller than the min range
        valid_ranges = [
            r for r in msg.ranges if r > msg.range_min and r < msg.range_max
        ]

        # Check the valid ranges against the threshold
        threshold = 0.1  # m?
        if min(valid_ranges) < threshold:
            self.too_close = True
            print("WALL!")
        else:
            self.too_close = False

    def get_goal(self) -> Point:
        # try:
        if not self.path.poses:
            return Point()

        poses = self.path.poses
        print("in goal function", len(poses))
        return poses[len(poses) - 1].pose.position
        # except IndexError:
        #     return Point()

    def find_lookahead(self, nearest_waypoint_index, lookahead_distance) -> Point:
        if not self.path.poses:
            return Point()

        i = nearest_waypoint_index
        while (
            i < len(self.path.poses)
            and self.get_distance_to_waypoint_index(i) < lookahead_distance
        ):
            i += 1
        return self.path.poses[i - 1].pose.position
    
    def find_nearest_waypoint_index(self) -> int:
        nearest_waypoint_index = -1
        if self.path.poses is None:
            return nearest_waypoint_index

        closest_distance = float("inf")
        for i in range(len(self.path.poses) - 1):
            distance = self.get_distance_to_waypoint_index(i)
            if distance and distance < closest_distance:
                closest_distance = distance
                nearest_waypoint_index = i
        print(nearest_waypoint_index)
        return nearest_waypoint_index
    
    def get_distance_to_waypoint_index(self, i: int) -> float:
        if self.pose is None or self.path.poses is None:
            return -1

        position = self.pose.position
        waypoint = self.path.poses[i].pose.position
        return math.sqrt((waypoint.x - position.x) ** 2 + (waypoint.y - position.y) ** 2)

    def run(self):
        """
        Runs the node until it is stopped.
        """
        while rclpy.ok():
            try:
                if self.pose is None:
                    rclpy.spin_once(self)
                    print("pose not got")

                # If no path, stop ### doesnt exist for this program
                # if self.path is None or not self.path.poses:
                #     print(self.path.poses)
                #     rclpy.spin_once(self)
                #     print("no path or no or self.path.poses, stopping")
                #     self.stop()
                
                goal = self.get_goal()

                nearest_waypoint_index = self.find_nearest_waypoint_index()
                lookahead = self.find_lookahead(
                        nearest_waypoint_index, self.LOOKAHEAD_DISTANCE
                    )
                self.goal = lookahead
                print("this is lookahead ",lookahead)
                # self.lookahead_pub.publish(
                #         PointStamped(header=Header(frame_id="map"), point=lookahead)
                #     )
            # If the goal has been updated, move to the new goal
                if self.goal:
                    print("inside going to goal")
                    # self.goal_updated = False
                    self.go_to(self.goal)
                    print("executed go to goal")
                
                rclpy.spin_once(self)
            except Exception as e:
                rclpy.shutdown()
                self.destroy_node()


if __name__ == "__main__":
    rclpy.init()
    gtp = GoToPoint()
    gtp.run()
    