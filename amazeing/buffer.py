import rclpy
from rclpy.node import Node
import tf2_ros
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

class TFDebugNode(Node):
    def __init__(self):
        super().__init__('tf_debugger')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmdpub = self.create_publisher(Twist, '/cmd_vel',10)

        # self.create_timer(1.0, self.check_transform)

    # def check_transform(self):
    #     frames = self.tf_buffer.all_frames_as_yaml()
    #     self.get_logger().info(f"Available TF Frames:\n{frames}")

    #     if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time()):
    #         self.get_logger().info("✅ Transform map -> odom is available!")
    #     else:
    #         self.get_logger().warn("❌ Transform map -> odom NOT available!")

    def velpub(self):
        print('sent')
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 0.0
        self.cmdpub.publish(twist)

def main():
    rclpy.init()
    node = TFDebugNode()
    node.velpub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    # def update_odometry(self, msg: Union[Odometry, None] = None):
    #     # Updates the current pose of the robot. Callback bound to subscriber

    #     # transform_map_to_odom = self.tf_buffer.lookup_transform(
    #     #     'map',
    #     #     'odom',
    #     #     rclpy.time.Time(),
    #     #     timeout=Duration(seconds=1)
    #     # )
    #     # if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
    #     #     transform = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
    #     #     print("sucess")
    #     # print('fail')
    #     # transform_odom_to_base = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
    #     # transform_base_to_map = self.tf_buffer.transform(transform_odom_to_base, "map")
    #     # trans = transform_base_to_map.transform.translation
    #     # rot = transform_base_to_map.transform.rotation

    #     # print("next step")
    #     # self.pose = Pose(
    #     #     position=Point(x=trans.x, y=trans.y),
    #     #     orientation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w),
    #     # )
    #     return

    # @staticmethod
    # def neighbors_and_distances(
    #     mapdata: OccupancyGrid,
    #     p: "tuple[int, int]",
    #     directions: "list[tuple[int, int]]",
    #     must_be_walkable: bool = True,
    # ) -> "list[tuple[tuple[int, int], float]]":
    #     """
    #     Returns the neighbors cells of (x,y) in the occupancy grid given directions to check and their distances.
    #     :param mapdata           [OccupancyGrid] The map information.
    #     :param p                 [(int, int)]    The coordinate in the grid.
    #     :param directions        [[(int,int)]]   A list of directions to check for neighbors.
    #     :param must_be_walkable  [bool]          Whether or not the cells must be walkable
    #     :return                  [[(int,int)]]   A list of 4-neighbors.
    #     """
    #     neighbors = []
    #     for direction in directions:
    #         candidate = (p[0] + direction[0], p[1] + direction[1])
    #         if not must_be_walkable or PathPlanner.is_cell_walkable(mapdata, candidate):
    #             distance = PathPlanner.euclidean_distance(direction, (0, 0))
    #             neighbors.append((candidate, distance))
    #     return neighbors