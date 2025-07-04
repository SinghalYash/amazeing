import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Header


class nav2_go_to_pose(Node):
    def __init__(self):
        
        super().__init__("nav2_travel_node")

        self.posepub = self.create_publisher(PoseStamped,"/goal_pose",10)

    
    def fn(self):
        while rclpy.ok():
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
        #     msg.pose.position.x = i.x
        #     msg.pose.position.y = i.y
            msg.pose.position.z = 0.0
            msg.pose.orientation.w = 1.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            
            self.posepub.publish(msg)
            print("goal published")

if __name__ == "__main__":
    rclpy.init()

    node = nav2_go_to_pose()
    node.fn()
    rclpy.spin(node)
    rclpy.shutdown()
