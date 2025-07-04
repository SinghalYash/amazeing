#!/usr/bin/env python3

import os
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import threading
import subprocess
import numpy as np
from typing import Union
from path_planner import PathPlanner
from frontier_search import FrontierSearch
from nav_msgs.msg import OccupancyGrid, Path, GridCells, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped,Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from custom_interfaces.msg import FrontierList
import rclpy.time
import tf2_ros
import cmath
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
from time import sleep, time
import math
import scipy.interpolate as si


class FrontierExploration(Node):
    
    def __init__(self):
        super().__init__("frontier_exploration")
#Class constructor

        # Set if in debug mode
        self.is_in_debug_mode = True

        # Publishers
        self.pure_pursuit_pub = self.create_publisher(Path, "/pure_pursuit/path", 10)

#        if self.is_in_debug_mode:
        self.frontier_cells_pub = self.create_publisher(GridCells, "/frontier_exploration/frontier_cells", 10)
        self.start_pub = self.create_publisher(GridCells, "/frontier_exploration/start", 10)
        self.goal_pub = self.create_publisher(GridCells, "/frontier_exploration/goal", 10)
        self.cspace_pub = self.create_publisher(OccupancyGrid, "/cspace", 10)
        self.publisher_ = self.create_publisher(OccupancyGrid, 'cost_map', 10,)
        self.posepub = self.create_publisher(PoseStamped,"/goal_pose",10)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel",10)

        # Subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            "odom",
            self.get_odom,
            10
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "map",
            self.update_map,
            qos_profile_sensor_data
        )
        self.navsub = self.create_subscription(Bool,"/navigatingyesno",self.update_nav,10)

        self.lidarsub = self.create_subscription(LaserScan, "/scan",self.lidarcallback, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(2.0, self.check_transform)


        #self.lock = threading.Lock()
        self.pose = None
        self.map = None
        self.is_navigating = Bool()
        self.is_navigating.data = False

        self.NUM_EXPLORE_FAILS_BEFORE_FINISH = 30
        self.no_path_found_counter = 0
        self.no_frontiers_found_counter = 0
        self.is_finished_exploring = False
        self.obstacle = 0
        self.worstcase = 0

        self.spin_thread = threading.Thread(target=rclpy.spin,args=(self,))
        self.spin_thread.daemon = True
        self.spin_thread.start()    

    def check_transform(self):
        if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time()):
            self.timer = self.create_timer(2.0, self.get_odom)  # Start main loop
        else:
            self.get_logger().info("warning")
    
    def lidarcallback(self, msg:LaserScan):
        self.laser_range = np.array(msg.ranges)
        self.laser_range[self.laser_range==0] = np.nan
        if self.laser_range[0] < 0.2:
            self.obstacle = True

    def velchecker(self,msg:Twist):

        self.velocityx = msg.linear.x
        self.velocityy = msg.linear.y

    def get_odom(self, msg: Union[Odometry, None] = None):
        try:
            # transform_map_to_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(), Duration(seconds=1))
            # transform_odom_to_base = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            # transform_base_to_map = self.tf_buffer.transform(transform_odom_to_base, "map")
            transform_base_to_map = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), Duration(seconds=1))
            trans = transform_base_to_map.transform.translation
            rot = transform_base_to_map.transform.rotation

            self.pose = Pose(
                position=Point(x=trans.x, y=trans.y),
                orientation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w),
            )
            orientation = self.pose.orientation
            roll, pitch, yaw = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
            self.yaw = yaw
            #print(self.pose.position)

        except tf2_ros.TransformException or tf2_ros.LookupException:
            print("didnt work")


    def update_map(self, msg: OccupancyGrid):
        #Updates the current map.
        #his method is a callback bound to a Subscriber.
        #param msg [OccupancyGrid] The current map information.
        self.map = msg
        self.resolution = self.map.info.resolution
        self.originX = self.map.info.origin.position.x
        self.originY = self.map.info.origin.position.y
        self.width = self.map.info.width
        self.height = self.map.info.height
        self.data = self.map.data

    def update_nav(self,msg:Bool):
        self.is_navigating = msg

    def save_map(self):
        # Get the path of the current package
        package_path = get_package_share_directory("amazeing")

        # Construct the path to the map
        map_path = os.path.join(package_path, "map/map")
        if not os.path.exists(os.path.dirname(map_path)):
            os.makedirs(os.path.dirname(map_path))

        # Run map_saver
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", map_path])

        self.get_odom()

        if self.pose is None:
            self.get_logger().info("Failed to get pose")
            return

        # Save the robot's position and orientation
        position = self.pose.position
        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        with open(os.path.join(package_path, "map/pose.txt"), "w") as f:
            f.write(f"{position.x} {position.y} {position.z} {yaw} {pitch} {roll}\n")

    @staticmethod
    def get_top_frontiers(frontiers, n):
        # Sort the frontiers by size in descending order
        sorted_frontiers = sorted(
            frontiers, key=lambda frontier: frontier.size, reverse=True
        )

        # Return the top n frontiers
        return sorted_frontiers[:n]

    # def publish_cost_map(self, mapdata: OccupancyGrid, cost_map: np.ndarray):
    #     # Create an OccupancyGrid message
    #     grid = OccupancyGrid()
    #     grid.header.stamp = self.get_clock().now().to_msg()
    #     grid.header.frame_id = "map"
    #     grid.info.resolution = mapdata.info.resolution
    #     grid.info.width = cost_map.shape[1]
    #     grid.info.height = cost_map.shape[0]
    #     grid.info.origin = mapdata.info.origin

    #     # Normalize the cost map to the range [0, 100] and convert it to integers
    #     # cost_map_normalized = (cost_map / np.max(cost_map) * 100).astype(np.int8)

    #     # # Flatten the cost map and convert it to a list
    #     # grid.data = cost_map_normalized.flatten().tolist()
    #     grid.data = cost_map

    #     # Publish the OccupancyGrid message
    #     self.publisher_.publish(grid)


    def calc_costmap(self,data,width,height,resolution):
        data = np.array(data).reshape(height,width)
        print(data)
        wall = np.where(data == 100)
        print(wall)
        for i in range(-2,2+1):
            for j in range(-2,2+1):
                if i  == 0 and j == 0:
                    continue
                x = wall[0]+i
                y = wall[1]+j
                x = np.clip(x,0,height-1)
                y = np.clip(y,0,width-1)
                data[x,y] = 100
        data = data*resolution

        return data

    def check_if_finished_exploring(self):
        # Publish empty path to stop the robot
        self.pure_pursuit_pub.publish(Path())

        # If no frontiers or paths are found for a certain number of times, finish exploring
        if (
            self.no_frontiers_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
            or self.no_path_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
        ):
            self.get_logger().info("Done exploring!")
            # self.save_map()
            self.get_logger().info("Saved map")
            self.is_finished_exploring = True

    def explore_frontier(self, frontier_list: FrontierList):
        # If finished exploring, no pose, no map, or no frontier list, return
        if self.is_finished_exploring or self.pose is None or self.map is None:
            return

        frontiers = frontier_list.frontiers

        # If no frontiers are found, check if finished exploring
        if not frontiers:
            self.get_logger().info("No frontiers")
            self.no_frontiers_found_counter += 1
            self.check_if_finished_exploring()
            return
        else:
            self.no_frontiers_found_counter = 0

        A_STAR_COST_WEIGHT = 10.0
        FRONTIER_SIZE_COST_WEIGHT = 1.0

        self.costmap = self.calc_costmap(self.data,self.width,self.height,self.resolution)
        # self.publish_cost_map(self.map, costmap)

        # # Calculate the C-space
        cspace, cspace_cells = PathPlanner.calc_cspace(self.map, self.is_in_debug_mode)
        # if cspace_cells is not None:
        self.cspace_pub.publish(cspace)

        # # Calculate the cost map
        # cost_map = PathPlanner.calc_cost_map(self.map)
        # if self.is_in_debug_mode:
        #     self.publish_cost_map(self.map, cost_map)

        # Get the start

        # Execute A* for every frontier
        lowest_cost = float("inf")
        second_lowest_cost = float("inf")
        best_path = None
        second_best_path = None


        # Check only the top frontiers in terms of size
        MAX_NUM_FRONTIERS_TO_CHECK = 8
        top_frontiers = FrontierExploration.get_top_frontiers(
            frontiers, MAX_NUM_FRONTIERS_TO_CHECK
        )

        starts = []
        goals = []
        paths = []
        frontierlist = []

        # Log how many frontiers are being explored
        self.get_logger().info(f"Exploring {len(top_frontiers)} frontiers")

        for frontier in top_frontiers:
            # Get goal coordinates
            print("sent to planner")
            frontierlist.append(frontier.centroid)
            goal = PathPlanner.world_to_grid(self.map, frontier.centroid)
            print(goal)
            goals.append(goal)
            print(frontierlist)
            # Execute A*
            # path, a_star_cost, start, goal = PathPlanner.a_star(
            #     cspace, cost_map, start, goal
            # )
            # print(path)

            # If in debug mode, append start and goal
            if self.is_in_debug_mode:

                goals.append(goal)

            # if path is None or a_star_cost is None:
            #     continue

            # Calculate cost
            # cost = (A_STAR_COST_WEIGHT * a_star_cost) + (
            #     FRONTIER_SIZE_COST_WEIGHT / frontier.size
            # )

            #Update best path
            # if cost < lowest_cost:
            #     lowest_cost = cost
            #     best_path = path
                # if second_lowest_cost > lowest_cost and second_lowest_cost < cost:
                #     second_best_path = path
                #     second_lowest_cost = cost
                # if cost > lowest_cost and cost 
        # if paths: 
        #     for i in range(1,len(paths)):
        #         print('entered paths loop')
        #         best_path = paths[i]
        

        #     while rclpy.ok():
        #         self.posepub.publish(msg)

        #         #if goal reached break

        #         print(self.pose.position.x)

        #         x_err = i.x - self.pose.position.x
        #         y_err = i.y - self.pose.position.y
        #         if x_err < 0.3 and y_err < 0.3:

        #             break
        # If in debug mode, publish the start and goal
        # if self.is_in_debug_mode:
        #     print("published start and goal")
        #     print(goals)
        #     self.start_pub.publish(PathPlanner.get_grid_cells(self.map, starts))
        #     self.goal_pub.publish(PathPlanner.get_grid_cells(self.map, goals))

        # If a path was found, publish it
        for i in goals:
            start = PathPlanner.world_to_grid(self.map, self.pose.position)

            path = PathPlanner.astar(self.costmap, start, i)
            
            #path = self.bspline_planning(i, len(i)*5)
            goal = i
            print(goal)
            self.mover(path,i,0)


            #self.get_logger().info(f"Found best path with cost {lowest_cost}")
            # path = PathPlanner.path_to_message(self.map, i)   #converts best_path which is a list of tuples of coordinates into a ros path message
            # self.pure_pursuit_pub.publish(path)
            print("moving to next goal")
           
            self.no_path_found_counter = 0
        # If no path was found, check if finished exploring
            sleep(1)

                
                
        #     # best_path.reverse()

        #     # print("returning back")
        #     # print(best_path)
        #     # path = PathPlanner.path_to_message(self.map, best_path)   #converts best_path which is a list of tuples of coordinates into a ros path message
        #     # self.pure_pursuit_pub.publish(path)

        # else:

        #     self.get_logger().info("No paths found")
        #     self.no_path_found_counter += 1
            #     self.check_if_finished_exploring()
    def bspline_planning(self,array, sn):
        try:
            array = np.array(array)
            x = array[:, 0]
            y = array[:, 1]
            N = 2
            t = range(len(x))
            x_tup = si.splrep(t, x, k=N)
            y_tup = si.splrep(t, y, k=N)

            x_list = list(x_tup)
            xl = x.tolist()
            x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

            y_list = list(y_tup)
            yl = y.tolist()
            y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

            ipl_t = np.linspace(0.0, len(x) - 1, sn)
            rx = si.splev(ipl_t, x_list)
            ry = si.splev(ipl_t, y_list)
            path = [(rx[i],ry[i]) for i in range(len(rx))]
        except:
            path = array
        return path




    def localcontroller(self):
        """
        Checks for obstacles and adjusts movement accordingly.
        """
        print("In local controller")

        linvel = 0.1  # Forward velocity
        angvel = 0.0  # Default no rotation
        obstacle_threshold = 0.2  # Distance at which we react to obstacles

        # Get front, left, and right laser scans
        left_scan = self.laser_range[:int(len(self.laser_range)/8)]  # First 1/8
        right_scan = self.laser_range[int(7/8*len(self.laser_range)):]  # Last 1/8
        front_scan = self.laser_range[int(len(self.laser_range)/3):int(2*len(self.laser_range)/3)]  # Front view

        # If an obstacle is too close in front, stop and decide a new direction
        if np.min(front_scan) < obstacle_threshold:
            print("Obstacle ahead! Stopping and turning.")
            linvel = 0.0
            ang = 90  # Turn in place to find a free path

        # If an obstacle is detected on the left, turn right
        elif np.min(left_scan) < obstacle_threshold:
            print("Obstacle on left! Turning right.")
            angvel = -90

        # If an obstacle is detected on the right, turn left
        elif np.min(right_scan) < obstacle_threshold:
            print("Obstacle on right! Turning left.")
            angvel = 90

        return linvel, ang  # Return movement values


        # self.worstcase = np.nanargmax(laser_range)


        # self.send_speed(0.0,0.0)
        # sleep(1)
        # if self.obstacle == 60:
  

        # # if self.worstcase < 180:
        # #     alpha = self.worstcase
        # #     alpha = alpha*(math.pi/180)
        # # if self.worstcase > 180:
        # #     alpha = -(self.worstcase-180)
        # #     alpha *= alpha*(math.pi/180)
        # if self.obstacle == 300:

            
        self.rotatebot(alpha)
        timeend = time() + 0.1
        while time()<timeend:
            self.send_speed(linvel,0.0)

        sleep(1)
    

    def mover(self,path,goal,fails):
        # self.get_odom()
        if self.yaw == 0:
            self.goalreached = False
            return 
        if path is None:
            print('no path yet')
            return
        if fails>3:
            return
        
        self.is_navigating.data = True

        for i in range(3,len(path)-1,2):
            try:

                coord = PathPlanner.grid_to_world(self.map,path[i])
                lookaheady = coord.x
                lookaheadx = coord.y
                print(lookaheadx,lookaheady)
                gridyourown = PathPlanner.world_to_grid(self.map,self.pose.position)
                print(gridyourown)

                #worldtogridoftarget = PathPlanner.world_to_grid(self.map,self.path.poses[i].pose.position)
                # yourowny = self.pose.position.x
                # yourownx = self.pose.position.y
                #print(worldtogridoftarget)
                print(self.pose.position)
                y_diff = path[i][1]-gridyourown[1]
                x_diff = path[i][0]-gridyourown[0]

                # print(distance)

                self.alpha = math.atan2(y_diff,x_diff)
                print(self.alpha)
                self.rotatetoangle(self.alpha)

                # alpha = math.pi/3   #self.alpha

                while True:
                    if self.obstacle == True:
                        self.send_speed(0.0, 0.0)
                        fails += 1
                        print(fails)
                        speed, angle = self.localcontroller()
                        self.rotatebot(angle)
                        print("sent to local controller")

                        new_start = PathPlanner.world_to_grid(self.map, self.pose.position)
                        new_path = PathPlanner.astar(self.costmap, new_start, goal)

                        print(new_path)

                        if new_path:
                            self.mover(new_path,fails)  # Restart the function with new path
                            return

                    distance = math.sqrt((lookaheady-self.pose.position.y)**2 + (lookaheadx-self.pose.position.x)**2)

                    if distance < 0.09:
                        break
                    
                    print("distance to goal is",distance)

                    self.send_speed(0.1,0.0)
                    # self.send_speed(0.3,0)
                    # print("sent speed")

                    sleep(0.1)


                self.send_speed(0.0,0.0)
                print("sent stop")

            except Exception as e:
                return
            
        print("goal reached")
        self.send_speed(0.0,0.0)
        twist = Twist(
            linear=Vector3(x=float(0), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(0))
        )
        self.cmd_vel_pub.publish(twist)

        
        self.path = None

    def rotatetoangle(self, alpha):
        angle = alpha - self.yaw
        angvel = 0.4
        print(self.yaw)
        print(alpha)

        if abs(angle) > 0.1:
            
            if angle > 0:

                # if alpha > 0 and self.yaw < 0:
                #     angle = math.pi+self.yaw + (math.pi - alpha)
                #     timeend = time() +angle/angvel
                #     while time() < timeend:
                #         self.send_speed(0.0,-0.4)

                # print(time())
                timeend = time() + angle/angvel
                # print(timeend)
            
                while time() < timeend:
                    self.send_speed(0.0,0.4)
                    # print("rotation sent")
            else:
                # if alpha < 0 and self.yaw > 0:
                #     angle = math.pi-self.yaw + (math.pi + alpha)
                #     timeend = time() +angle/angvel
                #     while time() < timeend:
                #         self.send_speed(0.0,0.4)
                # print(time())
                timeend = time() + abs(angle)/angvel
                # print(timeend)
            
                while time() < timeend:
                    self.send_speed(0.0,-0.4)
                    # print("rotation sent")         

    


    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # print("send_speed called")
        # print(linear_speed,angular_speed)
        twist = Twist(
            linear=Vector3(x=float(linear_speed), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(angular_speed))
        )
        self.cmd_vel_pub.publish(twist)

    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * 0.1
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)



    


    def run(self):
        try:
            # print("im working")
            # if self.is_navigating.data == True:
            #     print("robot is moving")
            #     return
            if self.pose is None or self.map is None:
                #rclpy.spin_once(self)
                return
            # Get the start position of the robot

            start = PathPlanner.world_to_grid(self.map, self.pose.position)
            if start != None:
                print(start)
            # Get frontiers
            frontier_list, frontier_cells = FrontierSearch.search(self.map, start, self.is_in_debug_mode)

            if frontier_list is None:
                #rclpy.spin_once(self)
                return
            
        # Publish frontier cells if in debug mode
            if self.is_in_debug_mode:
                self.frontier_cells_pub.publish(PathPlanner.get_grid_cells(self.map, frontier_cells))
            
            self.explore_frontier(frontier_list)
            sleep(0.1)
            # rclpy.spin_once(self)
        except KeyboardInterrupt:
            self.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    frontier_explorer = FrontierExploration()
    while rclpy.ok():
        frontier_explorer.run()
    #frontier_explorer.check_transform()
    # rclpy.spin(frontier_explorer)

