#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rclpy
import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Pose, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from robot_controller.robot_utils import Utils




ERROR_ADMITIDO = 0.11


class TurtlebotController(Node):
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # Declare parameters
        self.declare_parameter('robot_vel_topic', 'cmd_vel')
        self.declare_parameter('robot_scan_topic', 'scan')
        self.declare_parameter('max_lin_vel', 0.2)
        self.declare_parameter('max_ang_vel', 0.4)
        self.declare_parameter('control_rate', 10)
        
        # Get parameters
        robot_vel_topic = self.get_parameter('robot_vel_topic').value
        robot_scan_topic = self.get_parameter('robot_scan_topic').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        control_rate = self.get_parameter('control_rate').value
        
        # Store the received path here
        self.path = Path()
        self.path_received = False
        self.laser = LaserScan()
        self.laser_received = False
        self.last_path_idx = 0
        self.path_signature = None
        
        # Declare the velocity command publisher
        self.cmd_vel = self.create_publisher(TwistStamped, robot_vel_topic, 10)
        
        # Create tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.utils = Utils(self.tf_buffer)
        
        # Subscription to the scan topic [sensor_msgs/LaserScan]
        self.create_subscription(LaserScan, robot_scan_topic, self.laser_callback, 10)
        
        # Subscription to the path topic [nav_msgs/Path]
        self.create_subscription(Path, 'path', self.path_callback, 10)
        
        # Subscription to odometry
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Create a timer for the control loop
        timer_period = 1.0 / control_rate  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)
        
     
    def odom_callback(self, msg):
        """Callback to receive and store odometry messages"""
        self.utils.set_odom(msg)

    def control_loop(self):
        """Main control loop executed at fixed rate"""
        end = self.command()
        if end:
            self.get_logger().info("Goal reached, stopping")
            self.destroy_timer(self.timer)

    def command(self):
        """
        Command the robot to follow the path
        Returns True if goal reached, False otherwise
        """
        if not self.path_received or len(self.path.poses) == 0:
            self.publish(0.0, 0.0)
            return False

        if self.goal_reached():
            self.get_logger().info("GOAL REACHED!!! Stopping!")
            self.publish(0.0, 0.0)
            return True

        goal_pose = self.get_sub_goal()
        if goal_pose is None:
            self.publish(0.0, 0.0)
            return False

        current_goal = goal_pose.pose.position
        self.get_logger().info('current_goal: %.03f, %.03f' % (current_goal.x, current_goal.y))

        distancia = math.sqrt(current_goal.x ** 2 + current_goal.y ** 2)
        theta = math.atan2(current_goal.y, current_goal.x)
        self.get_logger().info('Theta: %.4f' % (theta))

        if distancia < 0.05:
            linear = 0.0
            angular = 0.0
        elif abs(theta) > 1.4:
            linear = 0.0
            angular = 1.5 * theta
        else:
            linear = self.max_lin_vel * max(0.25, math.cos(theta))
            linear = min(linear, distancia)
            angular = linear * (2.0 * current_goal.y / (distancia ** 2))

        angular = self.constrain_vel(angular, -self.max_ang_vel, self.max_ang_vel)
        linear = self.constrain_vel(linear, 0.0, self.max_lin_vel)

        if not self.check_collision(linear, angular):
            self.publish(linear, angular)
            return False

        self.get_logger().info('Colision frontal detectada. Evadiendo...')
        linear, angular = self.collision_avoidance()
        self.publish(linear, angular)
        return False


    def goal_reached(self):
        """
        Exercise 1.2: Check if the final goal has been reached
        TODO: use the last point of the path to check if the robot
        has reached the final goal (the robot is in a close position).
        Returns True if the FINAL goal was reached, False otherwise
        """
        if not self.path_received or len(self.path.poses) == 0:
            return False

        pos_final = self.path.poses[-1]

        #self.get_logger().info('goal_reached: %s' % type(pos_final))
        try:
            base_goal = self.utils.transform_pose(pos_final,'base_footprint',self.get_logger()) #self.tf_buffer.transform(pos_final, 'base_footprint', timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return False

        if base_goal is None:
            return False

        coords_final = base_goal.pose.position

        if math.sqrt(coords_final.x ** 2 + coords_final.y ** 2) < ERROR_ADMITIDO:
            return True
        
        return False


    
    def get_sub_goal(self):
        """
        Exercise 1.1: Get the next sub-goal from the path
        TODO: use self.path.poses to find the subgoal to be reached
        You could transform the path points to the robot reference
        to find the closest point:
        path_pose = self.path.poses[index]
        path_pose_in_robot_frame = self.utils.transform_pose(
            path_pose, 'base_footprint', self.get_logger())
        """
        if not self.path_received or len(self.path.poses) == 0:
            return None

        min_dist = float('inf')
        closest_index = min(self.last_path_idx, len(self.path.poses) - 1)

        for i in range(closest_index, len(self.path.poses)):
            path_pose = self.path.poses[i]
            try:
                transformed_pose = self.utils.transform_pose(path_pose, 'base_footprint', self.get_logger())
                if transformed_pose is not None:
                    coords = transformed_pose.pose.position
                    dist = math.sqrt(coords.x**2 + coords.y**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_index = i
            except Exception:
                continue

        if min_dist == float('inf'):
            return None

        self.last_path_idx = closest_index
        best_pose = None

        distancia_lookahead = 0.4 
        for i in range(closest_index, len(self.path.poses)):
            try:
                transformed_pose = self.utils.transform_pose(self.path.poses[i], 'base_footprint', self.get_logger())
                if transformed_pose is not None:
                    coords = transformed_pose.pose.position
                    dist = math.sqrt(coords.x**2 + coords.y**2)
                    
                    if dist > distancia_lookahead:
                        best_pose = transformed_pose
                        break
            except Exception:
                continue

        if best_pose is None:
            best_pose = self.utils.transform_pose(self.path.poses[-1], 'base_footprint', self.get_logger())
            
        return best_pose


    def check_collision(self, linear, angular):
        """
        Copy from EPD2: Check for possible collisions
        TODO: use self.laser to check possible collisions
        Optionally, you can also use the velocity commands
        Returns True if possible collision, False otherwise
        """
        if not self.laser_received or len(self.laser.ranges) == 0 or linear <= 0.0:
            return False

        front_ranges = self.get_valid_scan_ranges(-math.pi / 4.0, math.pi / 4.0)
        if not front_ranges:
            return False

        distancia_seguridad = 0.32
        return min(front_ranges) < distancia_seguridad


    def collision_avoidance(self):
        goal = self.get_sub_goal()
        
        # Por seguridad, si no hay meta, nos paramos
        if goal is None:
            return 0.0, 0.0
        m_atrac = 1.5 
        m_rep = 0.015  
        rsoi = 0.55

        coords = goal.pose.position
        distancia_goal = math.sqrt(coords.x**2 + coords.y**2)
        
        # FUERZA DE ATRACCIÓN
        if distancia_goal > 0.0:
            F_atr_x = m_atrac * (coords.x / distancia_goal)
            F_atr_y = m_atrac * (coords.y / distancia_goal)
        else:
            F_atr_x = 0.0
            F_atr_y = 0.0

        # FUERZA DE REPULSIÓN
        F_rep_x = 0.0
        F_rep_y = 0.0
        
        for i, r in enumerate(self.laser.ranges):
            if self.laser.range_min < r < rsoi:
                theta_i = self.laser.angle_min + (i * self.laser.angle_increment)
                
                magnitud_i = m_rep * ((rsoi - r) / r)
                
                F_rep_x -= magnitud_i * math.cos(theta_i)
                F_rep_y -= magnitud_i * math.sin(theta_i)

        # SUMA TOTAL DE FUERZAS
        F_x = F_atr_x + F_rep_x
        F_y = F_atr_y + F_rep_y
            
        # CÁLCULO DE VELOCIDAD FINAL 
        f_theta = math.atan2(F_y, F_x) 
        f_mag = math.sqrt(F_x**2 + F_y**2) 
        
        ang_vel = f_theta
        
        if abs(f_theta) > 0.6:
            lin_vel = 0.0
        else:
            lin_vel = self.max_lin_vel * f_mag
        
        ang_vel = self.constrain_vel(ang_vel, -self.max_ang_vel, self.max_ang_vel)
        lin_vel = self.constrain_vel(lin_vel, 0.0, self.max_lin_vel)
        
        return lin_vel, ang_vel

    def constrain_vel(self, input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel


    def get_valid_scan_ranges(self, min_angle, max_angle):
        readings = []
        angle = self.laser.angle_min

        for distance in self.laser.ranges:
            normalized_angle = math.atan2(math.sin(angle), math.cos(angle))
            if min_angle <= normalized_angle <= max_angle:
                if (
                    not math.isnan(distance)
                    and not math.isinf(distance)
                    and self.laser.range_min < distance < self.laser.range_max
                ):
                    readings.append(distance)

            angle += self.laser.angle_increment

        return readings

        
    def publish(self, lin_vel, ang_vel):
        """Publish velocity commands to the robot"""
        move_cmd = TwistStamped()
        move_cmd.header.stamp = self.get_clock().now().to_msg()
        move_cmd.twist.linear.x = lin_vel
        move_cmd.twist.angular.z = ang_vel
        self.cmd_vel.publish(move_cmd)


    def laser_callback(self, data):
        """Callback to receive and store laser scan messages"""
        self.laser = data
        self.laser_received = True
        

    def path_callback(self, path):
        """Callback to receive and store path messages"""
        signature = self.get_path_signature(path)
        if signature != self.path_signature:
            self.last_path_idx = 0
            self.path_signature = signature

        self.path = path
        self.path_received = True


    def get_path_signature(self, path):
        if len(path.poses) == 0:
            return (0, path.header.frame_id)

        first = path.poses[0].pose.position
        last = path.poses[-1].pose.position
        return (
            len(path.poses),
            path.header.frame_id,
            round(first.x, 3),
            round(first.y, 3),
            round(last.x, 3),
            round(last.y, 3),
        )
        

    def shutdown_callback(self):
        """Shutdown callback to stop the robot"""
        self.get_logger().info("Stop TurtleBot")
        self.publish(0.0, 0.0)
 

def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the controller
    robot_controller = TurtlebotController()
    
    robot_controller.get_logger().info("TurtleBot controller started")
    robot_controller.get_logger().info("To stop TurtleBot press CTRL+C")
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("TurtleBot controller stopped by user")
    finally:
        robot_controller.shutdown_callback()
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
