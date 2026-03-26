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

ERROR_ADMITIDO = 0.1


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
        if self.path_received:
            # Check if the final goal has been reached
            # TODO: Exercise 1:implement goal reached check
            if self.goal_reached():
                self.get_logger().info("GOAL REACHED!!! Stopping!")
                self.publish(0.0, 0.0)
                return True
            
            # Determine the local path point to be reached
            # TODO: Exercise 1: fill the method get_sub_goal
            current_goal = self.get_sub_goal()
            current_goal = current_goal.pose.position
            self.get_logger().info('current_goal: %.03f' % current_goal.x)
            self.get_logger().info('current_goal: %.03f' % current_goal.y)

            theta = math.atan2(current_goal.y, current_goal.x)
            linear = 0.0
            angular = 0.0
            self.get_logger().info('Theta: %.4f' % (theta))
            if abs(current_goal.x) > 0.01 or abs(current_goal.y) > 0.01:
                if abs(theta) > 0.01:
                    angular = self.max_ang_vel * theta
                else :
                    angular = 0.0
                    linear = self.max_lin_vel * math.sqrt(current_goal.x ** 2 + current_goal.y ** 2)  
                
            # TODO: use current_goal 
            # Put your control law here (copy from EPD1)
            
            # Check the maximum speed values allowed
            angular = self.constrain_vel(angular, -self.max_ang_vel, self.max_ang_vel)
            linear = self.constrain_vel(linear, 0.0, self.max_lin_vel)
                
            # If the computed commands does not provoke a collision,
            # send the commands to the robot
            # TODO: fill the check_collision function (copy from EPD2)
            if not self.check_collision(linear, angular):
                self.get_logger().info('Colision')
                self.publish(linear, angular)
                return False

            # If a possible collision is detected,
            # try to find an alternative command to avoid the collision
            # TODO: Exercise 2: fill the collision_avoidance function
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
        pos_final = self.path.poses[-1]

        #self.get_logger().info('goal_reached: %s' % type(pos_final))
        try:
            base_goal = self.utils.transform_pose(pos_final,'base_footprint',self.get_logger()) #self.tf_buffer.transform(pos_final, 'base_footprint', timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return

        coords_final = base_goal.pose.position

        if abs(coords_final.x) < ERROR_ADMITIDO and abs(coords_final.y) < ERROR_ADMITIDO:
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
        #self.get_logger().info('%d' % len(self.path.poses))
        """next_point = self.utils.transform_pose(self.path.poses[0], 'base_footprint', self.get_logger())
        j = sys.maxsize
        for i in range(len(self.path.poses)):

            path_pose = self.path.poses[i]
            path_pose_in_robot_frame = self.utils.transform_pose(path_pose, 'base_footprint', self.get_logger())
            coords = path_pose_in_robot_frame.pose.position
            if coords.x > 0:
                if coords.x >= ERROR_ADMITIDO and coords.y >= ERROR_ADMITIDO:
                    if coords.x < next_point.pose.position.x and coords.y < next_point.pose.position.y:
                        next_point = path_pose_in_robot_frame
                        j = i

        subgoal = self.path.poses[0]
        if j != sys.maxsize:
            subgoal = self.path.poses[j]

        self.get_logger().info('%d' % self.path.poses.index(subgoal))
        self.get_logger().info('%.2f' % subgoal.pose.position.x)
        self.get_logger().info('%.2f' % subgoal.pose.position.y)
        return subgoal"""
        if not self.path_received or len(self.path.poses) == 0:
            return None

        best_pose = None
        min_dist = float('inf')

        for path_pose in self.path.poses:
            try:
                # Transformamos cada punto al marco del robot
                transformed_pose = self.utils.transform_pose(path_pose, 'base_footprint', self.get_logger())
                if transformed_pose is not None:
                    coords = transformed_pose.pose.position
                    
                    # Solo nos interesan puntos delante del robot (x > 0)
                    if coords.x > 0.0:
                        # Calculamos la distancia real al punto
                        dist = math.sqrt(coords.x**2 + coords.y**2)
                        
                        # Nos quedamos con el punto más cercano que supere el error admitido
                        if dist > ERROR_ADMITIDO and dist < min_dist:
                            min_dist = dist
                            best_pose = transformed_pose
            except Exception:
                continue

        # Si no encontró ningún punto válido delante, va al final de la ruta
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

        """"
        for i in range(len(self.laser.ranges)):
            if self.laser.ranges[i] > 0.012:
                return True
        return False"""
        valid_ranges = [r for r in self.laser.ranges if r > self.laser.range_min and r < self.laser.range_max]
        
        # 2. Si no hay datos válidos, asumimos que no hay peligro inminente
        if not valid_ranges:
            return False
            
        # 3. Comprobamos la lectura más cercana
        distancia_minima = min(valid_ranges)
        distancia_seguridad = 0.4 # Metros (40 cm)
        
        if distancia_minima > distancia_seguridad:
            return True # ¡Peligro, hay un obstáculo cerca!
            
        return False # Camino libre


    def collision_avoidance(self):
        """
        Exercise 2: Try to find an alternative command to avoid collision
        TODO: try to find an alternative command to avoid the collision
        Here you must try to implement one of the reactive methods
        seen in T4: bug algorithm, potential fields, velocity obstacles,
        Dynamic Window Approach, others...
        Feel free to add the new variables and methods that you may need
        Returns (lin_vel, ang_vel)
        """

        m_atrac = 0.6
        rsoi = 1
        do = min(self.laser.ranges)

        indice_min = self.laser.ranges.index(do)

        theta = self.laser.angle_min + (indice_min * self.laser.angle_increment)

        goal = self.get_sub_goal()
        punto = [goal.pose.position.x, goal.pose.position.y]
        resultado_atraccion = [x / math.sqrt(goal.pose.position.x**2 + goal.pose.position.y**2) for x in punto]
        resultado_atraccion = [m_atrac * x for x in resultado_atraccion]
        F_atr_x = resultado_atraccion[0]
        F_atr_y = resultado_atraccion[1]

        self.get_logger().info('F_atr_x: %.3f' % F_atr_x)
        self.get_logger().info('F_atr_y: %.3f' % F_atr_y)

        if 0.0 < do <= rsoi:
            magnitud_fuerza = (rsoi - do) / do
            F_rep_x = -magnitud_fuerza * math.cos(theta)
            F_rep_y = -magnitud_fuerza * math.sin(theta)
        else:
            F_rep_x = 0.0
            F_rep_y = 0.0

        self.get_logger().info('F_rep_x: %.3f' % F_rep_x)
        self.get_logger().info('F_rep_y: %.3f' % F_rep_y)
        F_x = F_atr_x + F_rep_x
        F_y = F_atr_y + F_rep_y
        self.get_logger().info('F_x: %.3f' % F_x)
        self.get_logger().info('F_y: %.3f' % F_y)
            
        ang_vel = math.atan2(F_y, F_x)
        ang_vel = self.constrain_vel(ang_vel, -self.max_ang_vel, self.max_ang_vel)
        lin_vel = self.max_lin_vel * math.sqrt(F_x**2 + F_y**2)
        lin_vel = self.constrain_vel(lin_vel, 0.0, self.max_lin_vel)
        self.get_logger().info('ang_vel : %.3f' % ang_vel)
        self.get_logger().info('lin_vel: %.3f' % lin_vel)
        return lin_vel, ang_vel


    def constrain_vel(self, input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel

        
    def publish(self, lin_vel, ang_vel):
        """Publish velocity commands to the robot"""
        move_cmd = TwistStamped()
        move_cmd.twist.linear.x = lin_vel
        move_cmd.twist.angular.z = ang_vel
        self.cmd_vel.publish(move_cmd)


    def laser_callback(self, data):
        """Callback to receive and store laser scan messages"""
        self.laser = data
        self.laser_received = True
        

    def path_callback(self, path):
        """Callback to receive and store path messages"""
        self.path = path
        self.path_received = True
        

    def shutdown_callback(self):
        """Shutdown callback to stop the robot"""
        self.get_logger().info("Stop TurtleBot")
        # A default Twist has linear.x of 0 and angular.z of 0, so it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
 

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
