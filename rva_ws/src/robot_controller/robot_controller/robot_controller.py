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

# class TurtlebotController(Node):
    
#     def __init__(self):
#         super().__init__('robot_controller')
        
#         # Declare parameters
#         self.declare_parameter('robot_vel_topic', 'cmd_vel')
#         self.declare_parameter('robot_scan_topic', 'scan')
#         self.declare_parameter('max_lin_vel', 0.2)
#         self.declare_parameter('max_ang_vel', 0.4)
#         self.declare_parameter('control_rate', 10)
        
#         # Get parameters
#         robot_vel_topic = self.get_parameter('robot_vel_topic').value
#         robot_scan_topic = self.get_parameter('robot_scan_topic').value
#         self.max_lin_vel = self.get_parameter('max_lin_vel').value
#         self.max_ang_vel = self.get_parameter('max_ang_vel').value
#         control_rate = self.get_parameter('control_rate').value
        
#         # Store the received path here
#         self.path = Path()
#         self.path_received = False
#         self.laser = LaserScan()
#         self.laser_received = False
        
#         self.last_path_idx = 0

#         # Declare the velocity command publisher
#         self.cmd_vel = self.create_publisher(TwistStamped, robot_vel_topic, 10)
        
#         # Create tf2 buffer and listener
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         self.utils = Utils(self.tf_buffer)
        
#         # Subscription to the scan topic [sensor_msgs/LaserScan]
#         self.create_subscription(LaserScan, robot_scan_topic, self.laser_callback, 10)
        
#         # Subscription to the path topic [nav_msgs/Path]
#         self.create_subscription(Path, 'path', self.path_callback, 10)
        
#         # Subscription to odometry
#         self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
#         # Create a timer for the control loop
#         timer_period = 1.0 / control_rate  # seconds
#         self.timer = self.create_timer(timer_period, self.control_loop)
        
     
#     def odom_callback(self, msg):
#         """Callback to receive and store odometry messages"""
#         self.utils.set_odom(msg)

#     def control_loop(self):
#         """Main control loop executed at fixed rate"""
#         end = self.command()
#         if end:
#             self.get_logger().info("Goal reached, stopping")
#             self.destroy_timer(self.timer)

#     # def transform_to_robot_frame(self, map_pose):
#     #     """
#     #     Transforma manualmente una pose desde el frame 'map' al frame local 'base_footprint'
#     #     usando las transformaciones de TF robustas al tiempo actual (Time 0).
#     #     Retorna: (gx_local, gy_local, success)
#     #     """
#     #     try:
#     #         # Buscamos la posición y orientación actual del robot en el mapa
#     #         map_pose.header.stamp = rclpy.time.Time()
#     #         trans = self.tf_buffer.transform(
#     #             map_pose, 
#     #             'base_footprint', 
#     #             timeout=rclpy.duration.Duration(seconds=0.1)
#     #         )

#     #         self.get_logger().info(f"trans.header.frame_id: {trans.header.frame_id}")
            
#     #         # Posición del robot en el mapa
#     #         rx = trans.transform.translation.x
#     #         ry = trans.transform.translation.y
            
#     #         # Orientación del robot (Conversión Cuaternión -> Ángulo Yaw)
#     #         q = trans.transform.rotation
#     #         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     #         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     #         yaw = math.atan2(siny_cosp, cosy_cosp)
            
#     #         # Coordenadas absolutas del objetivo
#     #         mx = map_pose.pose.position.x
#     #         my = map_pose.pose.position.y
            
#     #         # Transmación geométrica (Traslación + Rotación inversa al frame del robot)
#     #         dx = mx - rx
#     #         dy = my - ry
            
#     #         gx_local = dx * math.cos(yaw) + dy * math.sin(yaw)
#     #         gy_local = -dx * math.sin(yaw) + dy * math.cos(yaw)
            
#     #         return gx_local, gy_local, True
            
#     #     except TransformException as ex:
#     #         self.get_logger().warning(f"Error en TF transform_to_robot_frame: {ex}")
#     #         return 0.0, 0.0, False

#     def command(self):
#         """
#         Command the robot to follow the path
#         Returns True if goal reached, False otherwise
#         """
#         # Check if the final goal has been reached
#         # TODO: Exercise 1:implement goal reached check
#         if self.goal_reached():
#             self.get_logger().info("GOAL REACHED!!! Stopping!")
#             self.publish(0.0, 0.0)
#             return True
        
#         # Determine the local path point to be reached
#         # TODO: Exercise 1: fill the method get_sub_goal
#         current_goal = self.get_sub_goal() 
#         self.get_logger().info(f"current_goal x: {current_goal.pose.position.x}, y: {current_goal.pose.position.y}")

#         if current_goal.header.frame_id == '':
#             self.publish(0.0, 0.0)
#             return False

#         self.get_logger().info(f"current_goal.header.frame_id: {current_goal.header.frame_id}")
#         current_goal.header.stamp = rclpy.time.Time()
#         goal_in_robot = self.tf_buffer.transform(
#                  current_goal, 
#                  'base_footprint', 
#                  timeout=rclpy.duration.Duration(seconds=0.1)
#              )
#         self.get_logger().info(f"goal_in_robot.header.frame_id: {goal_in_robot.header.frame_id}")
#         self.get_logger().info(f"goal_in_robot x: {goal_in_robot.pose.position.x}, y: {goal_in_robot.pose.position.y}")

#         gx = goal_in_robot.pose.position.x
#         gy = goal_in_robot.pose.position.y

#         alpha = math.atan2(gy, gx)
#         dist = math.sqrt(gx**2 + gy**2)

#         if abs(alpha) > 2.8:
#             linear = 0.0
#             angular = self.max_ang_vel if alpha > 0 else -self.max_ang_vel
#             self.get_logger().info("Objetivo a la espalda. Pivotando de forma estable...")
#         elif abs(alpha) > 0.4:
#             linear = 0.0
#             angular = 2.5 * alpha
#         else:
#             # Velocidad lineal adaptativa (frena suavemente si hay curvas cerradas)
#             linear = self.max_lin_vel * math.cos(alpha)
            
#             # w = v * (2 * y / d^2) -> Consigue un arco geométrico perfecto hacia el lookahead
#             if dist > 0.05:
#                 angular = linear * (2.0 * gy / (dist ** 2))
#             else:
#                 angular = 0.0

#         # K_z = 2.5  # Ganancia angular
#         # K_x = 0.6  # Ganancia lineal

#         # angular = K_z * alpha

#         # if abs(alpha) > 0.2:
#         #     linear = 0.0
#         # else:
#         #     linear = K_x * math.sqrt(gx**2 + gy**2) * math.cos(alpha)
            
#         # Check the maximum speed values allowed
#         angular = self.constrain_vel(angular, -self.max_ang_vel, self.max_ang_vel)
#         linear = self.constrain_vel(linear, 0.0, self.max_lin_vel)
            
#         # If the computed commands does not provoke a collision,
#         # send the commands to the robot
#         # TODO: fill the check_collision function (copy from EPD2)
#         if not self.check_collision(linear, angular):
#             self.publish(linear, angular)
#             return False

#         # If a possible collision is detected,
#         # try to find an alternative command to avoid the collision
#         # TODO: Exercise 2: fill the collision_avoidance function
#         self.get_logger().warn("Potential collision detected! Activating reactive potential fields avoidance...")
#         linear, angular = self.collision_avoidance() 
#         angular = self.constrain_vel(angular, -self.max_ang_vel, self.max_ang_vel)
#         linear = self.constrain_vel(linear, 0.0, self.max_lin_vel)
#         self.publish(linear, angular)
#         return False


#     def goal_reached(self):
#         """
#         Exercise 1.2: Check if the final goal has been reached
#         TODO: use the last point of the path to check if the robot
#         has reached the final goal (the robot is in a close position).
#         Returns True if the FINAL goal was reached, False otherwise
#         """
#         if not self.path_received or len(self.path.poses) == 0:
#             return False
            
#         # Evaluamos respecto al último punto de la trayectoria enviada por el planificador
#         final_goal = self.path.poses[-1]
#         final_goal.header.stamp = rclpy.time.Time()
#         goal_in_robot = self.tf_buffer.transform(
#                  final_goal, 
#                  'base_footprint', 
#                  timeout=rclpy.duration.Duration(seconds=0.1)
#              )
        
#         if goal_in_robot.header.frame_id == 'base_footprint':
#             dist = math.sqrt(goal_in_robot.pose.position.x**2 + goal_in_robot.pose.position.y**2)
#             # El archivo evaluation.py exige una tolerancia exacta de 0.15 metros
#             if dist <= 0.1:
#                 return True

#         return False


#     def get_sub_goal(self):
#         """
#         Exercise 1.1: Get the next sub-goal from the path
#         TODO: use self.path.poses to find the subgoal to be reached
#         You could transform the path points to the robot reference
#         to find the closest point:
#         path_pose = self.path.poses[index]
#         path_pose_in_robot_frame = self.utils.transform_pose(
#             path_pose, 'base_footprint', self.get_logger())
#         """
#         subgoal = PoseStamped()
#         if not self.path_received or len(self.path.poses) == 0:
#             return subgoal

#         lookahead_dist = 0.35
#         closest_idx = self.last_path_idx

#         for i in range(self.last_path_idx, len(self.path.poses)):
#             pose = self.path.poses[i]
#             pose.header.stamp = rclpy.time.Time()
#             pose_in_robot = pose_in_robot = self.tf_buffer.transform(
#                   pose, 
#                   'base_footprint', 
#                   timeout=rclpy.duration.Duration(seconds=0.1)
#               )
#             if pose_in_robot.header.frame_id == 'base_footprint':
#                 dist = math.sqrt(pose_in_robot.pose.position.x**2 + pose_in_robot.pose.position.y**2)
#                 chosen_idx = i
#                 # En el momento en que un punto supera el umbral de mirada, lo fijamos
#                 if dist >= lookahead_dist:
#                     break
                    
#         # Bloqueamos el índice para que en el siguiente tick de control sea imposible volver atrás
#         self.last_path_idx = chosen_idx
#         return self.path.poses[chosen_idx]
#         # min_dist = float('inf')
#         # for i in range(self.last_path_idx, len(self.path.poses)):
#         #     pose = self.path.poses[i]
#         #     pose.header.stamp = rclpy.time.Time()
#         #     pose_in_robot = self.tf_buffer.transform(
#         #          pose, 
#         #          'base_footprint', 
#         #          timeout=rclpy.duration.Duration(seconds=0.1)
#         #      )
#         #     if pose_in_robot.header.frame_id == 'base_footprint':
#         #         dist = math.sqrt(pose_in_robot.pose.position.x**2 + pose_in_robot.pose.position.y**2)
#         #         if dist < min_dist:
#         #             min_dist = dist
#         #             closest_idx = i

#         # self.last_path_idx = closest_idx
                    
#         # # 2. Desde esa posición del camino hacia adelante, buscamos el primer punto que supere la distancia lookahead
#         # subgoal = self.path.poses[closest_idx]
#         # for i in range(closest_idx, len(self.path.poses)):
#         #     pose = self.path.poses[i]
#         #     pose.header.stamp = rclpy.time.Time()
#         #     pose_in_robot = self.tf_buffer.transform(
#         #          pose, 
#         #          'base_footprint', 
#         #          timeout=rclpy.duration.Duration(seconds=0.1)
#         #      )
#         #     if pose_in_robot.header.frame_id == 'base_footprint':
#         #         dist = math.sqrt(pose_in_robot.pose.position.x**2 + pose_in_robot.pose.position.y**2)
#         #         if dist >= lookahead_dist:
#         #             subgoal = pose
#         #             break
                    
#         # return subgoal


#     def check_collision(self, linear, angular):
#         """
#         Copy from EPD2: Check for possible collisions
#         TODO: use self.laser to check possible collisions
#         Optionally, you can also use the velocity commands
#         Returns True if possible collision, False otherwise
#         """
#         if not self.laser_received or len(self.laser.ranges) == 0:
#             return False
        
#         safety_distance = 0.32

#         angle = self.laser.angle_min
#         for r in self.laser.ranges:
#             # Descartar lecturas no válidas (NaN o Inf) e inferiores al rango mínimo del sensor
#             if math.isnan(r) or math.isinf(r) or r < self.laser.range_min:
#                 angle += self.laser.angle_increment
#                 continue
                
#             # Evaluamos un sector angular frontal de -45 a +45 grados (-pi/4 a pi/4)
#             if -math.pi / 3.0 <= angle <= math.pi / 3.0:
#                 if r <= safety_distance:
#                     return True
                    
#             angle += self.laser.angle_increment
        
#         return False


#     def collision_avoidance(self):
#         """
#         Exercise 2: Try to find an alternative command to avoid collision
#         TODO: try to find an alternative command to avoid the collision
#         Here you must try to implement one of the reactive methods
#         seen in T4: bug algorithm, potential fields, velocity obstacles,
#         Dynamic Window Approach, others...
#         Feel free to add the new variables and methods that you may need
#         Returns (lin_vel, ang_vel)
#         """
#         ang_vel = 0.0
#         lin_vel = 0.0
#         if not self.path_received or not self.laser_received:
#             return lin_vel, ang_vel

#         current_goal = self.get_sub_goal()
#         current_goal.header.stamp = rclpy.time.Time()
#         goal_robot = self.tf_buffer.transform(
#                  current_goal, 
#                  'base_footprint', 
#                  timeout=rclpy.duration.Duration(seconds=0.1)
#              )
        
#         if goal_robot.header.frame_id != 'base_footprint':
#             return lin_vel, ang_vel
        
#         gx = goal_robot.pose.position.x
#         gy = goal_robot.pose.position.y
#         dist_goal = math.sqrt(gx**2 + gy**2)

#         K_att = 1.0
#         if dist_goal > 0.05:
#             F_att_x = K_att * (gx / dist_goal)
#             F_att_y = K_att * (gy / dist_goal)
#         else:
#             F_att_x = 0.0
#             F_att_y = 0.0

#         F_rep_x = 0.0
#         F_rep_y = 0.0
#         K_rep = 0.015  # Ganancia repulsiva escalada para la fuerza atractiva unitaria
#         rho_0 = 0.45   # Umbral de influencia del campo repulsivo (metros)
#         count_rep = 0
        
#         angle = self.laser.angle_min
#         for r in self.laser.ranges:
#             if math.isnan(r) or math.isinf(r) or r < self.laser.range_min:
#                 angle += self.laser.angle_increment
#                 continue
                
#             if r < rho_0:
#                 # Usamos un suelo mínimo de 0.13m para mitigar asíntotas infinitas cerca del límite crítico (0.12m)
#                 r_val = max(r, 0.13) 
#                 f_rep_mag = K_rep * (1.0 / r_val - 1.0 / rho_0) / (r_val * r_val)
                
#                 # Sumamos las componentes vectoriales en dirección opuesta al obstáculo
#                 F_rep_x += -f_rep_mag * math.cos(angle)
#                 F_rep_y += -f_rep_mag * math.sin(angle)
#                 count_rep += 1
                
#             angle += self.laser.angle_increment
            
#         # Promediamos las fuerzas según las lecturas que han contribuido para independizarnos de la resolución del LiDAR
#         if count_rep > 0:
#             F_rep_x /= count_rep
#             F_rep_y /= count_rep
            
#         # 3. Suma Vectorial Final
#         F_total_x = F_att_x + F_rep_x
#         F_total_y = F_att_y + F_rep_y
        
#         # 4. Extracción de velocidad angular y lineal basándonos en la dirección resultante
#         target_heading = math.atan2(F_total_y, F_total_x)
        
#         ang_vel = 2.5 * target_heading
        
#         # Si la dirección de escape nos exige un cambio brusco de orientación, frenamos el avance
#         if abs(target_heading) > 0.5:
#             lin_vel = 0.02 
#         else:
#             lin_vel = 0.12 * math.cos(target_heading)
            
#         return lin_vel, ang_vel


#     def constrain_vel(self, input_vel, low_bound, high_bound):
#         if input_vel < low_bound:
#             input_vel = low_bound
#         elif input_vel > high_bound:
#             input_vel = high_bound
#         else:
#             input_vel = input_vel

#         return input_vel

        
#     def publish(self, lin_vel, ang_vel):
#         """Publish velocity commands to the robot"""
#         move_cmd = TwistStamped()
#         move_cmd.header.stamp = self.get_clock().now().to_msg()
#         move_cmd.twist.linear.x = lin_vel
#         move_cmd.twist.angular.z = ang_vel
#         self.cmd_vel.publish(move_cmd)


#     def laser_callback(self, data):
#         """Callback to receive and store laser scan messages"""
#         self.laser = data
#         self.laser_received = True
        

#     def path_callback(self, path):
#         """Callback to receive and store path messages"""
#         self.path = path
#         self.path_received = True
        

#     def shutdown_callback(self):
#         """Shutdown callback to stop the robot"""
#         self.get_logger().info("Stop TurtleBot")
#         # A default Twist has linear.x of 0 and angular.z of 0, so it'll stop TurtleBot
#         self.cmd_vel.publish(TwistStamped())
 

# def main(args=None):
#     rclpy.init(args=args)
    
#     # Create and run the controller
#     robot_controller = TurtlebotController()
    
#     robot_controller.get_logger().info("TurtleBot controller started")
#     robot_controller.get_logger().info("To stop TurtleBot press CTRL+C")
    
#     try:
#         rclpy.spin(robot_controller)
#     except KeyboardInterrupt:
#         robot_controller.get_logger().info("TurtleBot controller stopped by user")
#     finally:
#         robot_controller.shutdown_callback()
#         robot_controller.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

"""========================================================================================"""

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

        if self.path_received:
            # Check if the final goal has been reached
            # TODO: Exercise 1:implement goal reached check
            if self.goal_reached():
                self.get_logger().info("GOAL REACHED!!! Stopping!")
                self.publish(0.0, 0.0)
                return True
            
            # Determine the local path point to be reached
            # TODO: Exercise 1: fill the method get_sub_goal
            goal_pose = self.get_sub_goal()
            if goal_pose is None:
                return False
                
            current_goal = goal_pose.pose.position
            self.get_logger().info('current_goal: %.03f, %.03f' % (current_goal.x, current_goal.y))

            # TODO: use current_goal 
            # Put your control law here (copy from EPD1)
            distancia = math.sqrt(current_goal.x ** 2 + current_goal.y ** 2)
            theta = math.atan2(current_goal.y, current_goal.x)
            
            self.get_logger().info('Theta: %.4f' % (theta))
            
            # Ley de control simultánea para evitar giros bruscos a tirones
            angular = theta 
            
            if abs(theta) > 0.5: # Si el ángulo es muy cerrado, avanza despacio y prioriza el giro
                linear = 0.05
            else:
                linear = distancia 
                
            # Check the maximum speed values allowed
            angular = self.constrain_vel(angular, -self.max_ang_vel, self.max_ang_vel)
            linear = self.constrain_vel(linear, 0.0, self.max_lin_vel)
                
            # If the computed commands does not provoke a collision,
            # send the commands to the robot
            # TODO: fill the check_collision function (copy from EPD2)
            if not self.check_collision(linear, angular):
                self.publish(linear, angular)
                return False

            # If a possible collision is detected,
            # try to find an alternative command to avoid the collision
            # TODO: Exercise 2: fill the collision_avoidance function
            self.get_logger().info('Colision detectada. Evadiendo...')
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

        # PASO 1: Buscar qué punto de la ruta es el más cercano al robot AHORA MISMO
        # (Esto evita que el robot intente volver al punto de inicio)
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

        # PASO 2: A partir de ese punto más cercano, buscar hacia adelante el primero 
        # que esté un poco más lejos que nuestro ERROR_ADMITIDO (lookahead)
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

        # Si no encontramos ninguno (ej. estamos llegando al final), apuntamos a la meta
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
        
        if distancia_minima < distancia_seguridad:
            return True # ¡Peligro, hay un obstáculo cerca!
            
        return False # Camino libre


    def collision_avoidance(self):
        goal = self.get_sub_goal()
        if goal is None:
            return 0.0, 0.0

        front = self.get_valid_scan_ranges(-math.pi / 4.0, math.pi / 4.0)
        left = self.get_valid_scan_ranges(0.0, math.pi / 2.0)
        right = self.get_valid_scan_ranges(-math.pi / 2.0, 0.0)

        front_min = min(front) if front else float('inf')
        left_clearance = min(left) if left else self.laser.range_max
        right_clearance = min(right) if right else self.laser.range_max

        if abs(left_clearance - right_clearance) > 0.05:
            turn_sign = 1.0 if left_clearance > right_clearance else -1.0
        else:
            turn_sign = 1.0 if goal.pose.position.y >= 0.0 else -1.0

        lin_vel = 0.0 if front_min < 0.24 else 0.04
        ang_vel = turn_sign * self.max_ang_vel

        return lin_vel, ang_vel

        """
        Exercise 2: Try to find an alternative command to avoid collision
        TODO: try to find an alternative command to avoid the collision
        Here you must try to implement one of the reactive methods
        seen in T4: bug algorithm, potential fields, velocity obstacles,
        Dynamic Window Approach, others...
        Feel free to add the new variables and methods that you may need
        Returns (lin_vel, ang_vel)
        """

        # ¡AQUÍ ESTÁ LA CLAVE! 
        # Aumentamos brutalmente la atracción y reducimos el miedo a las paredes.
        m_atrac = 2.0  # Atracción FUERTE hacia la línea verde
        m_rep = 0.2    # Repulsión SUAVE (solo para evitar el roce)
        rsoi = 0.4     # Solo le asustan paredes a menos de 40cm.

        valid_ranges = [r for r in self.laser.ranges if r > self.laser.range_min and r < self.laser.range_max]

        if not valid_ranges:
            do = float('inf')
            theta = 0.0
        else:
            do = min(valid_ranges)
            indice_min = self.laser.ranges.index(do)
            theta = self.laser.angle_min + (indice_min * self.laser.angle_increment)
            
        goal = self.get_sub_goal()
        
        # Por seguridad, si no hay meta, nos paramos
        if goal is None:
            return 0.0, 0.0
            
        coords = goal.pose.position
        
        distancia_goal = math.sqrt(coords.x**2 + coords.y**2)
        
        # 2. FUERZA DE ATRACCIÓN (Hacia la línea verde)
        if distancia_goal > 0.0:
            F_atr_x = m_atrac * (coords.x / distancia_goal)
            F_atr_y = m_atrac * (coords.y / distancia_goal)
        else:
            F_atr_x = 0.0
            F_atr_y = 0.0

        # 3. FUERZA DE REPULSIÓN (Alejarse de la pared)
        if 0.0 < do <= rsoi:
            # Hacemos que la repulsión no sea tan exagerada
            magnitud_fuerza = m_rep * ((rsoi - do) / do)
            F_rep_x = -magnitud_fuerza * math.cos(theta)
            F_rep_y = -magnitud_fuerza * math.sin(theta)
        else:
            F_rep_x = 0.0
            F_rep_y = 0.0

        # 4. SUMA DE FUERZAS
        F_x = F_atr_x + F_rep_x
        F_y = F_atr_y + F_rep_y
            
        # 5. CÁLCULO DE VELOCIDAD FINAL 
        f_theta = math.atan2(F_y, F_x) 
        f_mag = math.sqrt(F_x**2 + F_y**2) 
        
        ang_vel = f_theta
        
        # Freno de emergencia si tiene que dar un giro brusco (esquinas)
        if abs(f_theta) > 0.5:
            lin_vel = 0.0
        else:
            # Si el camino está más o menos recto, acelera a tope
            lin_vel = self.max_lin_vel * f_mag
        
        ang_vel = self.constrain_vel(ang_vel, -self.max_ang_vel, self.max_ang_vel)
        lin_vel = self.constrain_vel(lin_vel, 0.0, self.max_lin_vel)
        
        return lin_vel, ang_vel
        
        """
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
    """


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
