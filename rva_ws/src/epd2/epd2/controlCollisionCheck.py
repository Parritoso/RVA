#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs  # This registers geometry_msgs types with tf2
from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan

# x= r*cos(angulo) y = r*sin(angulo) puntos en polares


class TurtlebotControlCollisionCheck(Node):
    def __init__(self):
        super().__init__('controlCollisionCheck')
        
        # We will store here the goal point
        self.goal = None
        self.light_speed = 29979245
        
        
        # -------------------------------------------------------------
        # TODO: declare and read the maximum linear and angular velocities
        #  as ROS 2 node parameters!!!!
        self.declare_parameter('max_lin',0.3)
        self.declare_parameter('max_ang',0.4)
        self.max_lin_vel = self.get_parameter('max_lin').value # m/s
        self.max_ang_vel = self.get_parameter('max_ang').value # rad/s 
        self.get_logger().info("Maximum Turtlebot linear vel set: %.2f m/s" % self.max_lin_vel)
        self.get_logger().info("Maximum Turtlebot angular vel set: %.2f rad/s" % self.max_ang_vel) 
        # -------------------------------------------------------------
        
        # Set up TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create a publisher to command the robot
        self.cmd_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Create a subscription to the scan topic [sensor_msgs/LaserScan]
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.scan_sub  # prevent unused variable warning
        
        # Create a subscription to the goal topic [geometry_msgs/PoseStamped]
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10)
        self.goal_sub # prevent unused variable warning
        
        
        # Timer for control loop (10 HZ)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
   
   
    # Each time a new laser arrives, we store it in self.laser  
    def scan_callback(self,data):
        self.laser = data
        #self.get_logger().info("Laser received " + str(data.ranges))
        #self.get_logger().info("angle_increment" + str(data.angle_increment))
        #self.get_logger().info("range_max " + str(data.range_min))
        #rospy.loginfo("Laser received " + str(len(data.ranges)))
        
        
    # We receive a new goal
    def goal_callback(self, goal):
        self.get_logger().info("Goal received! x: %.2f, y:%.2f" % (goal.pose.position.x, goal.pose.position.y))
        self.goal = goal
        
    
    def timer_callback(self):
        self.command()
   
        
    def command(self):
        
        if self.goal is None:
            self.publish(0.0, 0.0)
            return
        
        g = self.goal
        g.header.stamp = rclpy.time.Time()  # Use 0 to get latest available transform
        
        # transform the goal received (usually in odom frame) to the robot frame
        try:
            base_goal = self.tf_buffer.transform(g, 'base_footprint', timeout=rclpy.duration.Duration(seconds=1.0))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return
            
        # -------------------------------------------------------------
        x = base_goal.pose.position.x
        y = base_goal.pose.position.y
        # TODO: put the control law here (copy from EPD1)
        angular = 0.0  
        linear = 0.0
        theta = math.atan2(y, x)
        self.get_logger().info('Theta: %.2f' % (theta))
        if abs(x) > 0.01 or abs(y) > 0.01:
            if abs(theta) > 0.01:
                angular = self.max_ang_vel * theta
            else :
                angular = 0.0
                linear =  self.max_lin_vel * math.sqrt(x ** 2 + y ** 2)  
        else:
            self.shutdown() 
        
        # TODO: fill the checkCollision function
        if(self.checkCollision()):
            self.get_logger().info('POSSIBLE COLLISION!!!! STOPPING ROBOT!!!')
            linear = 0.0
            angular = 0.0
        #--------------------------------------------------------------
        self.publish(linear, angular)



    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = TwistStamped()
        move_cmd.header.stamp = self.get_clock().now().to_msg()

        # forward velocity
        move_cmd.twist.linear.x = self.constrain_vel(lin_vel, -self.max_lin_vel, self.max_lin_vel)
        # angular velocity
        move_cmd.twist.angular.z = self.constrain_vel(ang_vel, -self.max_ang_vel, self.max_ang_vel)
        if(lin_vel != 0.0 or ang_vel != 0.0):    
            self.get_logger().info('Publishing linear: "%.2f angular: "%.2f"' % (move_cmd.twist.linear.x, move_cmd.twist.angular.z))
        self.cmd_vel.publish(move_cmd)



    def constrain_vel(self,input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel
        
        
        
    def checkCollision(self):
        # -------------------------------------------------------------
        # TODO: use self.laser to check possible collisions
        # return True if possible collision, False otherwise
        for i in range(len(self.laser.range)):
            if range[i] < self.laser.range_min:
                return True
        return False
        # -------------------------------------------------------------
    
        
        
    def shutdown(self):
        # stop turtlebot
        self.get_logger().info("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0. So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
 
 
def main():
    rclpy.init()
    try:
        robot = TurtlebotControlCollisionCheck()
        robot.get_logger().info("To stop TurtleBot CTRL + C")
        rclpy.spin(robot)

    except KeyboardInterrupt:
        robot.get_logger().info("controlCollisionCheck node terminated.")
    except IndexError:
        print("Usage: ros2 run epd2 controlCollisionCheck")
    finally:
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()