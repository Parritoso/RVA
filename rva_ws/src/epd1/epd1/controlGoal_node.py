#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs  # This registers geometry_msgs types with tf2
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PointStamped

BURGER_MAX_LIN_VEL = 0.50
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


class Turtlebot(Node):
    def __init__(self):
        super().__init__('robotcontrol')
        
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Set up TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.goalx = 0.0
        self.goaly = 0.0

        # Timer for control loop (10 HZ)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
     
    def timer_callback(self):
        self.command(self.goalx, self.goaly)
        
    def command(self, gx, gy):
        
        goal = PointStamped()
        
        goal.header.frame_id = 'odom'
        goal.header.stamp = rclpy.time.Time()  # Use 0 to get latest available transform
        
        goal.point.x = gx
        goal.point.y = gy
        goal.point.z = 0.0
        
        try:
            base_goal = self.tf_buffer.transform(goal, 'base_footprint', timeout=rclpy.duration.Duration(seconds=1.0))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return
        
        self.get_logger().info('Pos: %.2f, %.2f' % (base_goal.point.x, base_goal.point.y))

        #------------------------------------------------------------------
        # TODO: Put the control law here.
        # Compute linear and angular vels based on the distance to the goal
        # and the angle between the robot heading and the goal.
        theta = math.atan2(base_goal.point.y, base_goal.point.x)
        linear = 0.0
        angular = 0.0
        self.get_logger().info('Theta: %.2f' % (theta))
        if abs(base_goal.point.x) > 0.01 or abs(base_goal.point.y) > 0.01:
            if abs(theta) > 0.01:
                angular = BURGER_MAX_ANG_VEL * theta
            else :
                angular = 0.0
                linear = BURGER_MAX_LIN_VEL * math.sqrt(base_goal.point.x ** 2 + base_goal.point.y ** 2)  
        else:
            self.shutdown() 
        
        #------------------------------------------------------------------
        self.publish(linear, angular)

    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = TwistStamped()
        move_cmd.header.stamp = self.get_clock().now().to_msg()

        # Copy the forward velocity
        move_cmd.twist.linear.x = self.constrain_vel(lin_vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        # Copy the angular velocity
        move_cmd.twist.angular.z = self.constrain_vel(ang_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

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
        
    def shutdown(self):
        # stop turtlebot
        self.get_logger().info("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0. So it'll stop TurtleBot
        self.cmd_vel.publish(TwistStamped())
 
def main():
    rclpy.init()
    try:
        # tell user how to stop TurtleBot
        robot = Turtlebot()
        robot.get_logger().info("To stop TurtleBot CTRL + C")

        goalx = float(sys.argv[1])
        goaly = float(sys.argv[2])
        
        robot.get_logger().info("Goal set to x: %.2f y: %.2f" % (goalx, goaly))
        
        robot.goalx = goalx
        robot.goaly = goaly

        rclpy.spin(robot)

    except KeyboardInterrupt:
        robot.get_logger().info("robotcontrol node terminated.")
    except IndexError:
        print("Usage: ros2 run epd1 controlGoal.py <goalx> <goaly>")
    finally:
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()