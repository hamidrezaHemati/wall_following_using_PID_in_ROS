#!/usr/bin/python3


from math import dist
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Wall_follwer():

    def __init__(self):
        rospy.init_node("wall_follower", anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom = rospy.Subscriber("/odom", Odometry, self.update_pose, queue_size=10)

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.linear_Speed = 0.5

        self.kp = 0.5
        self.ki = 0
        self.kd = 15

        self.distance_margin = 1.5

        self.dt = 0.05
        self.rate = rospy.Rate(1/self.dt)
    

    def update_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.quaternion_to_euler(msg)
    

    def quaternion_to_euler(self, msg):
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw


    def distance_from_left_wall(self):
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        left_side = laser_data.ranges[10:180]
        return min(left_side)

    
    def follow_wall(self):
        rospy.sleep(2)
        speed = Twist()
        speed.linear.x = self.linear_Speed
        speed.angular.z = 0
        left_dist = self.distance_from_left_wall()

        error = 0
        integral = 0
        derivetive = 0
        privous_error = 0

        while not rospy.is_shutdown():

            error = left_dist - self.distance_margin

            P = self.kp * error
            I = self.ki * integral
            D = self.kd * derivetive

            speed.angular.z = P+I+D
            speed.linear.x = self.linear_Speed

            self.cmd_publisher.publish(speed)


            ## update controller
            integral += error * self.dt
            derivetive = error - privous_error
            privous_error = error

            left_dist = self.distance_from_left_wall()

            self.rate.sleep()


    def on_shutdown(self):
        rospy.sleep(1)



if __name__ == "__main__":
    controller = Wall_follwer()
    controller.follow_wall()