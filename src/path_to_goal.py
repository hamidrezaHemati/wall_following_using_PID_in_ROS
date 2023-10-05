#!/usr/bin/python3


from math import atan2, degrees
import rospy
import tf

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Wall_follwer():

    def __init__(self):
        rospy.init_node("wall_follower", anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom = rospy.Subscriber("/odom", Odometry, self.update_pose, queue_size=10)

        self.goal = Point ()
        self.goal.x = 3
        self.goal.y = -1

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.linear_Speed = 0.15

        self.kp = 0.5
        self.ki = 0
        self.kd = 10

        self.distance_margin = 0.4

        self.tuner = 0.1

        self.is_following_wall = False

        self.is_obstacle_in_front = False
        self.is_obstacle_in_upper_left = False
        self.is_obstacle_in_downer_left = False

        self.is_obstacle = [False, False, False] ##0: front, 1: upper left, 2: downer left


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
        left_side = laser_data.ranges[5:135]
        return min(left_side)

    def distance_from_upper_left_wall(self):
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        left_side = laser_data.ranges[5:90]
        return min(left_side)

    def distance_from_downer_left_wall(self):
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        left_side = laser_data.ranges[90:135]
        return min(left_side)

    
    def distance_from_front_wall(self):
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        l_front = laser_data.ranges[0:5]
        r_front = laser_data.ranges[355:360]
        front = l_front + r_front
        return min(front)


    def go_to_point(self):
        speed = Twist()

        inc_x = self.goal.x - self.x
        inc_y = self.goal.y - self.y

        angle_to_goal = atan2(inc_y, inc_x)

        # print(degrees(angle_to_goal), degrees(self.yaw))

        if abs(angle_to_goal - self.yaw) > 0.15:
            print("rotating")
            speed.linear.x = 0
            speed.angular.z = -0.3
        else:
            print("moving to point")
            speed.linear.x = 0.175
            speed.angular.z = 0

        self.cmd_publisher.publish(speed)

        return speed
        
        
    def follow_wall(self):
        rospy.sleep(2)
        speed = Twist()
        speed.linear.x = self.linear_Speed
        speed.angular.z = 0
        left_dist = self.distance_from_left_wall()
        upper_left = self.distance_from_upper_left_wall()
        downer_left = self.distance_from_downer_left_wall()
        front_dist = self.distance_from_front_wall()
        
        error = 0
        integral = 0
        derivetive = 0
        privous_error = 0
        print("debug 0")

        while not rospy.is_shutdown():
            print("debug 1")

            error = left_dist - self.distance_margin

            P = self.kp * error
            I = self.ki * integral
            D = self.kd * derivetive

            if sum(self.is_obstacle) >= 2:
                speed.angular.z = P+I+D
                speed.linear.x = self.linear_Speed
                print(" follow wall")
            else:
                speed = self.go_to_point()
            
            self.cmd_publisher.publish(speed)
                

            ## update controller
            if front_dist > 0.7:
                self.is_obstacle[0] = False
            else:
                self.is_obstacle[0] = True

            if upper_left > 0.7:
                self.is_obstacle[1] = False
            else:
                self.is_obstacle[1] = True

            if downer_left > 0.7:
                self.is_obstacle[2] = False
            else:
                self.is_obstacle[2] = True


            # integral += error * self.dt
            derivetive = error - privous_error
            privous_error = error

            left_dist = self.distance_from_left_wall()
            upper_left = self.distance_from_upper_left_wall()
            downer_left = self.distance_from_downer_left_wall()
            front_dist = self.distance_from_front_wall()

            self.rate.sleep()


    def on_shutdown(self):
        rospy.sleep(1)


if __name__ == "__main__":
    controller = Wall_follwer()
    controller.follow_wall()
    # controller.go_to_point()