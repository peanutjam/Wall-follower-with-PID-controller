#!/usr/bin/env python

import rospy
import sys
import math
import tf
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
from tf.transformations import euler_from_quaternion
import numpy as np

class wall_follower:
    def __init__ (self):
        self.zig_state = 160
        self.linear_speed = 0.1
        self.angular_speed = math.pi/4
        self.theta = 0

        # init node
        rospy.init_node('wall_follower')

        # subscribers/publishers
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)

        # RUN rosrun prrexamples key_publisher.py to get /keys
        self.key_sub = rospy.Subscriber('keys', String, self.key_cb)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # set rate
        self.rate = rospy.Rate(10)
        self.t = Twist()
        self.state = "H"

        self.front = []
        self.front_left = []
        self.front_right = []
        self.left = []
        self.right = []
        self.back = []
        self.back_left = []
        self.back_right = []
        self.direction_dict = {"front": self.front, "front_left": self.front_left, "front_right": self.front_right,
        "left" :self.left, "right": self.right, "back" : self.back, "back_left": self.back_left, "back_right": self.back_right}

        self.index_dict = {"front": [i for i in range(0,23)] + [i for i in range(338, 360)], 
        "front_right": [i for i in range(23,68)], 
        "right": [i for i in range(68,113)],
        "back_right" :[i for i in range(113,158)], 
        "back": [i for i in range(158,203)], 
        "back_left" : [i for i in range(203,248)], 
        "left": [i for i in range(248,293)], 
        "front_left": [i for i in range(293,338)]}

        self.obstacle_detected = False
        self.following = False
        self.obstacle_index = 0
        self.target_angle = 0

        self.go_up = '\x1b[1A' 
        self.erase_line = '\x1b[2K'
        self.x = 0
        self.y = 0
        self.rotate_count = 5
        self.Distance_to_wall = 0.5
        self.Kp = 1.0
        self.Kp2 = 1.0 
        self.Kd = 0 
        self.angle = 90   


    # call back function for scanner   
    def scan_cb(self,msg):
        self.update(msg)
        if not self.following:
            self.set_state(msg)
        else:
            self.set_state_follow(msg)

    # set state for moving parallel to the wall PID controller implemented 
    def set_state_follow(self,msg):
        self.state = "parallel moving"
        data = np.array(msg.ranges)
        data[data<msg.range_min] = 9999
        data[data==float('inf')] = 9999
        Dmin_idx, Dmin = min(enumerate(data), key=lambda x: x[1])
        self.angle = msg.angle_min + Dmin_idx * (msg.angle_max - msg.angle_min)/len(data) 

        E = Dmin - self.Distance_to_wall 
        Wpd = (self.Kp * E + self.Kd * (E - Eprev) / 0.1) if Dmin < 0.1 else 0
        direction = -1 if data[269] < data[89] else 1 
        Wp = self.Kp2 * (self.angle-math.pi/2*direction)
        self.t.angular.z = Wpd + Wp 
        Eprev = E 
        self.cmd_vel_pub.publish(self.t)

        # if(Wpd + Wp > 3):
        #     self.t.linear.x = 0.2 * self.linear_speed
        # else:
        #     self.t.linear. x = self.linear_speed
    
    # helper function to see if there are more "inf" than numbers
    def more_inf (self, ls):
        inf_count = 0
        else_count = 0
        for i in ls:
            if (i == float('inf')):
                inf_count += 1
            else:
                else_count += 1
        return inf_count >= else_count

    # update all the data from the scanner to local variables
    def update(self,msg):
        self.front = msg.ranges[0 : 23] + msg.ranges[-22 : ]
        self.front_right = msg.ranges[23 : 68]
        self.right = msg.ranges[68 : 113]
        self.back_right = msg.ranges[113 : 158]
        self.back = msg.ranges[158 : 203]
        self.back_left = msg.ranges[203 : 248]
        self.left = msg.ranges[248 : 293]
        self.front_left = msg.ranges[293 : 338]
        self.direction_dict["front"] = self.front
        self.direction_dict["front_right"] = self.front_right
        self.direction_dict["right"] = self.right
        self.direction_dict["back_right"] = self.back_right
        self.direction_dict["back"] = self.back
        self.direction_dict["back_left"] = self.back_left
        self.direction_dict["left"] = self.left
        self.direction_dict["front_left"] = self.front_left

    # helper function: return the average of a list
    def avg(self,ls):
        return sum(ls) / len(ls)

    # return how many minimums a list have
    def hasmin(self, ls):
        for i in ls:
            count = 0
            if (i == min(ls)):
                count += 1
        return count

    # set the state before following a wall. It will randomly walk first. Then rotate to the wall. Then go to the wall
    def set_state(self,msg):
        if(min(msg.ranges) == float('inf') or self.hasmin(msg.ranges) > 1):
            self.state = "random walk"
            self.t.linear.x = self.linear_speed
            self.t.angular.z = random.choice[self.angular_speed, 0 , -self.angular_speed]
        elif(min(msg.ranges) != float('inf') and min(msg.ranges) not in self.direction_dict["front"] and min(msg.ranges) >=1):
            if(self.rotate_count > 0):
                self.rotate_count -= 1
                self.t.linear.x = self.linear_speed
            self.state = "rotate to wall"
            self.t.angular.z = self.angular_speed
        elif(min(msg.ranges) >= 1):
            self.state = "going to wall"
            self.t.angular.z = 0
            self.t.linear.x = self.linear_speed
        elif((min(msg.ranges) <= 1 and min(msg.ranges) not in self.direction_dict["left"] ) ):
            self.state = "rotate to parallel"
            self.t.angular.z = self.angular_speed
            self.t.linear.x = 0
        elif(min(msg.ranges) <= 1 and ( min(msg.ranges) in self.direction_dict["left"] or  min(msg.ranges) not in self.direction_dict["right"] )):
            self.state = "following the wall"
            self.t.angular.z = 0
            self.t.linear.x = self.linear_speed
            self.following = True


    # it is not necessary to add more code here but it could be useful
    def key_cb(self,msg): 
        # global state; global last_key_press_time
        self.state = msg.data
        self.last_key_press_time = rospy.Time.now()

    # odom is also not necessary but very useful
    def odom_cb(self,odom):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.rot_q = odom.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])

    # print the states of the robot
    def print_state(self):
        print("|--------------------------------------------------------------|")
        print("|       STATE: " + self.state + "                  ")

        print("|       Location x: " + str(self.x) + " y: " + str(self.y))
        print("|       linear speed :" +str(self.t.linear.x) + "                  ")
        print("|       angular speed :" +str(self.t.angular.z) + "                  ")
        if(len(self.direction_dict["right"]) != 0):
            print("|       obstacle ahead: " + str(min(self.direction_dict["right"])) + " m         ")
        else:
            print()
        print("|       theta: " + str(self.theta))
        print("|--------------------------------------------------------------|")
    
    # erase the lines
    def delete_lines(self, n=1): 
        for _ in range(n):
            sys.stdout.write(self.go_up) 
            sys.stdout.write(self.erase_line) 

    


    # the main funciton, loop
    def loop(self):
        while not rospy.is_shutdown():

            self.print_state()
            self.cmd_vel_pub.publish(self.t)

            # run at 10hz
            self.rate.sleep()

if __name__ == '__main__':
    try:
        #Testing our function
        robot = wall_follower()
        robot.loop()
    except rospy.ROSInterruptException: pass
