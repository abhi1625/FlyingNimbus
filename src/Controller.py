#!/usr/bin/env python
import time
import rospy
import numpy as np 
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import math

class Controller:
    def __init__(self):
        # subscriber for current state
        self.state_sub = rospy.Subscriber('/current_state', Pose, self.state_cb)
	# subscriber forcurrent velocity
	self.vel_sub = rospy.Subscriber('/bebop/odom',Odometry, self.vel_cb)
        # subscriber for target pose
        self.target_sub = rospy.Subscriber('/relative_pose', Pose, self.target_cb)
        # publisher for control inputs
        self.cmd_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.flag_pub = rospy.Publisher('/exit_flag', Bool, queue_size = 1)

        # twist object for cmd_vel 
        self.vel = Twist()
        # pose object for current state
        self.state = Pose()
        # pose object for target state
        self.target = Pose()
        # velocity object for odometery
        self.curr_vel_odom = np.zeros((2,))

        # controller gains
        self.gain = np.array([0.50,0.5,0.3,0.05]) 

        # yaw bias
        self.yaw_bias = 0.0
    
    def state_cb(self, data):
        self.state.position.x = data.position.x
        self.state.position.y = data.position.y
        self.state.position.z = data.position.z
        self.state.orientation.z = data.orientation.z - self.yaw_bias

    def vel_cb(self, data):
	self.curr_vel_odom[0] = data.twist.twist.linear.x
	self.curr_vel_odom[1] = data.twist.twist.linear.y

    def target_cb(self, data):
        """
        data is relative position of the target in the quadrotor's body frame
        """
        self.target.position.x = data.position.x
        self.target.position.y = data.position.y
        self.target.position.z = data.position.z
        self.target.orientation.z = data.orientation.z 
	#print(self.target)

    def gen_ctrl_inputs(self):
        # relative angle between current and target heading
        # target orientation is relative to body frame
	#self.target.position.x = self.state.position.x #math.sqrt(3.0)/2.0
	#self.target.position.y = 1.0 #self.state.position.y
	#self.target.position.z = 0.0#2.0 - self.state.position.z
	#self.target.orientation.z = self.state.orientation.z #math.pi/6
        delta_th = self.target.orientation.z 
        #rot_mat = np.array([[math.cos(-delta_th), -math.sin(-delta_th)],
        #                    [math.sin(-delta_th),  math.cos(-delta_th)]])
        #trans_target = np.array([[self.target.position.x],
        #                         [self.target.position.y]])
	#print(self.target.orientation.z, delta_th)
        #rel_motion = np.matmul(rot_mat, trans_target)
        next_des = np.array([self.target.position.x, self.target.position.y, self.target.position.z])

        gains = np.array([[0.2236, 0.2657]])

        x_pos = np.array([- next_des[0],
                        self.curr_vel_odom[0]])
	#print(next_des)
        y_pos = np.array([- next_des[1],
                        self.curr_vel_odom[1]])

        # compute controller commands 
        x_cmd = - np.matmul(gains, x_pos)[0]
        y_cmd = - np.matmul(gains, y_pos)[0]
        z_cmd =   next_des[2]
        yaw_cmd = -delta_th
	if (self.target.position.x < 0.0):
		self.vel.linear.x = -0.015
	else:
		self.vel.linear.x = self.gain[0]*x_cmd
	#print("xcmd = {}, ycmd = {}, zcmd = {}, yawcmd = {}".format(x_cmd, y_cmd, z_cmd, yaw_cmd))
        # clip the x, y and yaw commands
        #if x_cmd > 0.2 :
        #    self.vel.linear.x = 0.2
        #elif x_cmd < -0.2:
        #    self.vel.linear.x = -0.2
        #else :
	print("curr x pos perceived", self.target.position.x)
	print("curr y perceived", self.target.position.y)
	if(self.target.position.x >=1.2):
            self.vel.linear.x = 0.015 
	elif (self.target.position.x> 0.0 and self.target.position.x<1.2):
	    if (abs(self.target.position.y)<0.1 and abs(self.target.position.z)<0.1 and abs(self.target.orientation.z) < 0.2 and self.target.position.y != 0.0 and self.target.position.z != 0.0 ):
	    	self.vel.linear.x = 0.0
            	flag = Bool()
            	flag.data = True
            	self.flag_pub.publish(flag)
		print("WOOOOOOOOOHHHHHHHHOOOOOOOOOOOOOO")
	    elif(self.target.position.y == 0.0 and self.target.position.z == 0.0):
		    
		self.vel.linear.x = 0.2
	    else:
		self.vel.linear.x = 0.0
	elif(self.target.position.x == 0.0 and self.target.position.y == 0.0 and self.target.position.z == 0.0):
            self.vel.linear.x = 0.0
	    self.vel.linear.y = 0.0
	    self.vel.linear.z = 0.0	
	elif (self.target.position.x < 0 and abs(self.target.position.y)<0.1 and abs(self.target.position.z)<0.1 and abs(self.target.orientation.z) < 0.2 and self.target.position.y != 0.0 and self.target.position.z != 0.0 ):
	    self.vel.linear.x = 0.0
	    self.vel.linear.y = 0.0
	    self.vel.linear.z = 0.0
	    flag = Bool()
	    flag.data = True
	    self.flag_pub.publish(flag)
	#if y_cmd > 0.15 :
        #    self.vel.linear.y = 0.15
        #elif y_cmd < -0.15:
        #    self.vel.linear.y = -0.15
        #else :
	if (abs(self.target.position.y) < 0.1):
	    self.vel.linear.y = 0
	    print("###############")
	else :
            self.vel.linear.y = self.gain[1]*y_cmd

        #if yaw_cmd > 0.1 :
        #    self.vel.angular.z = 0.1
        #elif yaw_cmd < -0.1:
        #    self.vel.angular.z = -0.1
        #else :
    	if abs(self.target.orientation.z) < 0.05:
	    self.vel.angular.z = 0.0
	else :
	    self.vel.angular.z = -self.gain[3]*yaw_cmd
	
	if abs(self.target.position.z) < 0.05:
	    self.vel.linear.z = 0.0
	else:
            self.vel.linear.z = self.gain[2]*z_cmd

	#print(self.vel.linear.y)
        
	#print("final x = {}, y = {}, z = {}, yaw = {}".format(self.vel.linear.x,self.vel.linear.y,self.vel.linear.z, self.vel.angular.z))
	if (self.target.position.x == self.target.position.y == self.target.position.z == 2.0):
		print("landing")
	elif(abs(self.target.position.y)>1.5 or abs(self.target.position.z)>2.0):
		print("discarding random values")
	else:
		self.cmd_pub.publish(self.vel)
        self.curr_vel_odom[0] = self.state.position.x 
        self.curr_vel_odom[1] = self.state.position.y
        return x_cmd


def main():
    rospy.init_node('Controller', anonymous=True)
    rate = rospy.Rate(10)
    ob = Controller()
    while not rospy.is_shutdown():
        ob.gen_ctrl_inputs()
        rate.sleep()

if __name__=='__main__':
    main()
