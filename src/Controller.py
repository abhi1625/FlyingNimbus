#!/usr/bin/env python
import time
import rospy
import numpy as np 
from geometry_msgs.msg import Twist, Pose
import math

class Controller:
    def __init__(self):
        # subscriber for current state
        self.state_sub = rospy.Subscriber('/current_state', Pose, self.state_cb)
        # subscriber for target pose
        self.target_sub = rospy.Subscriber('/relative_pose', Pose, self.target_cb)
        # publisher for control inputs
        self.cmd_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        # twist object for cmd_vel 
        self.vel = Twist()
        # pose object for current state
        self.state = Pose()
        # pose object for target state
        self.target = Pose()
        # velocity object for odometery
        self.curr_vel_odom = np.zeros((2,))

        # controller gains
        self.gain = np.array([1.0,0.8,0.5,0.5]) 
    
    def state_cb(self, data):
        self.state.position.x = data.position.x
        self.state.position.y = data.position.y
        self.state.position.z = data.position.z
        self.state.orientation.z = data.orientation.z

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
	self.target.position.x = self.state.position.x #math.sqrt(3.0)/2.0
	#self.target.position.y = 1.0 #self.state.position.y
	#self.target.position.z = 0.0#2.0 - self.state.position.z
	#self.target.orientation.z = self.state.orientation.z #math.pi/6
        delta_th = self.target.orientation.z 
        rot_mat = np.array([[math.cos(-delta_th), -math.sin(-delta_th)],
                            [math.sin(-delta_th),  math.cos(-delta_th)]])
        trans_target = np.array([[self.target.position.x],
                                 [self.target.position.y]])
	#print(self.target.orientation.z, delta_th)
        rel_motion = np.matmul(rot_mat, trans_target)
        next_des = np.array([rel_motion[0], rel_motion[1], self.target.position.z])

        gains = np.array([[1.2*0.2236, 1.5*0.2657]])

        x_pos = np.array([[- next_des[0][0]],
                        [5.0*(self.state.position.x - self.curr_vel_odom[0])]])
	#print(next_des)
        y_pos = np.array([- next_des[1][0],
                        5.0*(self.state.position.y - self.curr_vel_odom[1])])

        # compute controller commands 
        x_cmd = - np.matmul(gains, x_pos)[0][0]
        y_cmd = - np.matmul(gains, y_pos)[0]
        z_cmd =   next_des[2]
        yaw_cmd = -delta_th
	print(x_cmd, y_cmd, z_cmd, yaw_cmd)
        # clip the x, y and yaw commands
        if x_cmd > 0.2 :
            self.vel.linear.x = 0.2
        elif x_cmd < -0.2:
            self.vel.linear.x = -0.2
        else :
            self.vel.linear.x = self.gain[0]*x_cmd

        #if y_cmd > 0.15 :
        #    self.vel.linear.y = 0.15
        #elif y_cmd < -0.15:
        #    self.vel.linear.y = -0.15
        #else :
	if (abs(self.target.position.y - self.state.position.y) < 0.1):
	    self.vel.linear.y = 0
	    print("###############")
	else :
            self.vel.linear.y = self.gain[1]*y_cmd

        if yaw_cmd > 0.1 :
            self.vel.angular.z = 0.1
        elif yaw_cmd < -0.1:
            self.vel.angular.z = -0.1
        else :
            self.vel.angular.z = self.gain[3]*yaw_cmd

        self.vel.linear.z = self.gain[2]*z_cmd

	#print(self.vel.linear.y)
        self.cmd_pub.publish(self.vel)
        self.curr_vel_odom[0] = self.state.position.x 
        self.curr_vel_odom[1] = self.state.position.y
        return rel_motion


def main():
    rospy.init_node('Controller', anonymous=True)
    rate = rospy.Rate(15)
    ob = Controller()
    while not rospy.is_shutdown():
        ob.gen_ctrl_inputs()
        rate.sleep()

if __name__=='__main__':
    main()
