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
        self.target_sub = rospy.Subscriber('/target_state', Pose, self.target_cb)
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
        self.gain = np.ones((4,)) 
    
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

    def gen_ctrl_inputs(self):
        # relative angle between current and target heading
        # target orientation is relative to body frame
        delta_th = self.target.orientation.z  
        rot_mat = np.array([[math.cos(-delta_th), -math.sin(-delta_th)],
                            [math.sin(-delta_th),  math.cos(-delta_th)]])
        trans_target = np.array([[self.target.position.x],
                                 [self.target.position.y]])
        rel_motion = np.matmul(rot_mat, trans_target)
        next_des = np.array([rel_motion[0], rel_motion[1], self.target.position.z])

        gains = np.array([[0.2236, 0.2657]])

        x_pos = np.array([[- next_des[0][0]],
                        [5.0*(self.state.position.x - self.curr_vel_odom[0])]])
	print(x_pos.shape)
        y_pos = np.array([- next_des[1][0],
                        5.0*(self.state.position.y - self.curr_vel_odom[1])])

        # compute controller commands 
        x_cmd = - np.matmul(gains, x_pos)
        y_cmd = - np.matmul(gains, y_pos)
        z_cmd =   next_des[2]
        yaw_cmd = delta_th

        # clip the x, y and yaw commands
        if x_cmd > 0.3 :
            self.vel.linear.x = 0.25
        elif x_cmd < -0.3:
            self.vel.linear.x = -0.25
        else :
            self.vel.linear.x = self.gain[0]*x_cmd

        if y_cmd > 0.3 :
            self.vel.linear.y = 0.25
        elif y_cmd < -0.3:
            self.vel.linear.y = -0.25
        else :
            self.vel.linear.y = self.gain[1]*y_cmd

        if yaw_cmd > 0.2 :
            self.vel.angular.z = 0.15
        elif x_cmd < -0.2:
            self.vel.angular.z = -0.15
        else :
            self.vel.angular.z = self.gain[3]*yaw_cmd

        self.vel.linear.z = self.gain[2]*z_cmd

        self.cmd_pub.publish(self.vel)
        self.curr_vel_odom[0] = self.state.position.x 
        self.curr_vel_odom[1] = self.state.position.y
        return rel_motion


def main():
    rospy.init_node('Controller', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ob = Controller()
        ob.gen_ctrl_inputs()
        rate.sleep()

if __name__=='__main__':
    main()
