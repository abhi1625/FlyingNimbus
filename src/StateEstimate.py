#!/usr/bin/env python
import time
import rospy
import numpy as np 
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged

class StateEstimate:
    def __init__(self):
        self.current_pos = np.zeros((3,))
        self.current_yaw = 0.0
        self.current_state = Pose()

        # Subscribers for bebop odom estimates
        self.bebop_odomSub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
        # Subscriber for heading
        self.bebop_yawSub = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged',Ardrone3PilotingStateAttitudeChanged, self.yaw_callback)
        # Current state publisher 
        self.state_pub = rospy.Publisher('/current_state', Pose, queue_size=1)

    def odom_callback(self, data):
        self.current_pos[0] = data.twist.twist.linear.x*0.2 + self.current_pos[0]
        self.current_pos[1] = data.twist.twist.linear.y*0.2 + self.current_pos[1]
        self.current_pos[2] = data.twist.twist.linear.z*0.2 + self.current_pos[2]
    
    def yaw_callback(self, data):
        self.current_yaw = data.yaw

def main():
    rospy.init_node('estimateion_node', anonymous=True)
    rate = rospy.Rate(10)
    estimation = StateEstimate()
    while not rospy.is_shutdown():
        # update current state position
        estimation.current_state.position.x = estimation.current_pos[0]
        estimation.current_state.position.y = estimation.current_pos[1]
        estimation.current_state.position.z = estimation.current_pos[2]
        
        # update current heading
        estimation.current_state.orientation.z = estimation.current_yaw

        rate.sleep()

if __name__=='__main__':
    main()        
