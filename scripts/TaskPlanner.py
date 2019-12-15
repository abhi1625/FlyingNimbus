#!/usr/bin/env python
import rospy 
# import smach
from std_msgs.msg import Empty, Bool
from Window_detection.window_detection import video_stream
from Wall_detection.video_stream import video_stream as vid_stream
from geometry_msgs.msg import Twist, Pose
# define state takeoff
class Flag_sub:
	def __init__(self):
		self.flag_sb = rospy.Subscriber('/exit_flag', Bool, self.flag_cb)
		self.flag_pub = rospy.Publisher('/exit_flag', Bool, queue_size = 10)
		self.flag = Bool()
		#rospy.spin()
	def flag_cb(self, data):
		print("cb success")
		self.flag = data 

class TakeOff():
	"""docstring for ClassName"""
	def __init__(self):
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		self.move_up = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

	def __del__(self):
		print("deleted takeoff object")

	def execute(self):
		rospy.loginfo('Executing take off')
		self.takeoff_pub.publish()
		vel = Twist()
		vel.linear.z = 0.6
		rospy.sleep(3)
		self.move_up.publish(vel)
		rospy.sleep(2)
		self.move_up.publish(vel)
		rospy.sleep(2)
		self.move_up.publish(vel)
		return 'outcome'

# define state First wall detection

class FirstWall():
	def __init__():
		self.flag_ob = Flag_sub()

	def execute(self):
		rospy.loginfo("Executing first wall detection")
		# call wall detection script
		ob = vid_stream()
		count = 0
		rate = rospy.Rate(10)
		flag = Bool()
		flag.data = False
		
		self.flag_ob.flag_pub(flag)
		while(not rospy.is_shutdown()):
			rate.sleep()
			print count
			count +=1
			if(self.flag_ob.flag.data):
				break	
		return 'outcome'


# define state First wall detection

class WindowDetection():
	def __init__(self):
		self.ob = video_stream()
	def __del__(self):
		print("deleted window detection object")
		del self.ob
	def execute(self):
		flag_ob = Flag_sub()
		rospy.loginfo("Executing Window detection")
		# call Window detection script
		count = 0
		rate = rospy.Rate(10)
		flag = Bool()
		flag.data = False
		print("success")
		flag_ob.flag_pub.publish(flag)
		while(not rospy.is_shutdown()):
			print("exit flag",flag_ob.flag.data)
			self.ob.execute()
			rate.sleep()
			#print count
			count+=1
			if(flag_ob.flag.data):
				return 'outcome'
				break
		return 'outcome'

class Punch_forward():
	def __init__(self,err):
		self.rel_err = err
		self.pose_pub = rospy.Publisher('/relative_pose', Pose, queue_size = 1)

	def __del__(self):
		print("deleted Punch object")
	
	def execute(self):
		print("sleeping for 2 secs")
		rospy.sleep(2)
		vel = Pose()
		vel.position.x = 0.0
		vel.position.y = 0.0
		vel.position.z = 0.0
		vel.orientation.x = 0.0
		vel.orientation.y = 0.0
		vel.orientation.z = 0.0
		
		flag_ob = Flag_sub()
		rospy.loginfo("Executing Punch forward")
		rate = rospy.Rate(10)
		flag = Bool()
		flag.data = False
		flag_ob.flag_pub.publish(flag)
		while(self.rel_err>=0.0 and not rospy.is_shutdown()):
			print("this is a test")
			vel.position.x = self.rel_err
			self.pose_pub.publish(vel)
			self.rel_err -= 0.05
			rate.sleep()
		vel.position.x = 0.0
		self.pose_pub.publish(vel)					
		return 'outcome'



class GenXYZMove():
	def __init__(self,x_ref,y_ref,z_ref):
		self.x_target = x_ref
		self.y_target = y_ref
		self.z_target = z_ref

		# self.yaw_err = yaw_err
		self.pose_pub = rospy.Publisher('/relative_pose', Pose, queue_size = 1)
	        self.state_sub = rospy.Subscriber('/current_state', Pose, self.state_cb)
		self.vel = Pose()
		self.curr_state = Pose()

	def __del__(self):
		print("deleted GenXYZMove object")

	def state_cb(self,data):
		self.curr_state = data

	def execute(self):
		
		self.vel.position.x = 0.0
		self.vel.position.y = 0.0
		self.vel.position.z = 0.0
		self.vel.orientation.x = 0.0
		self.vel.orientation.y = 0.0
		self.vel.orientation.z = 0.0
		
		flag_ob = Flag_sub()
		rate = rospy.Rate(10)
		flag = Bool()
		flag.data = False
		flag_ob.flag_pub.publish(flag)
		while(not(abs(self.x_target - self.curr_state.position.x)<=0.08 and
				  abs(self.x_target - self.curr_state.position.y)<=0.08
				  abs(self.x_target - self.curr_state.position.z)<=0.08) and
				   not rospy.is_shutdown()):
			# print("this is a test")
			self.vel.position.z = self.h_reff - self.curr_state.position.z
			self.vel.orientation.z = self.yaw_err
			self.pose_pub.publish(self.vel)
			self.yaw_err -= 0.05
			rate.sleep()
		self.vel.position.z = 0.0
		self.vel.orientation.z = 0.0
		self.pose_pub.publish(self.vel)	
		return 'outcome'




class PrepareForBridge():
	def __init__(self,h_ref,yaw_err):
		self.h_reff = h_ref
		self.yaw_err = yaw_err
		self.pose_pub = rospy.Publisher('/relative_pose', Pose, queue_size = 1)
	        self.state_sub = rospy.Subscriber('/current_state', Pose, self.state_cb)
		self.vel = Pose()
		self.curr_state = Pose()

	def __del__(self):
		print("deleted prepare for bridge object")

	def state_cb(self,data):
		self.curr_state = data

	def execute(self):
		
		self.vel.position.x = 0.0
		self.vel.position.y = 0.0
		self.vel.position.z = 0.0
		self.vel.orientation.x = 0.0
		self.vel.orientation.y = 0.0
		self.vel.orientation.z = 0.0
		
		flag_ob = Flag_sub()
		rospy.loginfo("Preparing for bridge")
		rate = rospy.Rate(10)
		flag = Bool()
		flag.data = False
		flag_ob.flag_pub.publish(flag)
		while(not(abs(self.h_reff - self.curr_state.position.z)<=0.08 and abs(self.yaw_err)<=0.08) and not rospy.is_shutdown()):
			# print("this is a test")
			self.vel.position.z = self.h_reff - self.curr_state.position.z
			self.vel.orientation.z = self.yaw_err
			self.pose_pub.publish(self.vel)
			self.yaw_err -= 0.05
			rate.sleep()
		self.vel.position.z = 0.0
		self.vel.orientation.z = 0.0
		self.pose_pub.publish(self.vel)	
		return 'outcome'
# define state Bridge detection

class BridgeDetection():
	def __init__(self):
		pass
	def __del__(self):
		print("deleted object")

	def execute(self):
		rospy.loginfo("Executing Bridge detection")
		# call Bridge detection script


# define state CCTag detection

class CCTagDetection():
	def __init__(self):
		pass
	def execute(self):
		rospy.loginfo("Executing CCTag detection")
		# call CCTag detection script



# define state SecondWall detection

class SecondWall():
	def __init__(self):
		pass
	def execute(self):
		rospy.loginfo("Executing SecondWall detection")
		# call SecondWall detection script



# define state Square Tag detection

class SquareTagDetection():
	def __init__(self):
		pass
	def execute(self):
		rospy.loginfo("Executing square tag detection")
		# call square tag detection script



# define state land

class Land():
	"""docstring for ClassName"""
	def __init__(self):
		self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1 , latch=True)
		
	def execute(self):
		rospy.loginfo('Executing landing command')
		self.land_pub.publish()
		rospy.sleep(3)
		return 'outcome'


# main
def main():
	rospy.init_node('task_planner')

	# execute take off
	takeoff_ob = TakeOff()
	ret = takeoff_ob.execute()
	del takeoff_ob

	window_ob = WindowDetection()
	ret = window_ob.execute()
	del window_ob

	punch_ob = Punch_forward(1.0)
	ret = punch_ob.execute()
	del punch_ob

	prep_bridge_ob = PrepareForBridge(0.5,1.2)
	ret  = prep_bridge_ob.execute()
	del prep_bridge_ob

	land_ob = Land()
	ret = land_ob.execute()
	del land_ob


if __name__ == '__main__':
    main()
