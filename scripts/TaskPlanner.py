#!/usr/bin/env python
import rospy 
import smach
from std_msgs.msg import Empty, Bool
from Window_detection.window_detection import video_stream
from Wall_detection.video_stream import video_stream as vid_stream
from CCtag.tag_detection import BullsEyeDetection
from geometry_msgs.msg import Twist, Pose
from Bridge_detection.bridge_detection import video_stream as bridge_test
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

class TakeOff(smach.State):
	"""docstring for ClassName"""
	def __init__(self,h_ref):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		self.move_up = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
		self.h_reff = h_ref
		self.pose_pub = rospy.Publisher('/relative_pose', Pose, queue_size = 1)
		self.state_sub = rospy.Subscriber('/current_state', Pose, self.state_cb)
		self.vel = Pose()
		self.curr_state = Pose()

	def state_cb(self,data):
		self.curr_state = data


	def execute(self,userdata):
		rospy.loginfo('Executing take off')
		self.takeoff_pub.publish()
		rospy.sleep(3)
		
		rate = rospy.Rate(10)
		self.vel.position.x = 0.0
		self.vel.position.y = 0.0
		self.vel.position.z = 0.0
		self.vel.orientation.x = 0.0
		self.vel.orientation.y = 0.0
		self.vel.orientation.z = 0.0

		while((self.h_reff - self.curr_state.position.z)>=0.08 and not rospy.is_shutdown()):
			self.vel.position.z = self.h_reff - self.curr_state.position.z
			self.vel.position.x = 0.2
			self.vel.position.y = -0.2
			self.pose_pub.publish(self.vel)
			rate.sleep()
		self.vel.position.x = 0.0
		self.vel.position.y = 0.0
		self.vel.position.z = 0.0
		self.vel.orientation.x = 0.0
		self.vel.orientation.y = 0.0
		self.vel.orientation.z = 0.0
		self.pose_pub.publish(self.vel)		

		# vel = Twist()
		# vel.linear.z = 0.6
		#self.move_up.publish(vel)
		#rospy.sleep(2)
		#self.move_up.publish(vel)
		#rospy.sleep(2)
		#self.move_up.publish(vel)
		return 'outcome2'

# define state First wall detection

class FirstWall(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.flag_ob = Flag_sub()

	def execute(self, userdata):
		rospy.loginfo("Executing first wall detection")
		# call wall detection script
		ob = video_stream()
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
		return 'outcome2'


# define state First wall detection

class WindowDetection(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.ob = video_stream()

	def execute(self, userdata):
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
				return 'outcome2'
				break
		return 'outcome2'

class Punch_forward(smach.State):
	def __init__(self,err):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.rel_err = err
		self.pose_pub = rospy.Publisher('/relative_pose', Pose, queue_size = 1)
	
	def execute(self, userdata):
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
			if (vel.position.x >1.0):
				vel.position.x = 1.0
			self.pose_pub.publish(vel)
			self.rel_err -= 0.05
			rate.sleep()
		vel.position.x = 0.0
		vel.position.y = 0.0
		vel.position.z = 0.0
		self.pose_pub.publish(vel)					
		return 'outcome2'



class PrepareForBridge(smach.State):
	def __init__(self,h_ref,yaw_err):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.h_reff = h_ref
		self.yaw_errr = yaw_err
		self.pose_pub = rospy.Publisher('/relative_pose', Pose, queue_size = 1)
	        self.state_sub = rospy.Subscriber('/current_state', Pose, self.state_cb)
		self.vel = Pose()
		self.curr_state = Pose()

	def state_cb(self,data):
		self.curr_state = data

	def execute(self, userdata):
		
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
		#if (self.yaw_err) <0.0:
		#	yaw_cmd = -6.0
		#else:
		#		yaw_cmd = 6.0
		#self.yaw_err = abs(self.yaw_err)
		while(not(self.yaw_errr<0.2) and not rospy.is_shutdown()):
			# print("this is a test")
			self.vel.position.z = self.h_reff - self.curr_state.position.z
			self.vel.orientation.z = -6.0
			self.vel.position.y = -0.1
			self.vel.position.x = 0.2
			self.pose_pub.publish(self.vel)
			self.yaw_errr -= 0.2
			print(self.yaw_errr)
			rate.sleep()
		self.vel.position.z = 0.0
		self.vel.position.x = 0.0
		self.vel.position.y = 0.0
		self.vel.orientation.z = 0.0
		self.pose_pub.publish(self.vel)	
		return 'outcome2'




class HeightControl(smach.State):
	def __init__(self,h_ref):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.h_reff = h_ref
		self.pose_pub = rospy.Publisher('/relative_pose', Pose, queue_size = 1)
	        self.state_sub = rospy.Subscriber('/current_state', Pose, self.state_cb)
		self.vel = Pose()
		self.curr_state = Pose()

	def state_cb(self,data):
		self.curr_state = data

	def execute(self, userdata):
		
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
		#if (self.yaw_err) <0.0:
		#	yaw_cmd = -6.0
		#else:
		#		yaw_cmd = 6.0
		#self.yaw_err = abs(self.yaw_err)
		while(not((self.h_reff - self.curr_state.position.z) < 0.1) and not rospy.is_shutdown()):
			# print("this is a test")
			self.vel.position.z = self.h_reff - self.curr_state.position.z
			self.pose_pub.publish(self.vel)
			rate.sleep()
		self.vel.position.z = 0.0
		self.vel.position.x = 0.0
		self.vel.position.y = 0.0
		self.vel.orientation.z = 0.0
		self.pose_pub.publish(self.vel)	
		return 'outcome2'





# define state Bridge detection

class BridgeDetection(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.StereoVO_obj = bridge_test()


	def execute(self, userdata):
		rospy.loginfo("Executing Bridge detection")
		# call Bridge detection script
		rate = rospy.Rate(30)
		flag_ob = Flag_sub()
		flag = Bool()
		flag.data = False
		flag_ob.flag_pub.publish(flag)
		while(not rospy.is_shutdown()):
			self.StereoVO_obj.execute()
			rate.sleep()
			if(flag_ob.flag.data):
				return 'outcome2'
				break
		return 'outcome2'


# define state CCTag detection

class CCTagDetection(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.tag_ob = BullsEyeDetection()

	def execute(self, userdata):
		rospy.loginfo("Executing CCTag detection")
		# call CCTag detection script
		rate = rospy.Rate(30)
		flag_ob = Flag_sub()
		flag = Bool()
		flag.data = False
		print("success")
		flag_ob.flag_pub.publish(flag)
		while(not rospy.is_shutdown()):
			self.tag_ob.run_pipeline()
			rate.sleep()
			if(flag_ob.flag.data):
				return 'outcome2'
				break
		return 'outcome2'



# define state SecondWall detection

class SecondWall(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])

	def execute(self, userdata):
		rospy.loginfo("Executing SecondWall detection")
		# call SecondWall detection script



# define state Square Tag detection

class SquareTagDetection(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])

	def execute(self, userdata):
		rospy.loginfo("Executing square tag detection")
		# call square tag detection script



# define state land

class Land(smach.State):
	"""docstring for ClassName"""
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1 , latch=True)
		self.flag_pub = rospy.Publisher('/relative_pose',Pose, queue_size=1,latch=True)
	def execute(self,userdata):
		#rospy.sleep(2)
		rospy.loginfo('Executing landing command')
		self.land_pub.publish()
		pose_flag = Pose()
		pose_flag.position.x = 2.0
		pose_flag.position.y = 2.0
		pose_flag.position.z = 2.0
		self.flag_pub.publish(pose_flag)
		rospy.sleep(5)
		return 'outcome2'


# main
def main():
    rospy.init_node('task_planner')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['SMend', 'outcome5'])

    # Open the container
    with sm:
    # Add states to the container
	smach.StateMachine.add('TAKEOFF', TakeOff(1.8), transitions={'outcome2':'FIRSTWALL'})

	#'HEIGHT'
	smach.StateMachine.add('HEIGHT', HeightControl(1.5), transitions={'outcome2':'FIRSTWALL'})
	rospy.sleep(15)


    	smach.StateMachine.add('FIRSTWALL', Punch_forward(1.5), transitions={'outcome2':'WINDOWYAW'})
    	smach.StateMachine.add('WINDOWYAW',PrepareForBridge(1.5,2.6), transitions={'outcome2':'WINDOW'})
    	smach.StateMachine.add('WINDOW', WindowDetection(), transitions={'outcome2':'PUNCH'})
    	smach.StateMachine.add('PUNCH',Punch_forward(1.3), transitions={'outcome2':'PREP'})
    	smach.StateMachine.add('PREP',PrepareForBridge(1.5,3.2), transitions={'outcome2':'BRIDGE'})

    	#not working as of now
    	smach.StateMachine.add('PBRIDGE',Punch_forward(0.4), transitions={'outcome2':'BRIDGE'})
    	smach.StateMachine.add('BRIDGE',BridgeDetection(), transitions={'outcome2':'CROSSBRIDGE'})
	smach.StateMachine.add('CROSSBRIDGE',Punch_forward(0.8), transitions={'outcome2':'YAWCC'})
	smach.StateMachine.add('YAWCC', PrepareForBridge(1.5, 2.8), transitions={'outcome2':'CCTAGPUNCH'})
	smach.StateMachine.add('CCTAGPUNCH',Punch_forward(0.4), transitions={'outcome2':'CCTAG'})
    	smach.StateMachine.add('CCTAG', CCTagDetection(), transitions={'outcome2':'LANDF'})
    	smach.StateMachine.add('LANDF', Land(),transitions={'outcome2':'TAKEOFF2'})	
    	smach.StateMachine.add('TAKEOFF2', TakeOff(1.0),transitions={'outcome2':'YAW'})
	smach.StateMachine.add('YAW',PrepareForBridge(0.9,1.5), transitions={'outcome2':'SECONDWALL'})	
    	smach.StateMachine.add('SECONDWALL', Punch_forward(1.2),transitions={'outcome2':'TAG2'})
    	smach.StateMachine.add('TAG2', CCTagDetection(),transitions={'outcome2':'LANDF2'})	
    	smach.StateMachine.add('LANDF2', Land(),transitions={'outcome2':'SMend'})
	

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
