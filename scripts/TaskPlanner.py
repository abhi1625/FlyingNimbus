#!/usr/bin/env python
import rospy 
import smach
from std_msgs import Empty

# define state takeoff

class TakeOff(smach.State):
	"""docstring for ClassName"""
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		
	def execute(self,userdata):
		rospy.loginfo('Executing take off')
		self.takeoff_pub.publish()
		rospy.sleep(3)

# define state First wall detection

class FirstWall(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])

	def execute(self, userdata):
		rospy.loginfo("Executing first wall detection")
		# call wall detection script


# define state First wall detection

class WindowDetection(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])

	def execute(self, userdata):
		rospy.loginfo("Executing Window detection")
		# call Window detection script


# define state Bridge detection

class BridgeDetection(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])

	def execute(self, userdata):
		rospy.loginfo("Executing Bridge detection")
		# call Bridge detection script


# define state CCTag detection

class CCTagDetection(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['outcome2'])

	def execute(self, userdata):
		rospy.loginfo("Executing CCTag detection")
		# call CCTag detection script



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
		
	def execute(self,userdata):
		rospy.loginfo('Executing landing command')
		self.land_pub.publish()
		rospy.sleep(3)


# main
def main():
    rospy.init_node('task_planner')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['SMend', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TAKEOFF', TakeOff(), 
                               transitions={'outcome2':'FIRSTWALL'})
        
        smach.StateMachine.add('FIRSTWALL', FirstWall(), 
                               transitions={'outcome2':'WINDOW'})

        smach.StateMachine.add('WINDOW', WindowDetection(), 
                               transitions={'outcome2':'BRIDGE'})

        smach.StateMachine.add('BRIDGE', BridgeDetection(), 
                               transitions={'outcome2':'CCTAG'})

        smach.StateMachine.add('CCTAG', CCTagDetection(), 
                               transitions={'outcome2':'LANDCC'})

        smach.StateMachine.add('LANDCC', Land(), 
                               transitions={'outcome2':'TAKEOFFSW'})

        smach.StateMachine.add('TAKEOFFSW', TakeOff(), 
                               transitions={'outcome2':'SECONDWALL'})

        smach.StateMachine.add('SECONDWALL', SecondWall(), 
                               transitions={'outcome2':'SQTAG'})

        smach.StateMachine.add('SQTAG', SquareTagDetection(), 
                               transitions={'outcome2':'LANDF'})

        smach.StateMachine.add('LANDF', Land(), 
                               transitions={'outcome2':'SMend'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
