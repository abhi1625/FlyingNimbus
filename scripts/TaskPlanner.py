#!/usr/bin/env python
import rospy 
import smach
from std_msgs import Empty

# define state takeoff

class TakeOff(smach.State):
	"""docstring for ClassName"""
	def __init__(self, arg):
		smach.State.__init__(self,outcomes=['outcome2'])
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		
	def execute(self,userdata):
		rospy.loginfo('Executing take off')
		self.takeoff_pub.publish()
		rospy.sleep(3)


# define state land

class Land(smach.State):
	"""docstring for ClassName"""
	def __init__(self, arg):
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
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TAKEOFF', TakeOff(), 
                               transitions={'outcome2':'LAND'})
        smach.StateMachine.add('LAND', Land(), 
                               transitions={'outcome2':'TAKEOFF'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
