#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class TurtleBotSim():
	def __init__(self):
		rospy.init_node('ControlTurtleBot', anonymous=False)

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)

	def move(self):
		twist = Twist()
		twist.linear.x = 0.
		twist.angular.z = 0.5
		self.cmd_vel.publish(twist)
		rospy.loginfo("Twist published")

if __name__ == '__main__':
	turtlebotsim = TurtleBotSim()
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		turtlebotsim.move()
		r.sleep()
