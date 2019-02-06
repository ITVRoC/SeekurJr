#!/usr/bin/python
import rospy
import roslib
import time
import math
import tf

#imports of messages

from pan_tilt_zoom.msg import pan_tilt_goal
from goal_publisher.msg import SchunkCoordenate
from sensor_msgs.msg import JointState

# Create the Class that will publish on the camera's topic
class ControlePTZ():

	# Class-creating method
	def __init__(self):
		
		# Declaration of variables
		self.X = 0.0
		self.Y = 0.0
		self.Z = 0.0
		self.pub = rospy.Publisher('/schunk_coordenates', SchunkCoordenate)
		#rospy.spin()


		# Command iniciate 
		command = SchunkCoordenate()

		rate = rospy.Rate(1000.0)
	
		print ("NODE RUNING: Schunk Coordenate")		
		while not rospy.is_shutdown():
			try:
				(trans, rot) = tf_listener.lookupTransform("/base_link", "/arm_6_link", rospy.Time(0))
				#print trans, rot
				
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			


			self.X = (trans[0] * 1000)
			self.Y = (trans[1] * 1000)
			self.Z = (trans[2] * 1000)
			command.msgX=self.X
			command.msgY=self.Y
			command.msgZ=self.Z
			self.pub.publish(command)
			#time.sleep(0.2)			

	

# Main Function#
if __name__ == '__main__':

	# Create the node
	rospy.init_node('schunk_coordenate', anonymous=True)

	# Creates tf listener
	tf_listener = tf.TransformListener()
	
	# Create a object
	try:
		obj_no = ControlePTZ()
	except rospy.ROSInterruptException: pass
