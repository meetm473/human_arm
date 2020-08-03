#!/usr/bin/env python3

import threading
from d_sensorFusion import Rpy

import sys, tty, termios
import threading
import rospy
from std_msgs.msg import Float64
				
class GazeboInterface(threading.Thread):	
		
	def __init__(self):
		threading.Thread.__init__(self)
		self.connect_gazebo = False
		
	def run(self):
		cur_rpy = [0,0,0]
		prev_rpy = [0,0,0]
		
		print('Initializing ROS node...')
		
		"""
		By default, rospy registers signal handlers so that it can exit on
		Ctrl-C. In some code, you may wish to disable this, including:
		- you are not calling init_node() from python main thread. Python 
		only allows signals to be registered from the main thread.
		- you are running rospy within wxPython or another GUI toolkit that
		has its own exit handlers.
		- you wish to have your own signal handlers by default.
		
		Source: roswiki
		"""
		rospy.init_node('human_arm_key_control', anonymous=True, disable_signals=True)
		
		print('Setting up publishers...')
		# define publishers
		self.body_pub = rospy.Publisher('/human_arm/body_joint_controller/command', Float64, queue_size=1000)
		self.shoulder_pub = rospy.Publisher('/human_arm/shoulder_move_1_joint_controller/command', Float64, queue_size=1000)
		self.upper_arm_pub = rospy.Publisher('/human_arm/upper_arm_joint_controller/command', Float64, queue_size=1000)
		self.gripper_pub = rospy.Publisher('/human_arm/gripper_finger_joint_controller/command', Float64, queue_size=1000)
		self.rate = rospy.Rate(8)
		
		
		while self.connect_gazebo and not rospy.is_shutdown():
			
			# Acquiring data from sensor fusion
			Rpy.lock.acquire()
			try:
				Rpy.lock.wait(0.1)
				# Converting deg to rad
				cur_rpy = [Rpy.rotateX, Rpy.rotateY, Rpy.rotateZ]
			except:
				print('Some prob')
			finally:
				Rpy.lock.release()
			
			# Setting limits
			if cur_rpy[2] > 3.14:
				cur_rpy[2] = 3.14
			elif cur_rpy[2] < -3.14:
				cur_rpy[2] = -3.14
			if cur_rpy[1] < -3.14:
				cur_rpy[1] = -3.14
			elif cur_rpy[1] > 0.19:
				cur_rpy[1] = 0.19
			if cur_rpy[0] > 1.5708:
				cur_rpy[0] = 1.5708
			elif cur_rpy[0] < -2:
				cur_rpy[0] = -2
			
			#print(cur_rpy)
			
			# Sending unique commands to the manipulator
			for i in range(0,3):
				if prev_rpy[i] != cur_rpy[i]:
					if i == 2:
						self.body_pub.publish(cur_rpy[i])
					elif i == 1:
						self.shoulder_pub.publish(cur_rpy[i])
					elif i == 0:
						self.upper_arm_pub.publish(cur_rpy[i])
					prev_rpy[i] = cur_rpy[i]
			#self.rate.sleep()
		print('Gazebo interface is disconnected')

if __name__ == '__main__':
	print('Please run the script a_imu_control.py')
