#!/usr/bin/env python3

import sys, tty, termios
import threading
import rospy
from std_msgs.msg import Float64

# input variables
ch = ''

def read_input():
	global ch
	while True:
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		
		# interpretting input
		if ch=='h':
			show_help()
		elif ch == 'x':
			break
		elif ch=='q':
			cur_cmd[0] += 0.1
			if cur_cmd[0] > 3.14:
				cur_cmd[0] = 3.14
		elif ch=='e':
			cur_cmd[0] -= 0.1
			if cur_cmd[0] < -3.14:
				cur_cmd[0] = -3.14
		elif ch=='d':
			cur_cmd[1] -= 0.1
			if cur_cmd[1] < -3.14:
				cur_cmd[1] = -3.14
		elif ch=='a':
			cur_cmd[1] += 0.1
			if cur_cmd[1] > 0.19:
				cur_cmd[1] = 0.19
		elif ch=='w':
			cur_cmd[2] -= 0.1
			if cur_cmd[2] < -2:
				cur_cmd[2] = -2
		elif ch=='s':
			cur_cmd[2] += 0.1
			if cur_cmd[2] > 1.5708:
				cur_cmd[2] = 1.5708
		elif ch=='p':
			cur_cmd[3] += 0.001
			if cur_cmd[3] > 0.055:
				cur_cmd[3] = 0.055
		elif ch=='o':
			cur_cmd[3] -= 0.001
			if cur_cmd[3] < 0:
				cur_cmd[3] = 0
		else:
			print('\nInvalid input. Press h to see help.\n')
    
    
def show_help():
	print('\nControl the human_arm using keyboard with following keys')
	print('q - move body CCW')
	print('e - move body CW')
	print('d - move shoulder right')
	print('a - move shoulder left')
	print('w - move upper arm up')
	print('s - move upper arm down')
	print('p - close gripper')
	print('o - open gripper')
	print('h - to show this help')
	print('x - to exit')

def send_cmds():
	for i in range(0,4):
		if prev_cmd[i] != cur_cmd[i]:
			if i == 0:
				body_pub.publish(cur_cmd[i])
			elif i == 1:
				shoulder_pub.publish(cur_cmd[i])
			elif i == 2:
				upper_arm_pub.publish(cur_cmd[i])
			elif i == 3:
				gripper_pub.publish(cur_cmd[i])
			prev_cmd[i] = cur_cmd[i]
			#print(cur_cmd)
	rate.sleep()
	
if __name__ == '__main__':

	# Control variables
	prev_cmd = [0,0,0,0]	# 0 - body | 1 - shoulder | 2 - upper arm | 3 - gripper
	cur_cmd = [0,0,0,0]

	# initialize the node
	rospy.init_node('human_arm_key_control', anonymous=False)
	
	# define publishers
	body_pub = rospy.Publisher('/human_arm/body_joint_controller/command', Float64, queue_size=1000)
	shoulder_pub = rospy.Publisher('/human_arm/shoulder_move_1_joint_controller/command', Float64, queue_size=1000)
	upper_arm_pub = rospy.Publisher('/human_arm/upper_arm_joint_controller/command', Float64, queue_size=1000)
	gripper_pub = rospy.Publisher('/human_arm/gripper_finger_joint_controller/command', Float64, queue_size=1000)
	
	# background daemon thread to take user input
	th_user_input = threading.Thread(target=read_input)
	th_user_input.daemon = True
	th_user_input.start()
	
	rate = rospy.Rate(8)
	try:
		show_help()
		while not (rospy.is_shutdown() or ch=='x'):
			send_cmds()
	except rospy.ROSInterruptException:
		pass
	finally:
		print('Ended human_arm key_control.')
	
	
