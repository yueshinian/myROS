#!/usr/bin/env python
import roslib
roslib.load_manifest('mobot6_interface')
import rospy
from mobot6_interface.msg import Keyboard
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Keyboard.msg !
---------------------------
(1-7) or shift+(1-7) for position control of the manipulator in joint space
(q-y) or shift+(q-y) for position control of the manipulator in cartesian space
(uiojlm,.) for position control of movebase navigation
(890) for position control of swingarm
(-=) for the selection of the manipulator reference frame
(gG) for position control of gripper

CTRL-C to quit
"""

# the default velocity is 3deg/s or 2mm/s, and GMAS_TIME in test is 0.05s
jointBindings = {
		'1':( 0.15, 0.0, 0.0, 0.0, 0.0, 0.0 ),
		'2':( 0.0, 0.15, 0.0, 0.0, 0.0, 0.0 ),
		'3':( 0.0, 0.0, 0.15, 0.0, 0.0, 0.0 ),
		'4':( 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, ),
		'5':( 0.0, 0.0, 0.0, 0.0, 0.15, 0.0 ),
		'6':( 0.0, 0.0, 0.0, 0.0, 0.0, 0.15 ),
		'!':( -0.15, 0.0, 0.0, 0.0, 0.0, 0.0 ),
		'@':( 0.0, -0.15, 0.0, 0.0, 0.0, 0.0 ),
		'#':( 0.0, 0.0, -0.15, 0.0, 0.0, 0.0 ),
		'$':( 0.0, 0.0, 0.0, -0.15, 0.0, 0.0 ),
		'%':( 0.0, 0.0, 0.0, 0.0, -0.15, 0.0 ),
		'^':( 0.0, 0.0, 0.0, 0.0, 0.0, -0.15 ),
	       }

# the default velocity is 3deg/s and 5mm/s, and GMAS_TIME in test is 0.05s
cartesianBindings={
		'q':( 0.25, 0.0, 0.0, 0.0, 0.0, 0.0 ),
		'w':( 0.0, 0.25, 0.0, 0.0, 0.0, 0.0 ),
		'e':( 0.0, 0.0, 0.25, 0.0, 0.0, 0.0 ),
		'r':( 0.0, 0.0, 0.0, 0.10, 0.0, 0.0 ),
		't':( 0.0, 0.0, 0.0, 0.0, 0.10, 0.0 ),
		'y':( 0.0, 0.0, 0.0, 0.0, 0.0, 0.10 ),
		'Q':( -0.25, 0.0, 0.0, 0.0, 0.0, 0.0 ),
		'W':( 0.0, -0.25, 0.0, 0.0, 0.0, 0.0 ),
		'E':( 0.0, 0.0, -0.25, 0.0, 0.0, 0.0 ),
		'R':( 0.0, 0.0, 0.0, -0.10, 0.0, 0.0 ),
		'T':( 0.0, 0.0, 0.0, 0.0, -0.10, 0.0 ),
		'Y':( 0.0, 0.0, 0.0, 0.0, 0.0, -0.10 )
	      }

# the default velocity is 1deg/s and 3mm/s, and GMAS_TIME in test is 0.05s
homeBindings = {
                'z':1.0, 'x':2.0, 'c':3.0, 'v':4.0, 'b':5.0,
                'n':6.0, 'h':7.0, ';':8.0, '/':9.0
               }

moveBindings = {
		'i':( 0.5, 0.0, 0.0, 0.0 ),
		'o':( 0.5, 0.0, 0.0, -0.5236 ),
		'j':( 0.0, 0.0, 0.0, 0.5236 ),
		'l':( 0.0, 0.0, 0.0, -0.5236 ),
		'u':( 0.5, 0.0, 0.0, 0.5236 ),
		',':( -0.5, 0.0, 0.0, 0.0 ),
		'.':( -0.5, 0.0, 0.0, 0.5236 ),
		'm':(-0.5, 0.0, 0.0, -0.5236)
	       }
moveSpeeds = { 'p':3.0, '[':6.0, ']':9.0 }

swingBindings = {
		'8':(0.1745, 0.0),
		'9':(0.0, 0.1745),
		'0':(0.1745, 0.1745),
		'*':(-0.1745, 0.0),
		'(':(0.0, -0.1745),
		')':(-0.1745, -0.1745),
		'a':(6.0, 6.0),
		's':(7.0, 7.0),
		'd':(8.0, 8.0),
	        }

gripperBindings = { 'g': 1, 'G': -1 }

frameSelections = { '-': 0, '=': 7 }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(jointSpeed, cartesianSpeed, moveSpeed, swingSpeed, gripperSpeed, frameID):
	return "currently:\tJointSpeed %s\tCartesianSpeed %s\tMoveSpeed %s\tSwingSpeed %s\tGripperSpeed %s\tFrameID %s" % (jointSpeed, cartesianSpeed, moveSpeed, swingSpeed, gripperSpeed, frameID+1)

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('keyboard_input', Keyboard, queue_size = 1)
	rospy.init_node('teleop_keyboard')

	jointSpeed = rospy.get_param("~JointSpeed", 1.00)
	cartesianSpeed = rospy.get_param("~CartesianSpeed", 1.00) 
	moveSpeed = rospy.get_param("~MoveSpeed", 3.00)
	swingSpeed=rospy.get_param("~SwingSpeed", 1.00)
	gripperSpeed=rospy.get_param("~GripperSpeed", 1.00)
        frameID=rospy.get_param("~FrameID",7)

	joints = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
	cartesians = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
	moves = [ 0.0, 0.0, 0.0, 0.0 ]
	swings = [ 0.0, 0.0 ]
	grippers = 0.0
	commandid = 8  # corresponding branches: 1.STOP,2.MOVE,3.joints,4.cartesians,5.moves,6.swings,7.grippers,8.frames
	status = 0

	try:
		print msg
		print vels(jointSpeed, cartesianSpeed, moveSpeed, swingSpeed, gripperSpeed, frameID)
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			joints = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
			cartesians = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
			moves = [ 0.0, 0.0, 0.0, 0.0 ]
			swings = [ 0.0, 0.0 ]
			grippers = 0.0
			commandid = 8
			key = getKey()
			if (key == '`'):
                                commandid = 0
			elif (key == '~'):
				commandid = 1
			elif key in jointBindings.keys():
				joints[0] = jointBindings[key][0]
				joints[1] = jointBindings[key][1]
				joints[2] = jointBindings[key][2]
				joints[3] = jointBindings[key][3]
				joints[4] = jointBindings[key][4]
				joints[5] = jointBindings[key][5]
				commandid = 2
			elif key in cartesianBindings.keys():
				cartesians[0] = cartesianBindings[key][0]
				cartesians[1] = cartesianBindings[key][1]
				cartesians[2] = cartesianBindings[key][2]
				cartesians[3] = cartesianBindings[key][3]
				cartesians[4] = cartesianBindings[key][4]
				cartesians[5] = cartesianBindings[key][5]
				commandid = 3
			elif key in moveSpeeds.keys():
				moveSpeed = moveSpeeds[key]
				print vels(jointSpeed, cartesianSpeed, moveSpeed, swingSpeed, gripperSpeed, frameID)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			elif key in moveBindings.keys():
				moves[0] = moveBindings[key][0] * moveSpeed
				moves[1] = moveBindings[key][1]
				moves[2] = moveBindings[key][2]
				moves[3] = moveBindings[key][3]
				commandid = 4
			elif key in swingBindings.keys():
				swings[0] = swingBindings[key][0]
				swings[1] = swingBindings[key][1]
				commandid = 5
			elif key in gripperBindings.keys():
				grippers = gripperBindings[key]
				commandid = 6
			elif key in frameSelections.keys():
				frameID = frameSelections[key]
                                if (status == 14):
                                        print msg
                                status = (status + 1) % 15
                        elif key in homeBindings.keys():
                                joints[0] = homeBindings[key]
                                joints[1] = homeBindings[key]
                                joints[2] = homeBindings[key]
                                joints[3] = homeBindings[key]
                                joints[4] = homeBindings[key]
                                joints[5] = homeBindings[key]
                                commandid = 9
                        elif (key == 'f'):
                                commandid = 10
                        elif (key == '?'):
                                commandid = 11
                        elif (key == '{'):
                                commandid = 12
			else:
				joints = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
				cartesians = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
				moves = [ 0.0, 0.0, 0.0, 0.0 ]
				swings = [ 0.0, 0.0 ]
				grippers = 0.0
                                commandid = 8
				if (key == '\x03'):
					break

                        keyboard = Keyboard()
                        for i in range(len(joints)):
				keyboard.joints.append(joints[i] * jointSpeed)
			for i in range(len(cartesians)):
				keyboard.cartesians.append(cartesians[i] * cartesianSpeed)
			for i in range(len(moves)):
				keyboard.moves.append(moves[i])
			for i in range(len(swings)):
				keyboard.swings.append(swings[i] * swingSpeed)
                        keyboard.grippers = grippers * gripperSpeed
			keyboard.framesID = frameID
			keyboard.commandID = commandid
			print keyboard.joints, keyboard.commandID
			print " "
			pub.publish(keyboard)
			rate.sleep()

	except Exception , e:
		print e,"shit"



