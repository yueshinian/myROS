#!/usr/bin/env python
import roslib
roslib.load_manifest('mobot6_interface')
import rospy
from mobot6_interface.msg import KeyboardNuclear
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

# 'm':MOVE 's':STOP 'r':RESET 'h':HOME 'z':AUTOSAMPLE 'a':AUTOSTOP
# 'f':DETECTOR(first) 'g':DETECTOR(second)
angularLevel = { '1':1.0, '2':2.0, '3':3.0, '4':4.0, '5':5.0, '6':0.0 }
nuclearBindings = {
                    'm':(60.0, 10.0, 0.0),
                    's':(-1.0, -1.0, -1.0),
                    'r':(-2.0, -2.0, -2.0),
                    'h':(-3.0, -3.0, -3.0),
                    'f':(-4.0, 1.0, 25.0),
                    'g':(-4.0, 2.0, 65.0),
                    'z':(-5.0, 30.0, 0.0),
                    'a':(-6.0, -6.0, -6.0),
                    'x':(-7.0, -7.0, -7.0)
                  }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def vels(angular):
        return "currently:\tAngularLevel\t%s" % (angular)

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	
        pub = rospy.Publisher('keyboard_input', KeyboardNuclear, queue_size = 1)
	rospy.init_node('teleop_keyboard')

        angLevel=rospy.get_param("~AngularLevel",0)
        frameID=rospy.get_param("~FrameID",7)

	joints = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
	cartesians = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
	moves = [ 0.0, 0.0, 0.0, 0.0 ]
	swings = [ 0.0, 0.0 ]
        nuclear = [ 0.0, 0.0, 0.0 ]
	grippers = 0.0
	commandid = 8  # corresponding branches: 1.STOP,2.MOVE,3.joints,4.cartesians,5.moves,6.swings,7.grippers,8.frames
	status = 0

	try:
		print msg
                print vels(angLevel)
		rate = rospy.Rate(100)
                while not rospy.is_shutdown():
			joints = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
			cartesians = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
			moves = [ 0.0, 0.0, 0.0, 0.0 ]
			swings = [ 0.0, 0.0 ]
                        nuclear = [ 0.0, 0.0, 0.0 ]
			grippers = 0.0
			commandid = 8
			key = getKey()

                        if key in angularLevel.keys():
                                angLevel = angularLevel[key]
                                print vels(angLevel)
				if (status == 14):
					print msg
				status = (status + 1) % 15
                        elif key in nuclearBindings.keys():
                                nuclear[0] = nuclearBindings[key][0]
                                nuclear[1] = nuclearBindings[key][1]
                                nuclear[2] = nuclearBindings[key][2]
                                commandid = 13
			else:
				joints = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
				cartesians = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
				moves = [ 0.0, 0.0, 0.0, 0.0 ]
				swings = [ 0.0, 0.0 ]
				grippers = 0.0
                                nuclear = [ 0.0, 0.0, 0.0 ]
                                commandid = 8
				if (key == '\x03'):
					break

                        keyboard = KeyboardNuclear()
                        for i in range(len(joints)):
                                keyboard.joints.append(joints[i])
			for i in range(len(cartesians)):
                                keyboard.cartesians.append(cartesians[i])
			for i in range(len(moves)):
				keyboard.moves.append(moves[i])
			for i in range(len(swings)):
                                keyboard.swings.append(swings[i])
                        for i in range(len(nuclear)):
				keyboard.nuclear.append(nuclear[i])
			if keyboard.nuclear[0] > 0:
				keyboard.nuclear[0]=keyboard.nuclear[0]*angLevel
                        keyboard.grippers = grippers
			keyboard.framesID = frameID
			keyboard.commandID = commandid
			print keyboard.nuclear, keyboard.commandID
			print " "
			pub.publish(keyboard)
			rate.sleep()

	except Exception , e:
		print e,"shit"



