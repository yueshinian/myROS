# The RobotStatus message contains low level status information 
# that is specific to an industrial robot controller

# The header frame ID is not used
Header header

# The robot mode captures the operating mode of the robot.  When in
# manual, remote motion is not possible.
mobot6_interface/RobotMode mode

# Estop status: True if robot is e-stopped.  The drives are disabled
# and motion is not possible.  The e-stop condition must be acknowledged
# and cleared before any motion can begin.
mobot6_interface/TriState e_stopped

# Drive power status: True if drives are powered.  Motion commands will 
# automatically enable the drives if required.  Drive power is not requred
# for possible motion
mobot6_interface/TriState drives_powered

# Motion enabled: Ture if robot motion is possible.
mobot6_interface/TriState motion_possible

# Motion status: True if robot is in motion, otherwise false
mobot6_interface/TriState in_motion

# Error status: True if there is an error condition on the robot. Motion may
# or may not be affected (see motion_possible)
mobot6_interface/TriState in_error

# Error code: Vendor specific error code (non zero indicates error)
int32 error_code
