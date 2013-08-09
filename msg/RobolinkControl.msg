# This message will be used to control the Robolink via direct joint control, pose control, or twist control

# Control Mode Options
uint8 JOINT_VELOCITY =1
uint8 JOINT_ABSOLUTE_POSITION = 2
uint8 JOINT_RELATIVE_POSITION = 3
uint8 POSE_CONTROL = 4
uint8 TWIST_CONTROL = 5

Header header

uint8 control_mode

#Direct joint control
int32 joint_0_setpoint
int32 joint_1_setpoint
int32 joint_2_setpoint
int32 joint_3_setpoint
int32 joint_4_setpoint

#Pose Control
geometry_msgs/Pose pose

#Twist Control
geometry_msgs/Twist twist