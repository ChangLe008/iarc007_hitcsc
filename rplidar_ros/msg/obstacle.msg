# pose of obstacle

Header header

uint8 num
float32[] o_x
float32[] o_y
uint8[]   flag_v
# This flag denotes the ability to calculate the orientation of the robot
# flag_v = 1 :	the velocity can be calculated
# flag_v = 0 :	Unable to calculate the velocity
# flag_v = 2-9 :	Reserved to denote the accuracy of velocity detection
float32[] v_x
float32[] v_y   #-999
