# a message used to share goals between team members

# the tf frame that x,y are expressed with respect to
string frame_id 

# a 3 digit number XYZ used to keep track of goals and captured panoramas:
#     X - the robot id
#     YZ - the panorama number the goal relates to
int16 goal_id 

# the position of the panorama with respect to the frame_id
float64 x
float64 y
