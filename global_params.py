import time
import serial
from pySerialTransfer import pySerialTransfer as txfer
alpha = 2 # multiplier for distance to target
beta = 3 # multiplier for angle of movement
gamma = 4 # multiplier for size of robot so that it does not hit any obstacle

show_animation = True # will be set to false during the hardware integration with robots
grid_size = 1.0  # workspace grid size 
robot_radius = 1.0  # robot radius - size of the robot. 
incr_d = grid_size # make incremental step equal to grid size or less for smoother movement
o_rad = 1.0 # obstacle radius
simulation = False #Whether running simulation or not

#current orientation wrt x-axis. It is 0 at start
robot1_orientation = 0 # in degrees wrt x-axis
#robot2_orientation = 0 # in degrees wrt x-axis
#robot3_orientation = 0 # in degrees wrt x-axis

robot1_angle = 0 # in degrees. Initialization for angle to next step/target
claw_opened1 = False # whather claw is opened for robot 1 for picking object
# goal reached variable
goal_reached_1 = False
#goal_reached_2 = False
#goal_reached_3 = False

# object selected by robot at start or not
obj1_is_selected_to_pick = False
obj2_is_selected_to_pick = False
obj3_is_selected_to_pick = False

# Object actually picked by robot or not
robot1_obj_picked = False
#robot2_obj_picked = False
#robot3_obj_picked = False

# objects
objx = [0.0, 5.0, 5.0] #objects x position list of left bottom corner
objy = [3.0, 12.0, 20.0] #objects y position list of left bottom corner
obj_ht = 1 #object height
obj_wd = 1 # object width
# object center will be objx + obj_wd/2 and objy+ obj_ht/2

#initialize other params
obj1x, obj1y = [5, 10]
obj2x, obj2y = [5, 12]
obj3x, obj3y = [5, 20]

next_x1, next_y1 = [0.0, 0.0]
#next_x2, next_y2 = [0.0, 0.0]
#next_x3, next_y3 = [0.0, 0.0]

#Robots - start position 
s1x, s1y = [0.0, 0.0]
#s2x, s2y = [0.0, 10.0]
#s3x, s3y = [0.0, 20.0]

#Robots - actual position on the ground (for error correction)
actual_x1, actual_y1 = [s1x, s1y]
prev_x1, prev_y1 = [0,0]

#Robots - goal position 
g1x, g1y = [30.0, 30.0]
#g2x, g2y = [30.0, 30.0]
#g3x, g3y = [30.0, 30.0]

#Robots previous positions

old_robot_x1, old_robot_y1 = [0.0, 0.0]
#old_robot_x2, old_robot_y2 = [0.0, 0.0]
#old_robot_x3, old_robot_y3 = [0.0, 0.0]


ox = [15.0, 5.0, 20.0, 23.0]  # obstacle x position list 
oy = [25.0, 15.0, 10.0, 25.0]  # obstacle y position list 

logFile = open("runLog.txt","w")
resultFile = open("result.txt","w")
stepno_r1 = 0
timeFile = open("timetaken.txt","w")

#serial communication
port = 'com6'
arduino = serial.Serial(port, 9600);
time.sleep(2) #give tome for the connection


