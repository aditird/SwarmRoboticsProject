
1. global_params: This is a global parameters file. In this file global variables that are used by all the programs are declared and initialized.
2. calcdistance_v4: This program computes the distance and the angle between 2 gives points (co-ordinates).
3. Robot_video_tracking: This program captures the video frames f the robot and its movements. Initially in this program the co-ordinates of the robot, object, obstacles and goal are determined at the start. 
4. robot_main: This program is the main program that has the algorithm for Pattern Search. In this program neighbouring points of the robot are considered to calcualte the objective function and the best next step, avoiding obstacles, for the robot to take is determined. This program. This program can also be run for simulation purpose.
5. robot_movement: This program constructs the message that must be sent to the robot based on the next step decided by robot_main program.
6. comm: This program sends the message to the robot Arduino microcontroller via NRF24 radio transceiver module.
Note:
To run the robot and perform object transport run the program "robot_video_tracking.py" which inturn calls robot_main and other programs.