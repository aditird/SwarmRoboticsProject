# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
#from calcdistance_v4 import calculateDistAngle
#from pattern_search_3robots_v5 import pattern_search_path
import numpy as np
import imutils
import global_params as gp
from robot_main import init
from robot_main import robot_plotting
import matplotlib.pyplot as plt
import sys
from calcdistance_v4 import calculateDistAngle
import math
import pdb


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
        help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="kcf",
        help="kcf")
args = vars(ap.parse_args())
run_started = False

trkr ="csrt"
# initialize a dictionary that maps strings to their corresponding
# OpenCV object tracker implementations
OPENCV_OBJECT_TRACKERS = {
        "csrt": cv2.TrackerCSRT_create,
        "kcf": cv2.TrackerKCF_create,
        "boosting": cv2.TrackerBoosting_create,
        "mil": cv2.TrackerMIL_create,
        "tld": cv2.TrackerTLD_create,
        "medianflow": cv2.TrackerMedianFlow_create,
        "mosse": cv2.TrackerMOSSE_create
}

# initialize OpenCV's special multi-object tracker
trackers = cv2.MultiTracker_create()

# if a video path was not supplied, grab the reference to the web cam
if not args.get("video", False):
        print("[INFO] starting video stream...")
        vs = VideoStream(src=0).start()
        time.sleep(1.0)

# otherwise, grab a reference to the video file
else:
        #vs = cv2.VideoCapture("C:\aditi\swarm robots\multi-object-tracking\multi-object-tracking\videos\soccer_01")
     vs = cv2.VideoCapture(args["video"])
# loop over frames from the video stream
prev_robot_x = 0
prev_robot_y = 0

# starting positions of robots
robot1_pos_fixed = False
obj1_pos_fixed = False
obs1_pos_fixed = False
obs2_pos_fixed = False
obs3_pos_fixed = False
obs4_pos_fixed = False
target_pos_fixed = False



mid_point_x = 0
mid_point_y = 0

init_called = False
gp.grid_size= 10
gp.robot_radius = 10
gp.o_rad = 10
gp.obj_ht = 10
gp.obj_wd = 10
gp.incr_d = 10 # This is min(width, height)/25 steps



try:
        
        
        while True:
                
                        
                # grab the current frame, then handle if we are using a
                # VideoStream or VideoCapture object
                frame = vs.read()
                frame = frame[1] if args.get("video", False) else frame

                # check to see if we have reached the end of the stream
                if frame is None:
                        break

                # resize the frame (so we can process it faster)
                frame = imutils.resize(frame, width=600)
                (frm_ht, frm_wd, c) = frame.shape
                
                
                #frame = cv2.flip(frame, 0)     ##Flip vertical

                # grab the updated bounding box coordinates (if any) for each
                # object that is being tracked
                (success, boxes) = trackers.update(frame)
                        

                # loop over the bounding boxes and draw then on the frame
                # There will be 3 boxes - each for 1 robot

                
                for box in boxes:
                        #print("boxes = ",boxes)
                        #print("box = ",box)
                        (x, y, w, h) = [int(v) for v in box]
                        
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
##                        y = frm_ht - y
##                        mid_point_x = x + (w/2)
##                        mid_point_y = y + (h/2)
##                        gp.actual_x1 = mid_point_x
##                        gp.actual_y1 = mid_point_y
                        #print('x=',x+(w/2))
                        #print('y=',y+(h/2))
                        # Calculate the distance between the mid points of
                        # previous box and the current box
                        #print('prev_robot_x = ',prev_robot_x)
                        #print('prev_robot_y = ',prev_robot_y)
                        '''
                        if (prev_robot_x == 0 and prev_robot_y == 0):
                                #print("came into if")
                                prev_robot_x = mid_point_x
                                prev_robot_y = mid_point_y
                        else:
                                dist_x, dist_y = calculateDistAngle(mid_point_x,mid_point_y,prev_robot_x,prev_robot_y)
                                #print('dist x = ', dist_x)
                                #print('dist y = ', dist_y)
                                prev_robot_x = mid_point_x
                                prev_robot_y = mid_point_y
                        #time.sleep(2)
                        '''

                '''                
                if(box ==1):
                        mid_point_x = mid_point_x1
                        mid_point_y = mid_point_y1
                else if(box == 2):
                        mid_point_x = mid_point_x2
                        mid_point_y = mid_point_y2
                else if(box == 3):
                        mid_point_x = mid_point_x3
                        mid_point_y = mid_point_y3
                '''                
                # show the output frame
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF

                # if the 's' key is selected, we are going to "select" a bounding
                # box to track
                if key == ord("r"):
                        #run_started = False
                        # select the bounding box of the object we want to track (make
                        # sure you press ENTER or SPACE after selecting the ROI)
                        
                        box = cv2.selectROI("Frame", frame, fromCenter=False,
                                showCrosshair=True)
                        (x, y, w, h) = [int(v) for v in box]
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        y = frm_ht - y
                        mid_point_x = x +(w/2)
                        mid_point_y = y - (h/2)
                        gp.robot_radius = w/2
                        gp.incr_d = gp.robot_radius * 2


                        # create a new object tracker for the bounding box and add it
                        # to our multi-object tracker
                        #tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
                        tracker = OPENCV_OBJECT_TRACKERS[trkr]()
                        trackers.add(tracker, frame, box)
                        #robot1_tracker = tracker

                        #TODO
                        # set the robot positions to start

                        print("mid_point_x = ",mid_point_x)
                        print("mid_point_y = ",mid_point_y)
                        if mid_point_x != 0 and mid_point_y != 0:
                                if (not robot1_pos_fixed):
                                        print("came into robot1_pos_fixed if")
                                        gp.s1x = mid_point_x
                                        gp.s1y = mid_point_y
                                        gp.actual_x1 = gp.s1x
                                        gp.actual_y1 = gp.s1y
                                        gp.prev_x1 = gp.s1x
                                        gp.prev_y1 = gp.s1y
                                        mid_point_x = 0
                                        mid_point_y = 0
                                        robot1_pos_fixed = True
                                        logLine="first robot fixed \n"
                                        gp.logFile.write(logLine)
                                        logLine="gp.s1x = "+str(gp.s1x)+" gp.s1y = "+str(gp.s1y)+"\n"
                                        gp.logFile.write(logLine)
                        
                elif key == ord("j"): #draw boxf for objects
                        box = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)

                        # create a new object tracker for the bounding box and add it
                        # to our multi-object tracker
                        #tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
                        tracker = OPENCV_OBJECT_TRACKERS[trkr]()
                        trackers.add(tracker, frame, box)
                        (x, y, w, h) = [int(v) for v in box]
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        y = frm_ht - y
                        mid_point_x = x + (w/2)
                        mid_point_y = y - (h/2)
                        if mid_point_x != 0 and mid_point_y != 0:
                                if (not obj1_pos_fixed):
                                        print("came into obj_pos_fixed if")
                                        gp.objx[0] = mid_point_x
                                        gp.objy[0] = mid_point_y
                                        mid_point_x = 0
                                        mid_point_y = 0
                                        obj1_pos_fixed = True
                                        logLine="first obj fixed \n"
                                        gp.logFile.write(logLine)
                                        logLine="gp.objx[0]= "+str(gp.objx[0])+" gp.objy[0] = "+str(gp.objy[0])+"\n"
                                        gp.logFile.write(logLine)
                                        

                      
                        

                elif key == ord("b"): #draw for obstacles
                        box = cv2.selectROI("Frame", frame, fromCenter=False,
                                showCrosshair=True)

                        # create a new object tracker for the bounding box and add it
                        # to our multi-object tracker
                        #tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
                        tracker = OPENCV_OBJECT_TRACKERS[trkr]()
                        trackers.add(tracker, frame, box)
                        (x, y, w, h) = [int(v) for v in box]
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        y = frm_ht - y
                        mid_point_x = x + (w/2)
                        mid_point_y = y - (h/2)
                        gp.o_rad = w/2
                        if mid_point_x != 0 and mid_point_y != 0:
                                if (not obs1_pos_fixed):
                                        print("came into obs1_pos_fixed if")
                                        gp.ox[0] = mid_point_x
                                        gp.oy[0] = mid_point_y
                                        mid_point_x = 0
                                        mid_point_y = 0
                                        obs1_pos_fixed = True
                                        logLine="first obstcl fixed \n "
                                        gp.logFile.write(logLine)
                                        logLine="gp.ox[0] = "+str(gp.ox[0])+" gp.oy[0] = "+str(gp.oy[0])+"\n"
                                        gp.logFile.write(logLine)

                        if mid_point_x != 0 and mid_point_y != 0:
                                if (not obs2_pos_fixed):
                                                                       
                                        print("came into obs2_pos_fixed if")
                                        gp.ox[1] = mid_point_x
                                        gp.oy[1] = mid_point_y
                                        mid_point_x = 0
                                        mid_point_y = 0
                                        obs2_pos_fixed = True
                                        logLine="second obstcl fixed \n"
                                        gp.logFile.write(logLine)
                                        logLine="gp.ox[1] = "+str(gp.ox[1])+" gp.oy[1] = "+str(gp.oy[1])+"\n"
                                        gp.logFile.write(logLine)

                        if mid_point_x != 0 and mid_point_y != 0:
                                if (not obs3_pos_fixed):
                                        print("came into obs3_pos_fixed if")
                                        gp.ox[2] = mid_point_x
                                        gp.oy[2] = mid_point_y
                                        mid_point_x = 0
                                        mid_point_y = 0
                                        obs3_pos_fixed = True
                                        logLine="third obstcl fixed \n"
                                        gp.logFile.write(logLine)
                                        logLine="gp.ox[2] = "+str(gp.ox[2])+" gp.oy[2] = "+str(gp.oy[2])+"\n"
                                        gp.logFile.write(logLine)

                        if mid_point_x != 0 and mid_point_y != 0:
                                if (not obs4_pos_fixed):
                                        print("came into obs4_pos_fixed if")
                                        gp.ox[3] = mid_point_x
                                        gp.oy[3] = mid_point_y
                                        mid_point_x = 0
                                        mid_point_y = 0
                                        obs4_pos_fixed = True
                                        logLine="fourth obstcl fixed \n "
                                        gp.logFile.write(logLine)
                                        logLine="gp.ox[3] = "+str(gp.ox[3])+" gp.oy[3] = "+str(gp.oy[3])+"\n"
                                        gp.logFile.write(logLine)

       

                elif key == ord("g"): #draw for target
                        box = cv2.selectROI("Frame", frame, fromCenter=False,
                                showCrosshair=True)

                        # create a new object tracker for the bounding box and add it
                        # to our multi-object tracker
                        #tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
                        tracker = OPENCV_OBJECT_TRACKERS[trkr]()
                        trackers.add(tracker, frame, box)
                        (x, y, w, h) = [int(v) for v in box]
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        y = frm_ht - y
                        mid_point_x = x + (w/2)
                        mid_point_y = y - (h/2)
                        
                        if mid_point_x != 0 and mid_point_y != 0:
                                if (not target_pos_fixed):
                                        print("came into target_pos_fixed if")
                                        gp.g1x = mid_point_x
                                        gp.g1y = mid_point_y
                                        mid_point_x = 0
                                        mid_point_y = 0
                                        target_pos_fixed = True
                                        logLine="first target fixed \n "
                                        gp.logFile.write(logLine)
                                        logLine="gp.g1x = "+str(gp.g1x)+" gp.g1y = "+str(gp.g1y)+"\n"
                                        gp.logFile.write(logLine)

                # if the `q` key was pressed, break from the loop
                elif key == ord("q"):
                        break

                elif key == ord("s") or init_called:
                        print("came into start")
##                        print(robot1_pos_fixed)
##                        print(obj1_pos_fixed)
##                        print(obs1_pos_fixed)
##                        print(obs2_pos_fixed)
##                        print(obs3_pos_fixed)
##                        print(obs4_pos_fixed)
##                        print(target_pos_fixed)
##                        print("before if")
                        if robot1_pos_fixed and obj1_pos_fixed and obs1_pos_fixed and obs2_pos_fixed and obs3_pos_fixed and obs4_pos_fixed and target_pos_fixed:
                                #print("before calling init")
                                #print("gp.robot_radius = ",gp.robot_radius)

                                if not init_called:
                                        logLine="Before calling init()\n"
                                        gp.logFile.write(logLine)
                                        init()
                                        init_called = True
                                        #print("init called",init_called)
                                #pdb.set_trace()
                                robot_box = boxes[0]
                                gp.actual_x1 = robot_box[0] + robot_box[2]/2
                                gp.actual_y1 = frm_ht - robot_box[1] - robot_box[3]/2
                                print("gp.actual_x1, y1 = ",gp.actual_x1, gp.actual_y1)
                                print("gp.prev_x1, y1 = ",gp.prev_x1, gp.prev_y1)
##                                print("gp.objx[0], objy[0] = ",gp.objx[0], gp.objy[0])                        
##                                print("gp.g1x, g1y = ",gp.g1x, gp.g1y)
                                if (abs(gp.prev_x1 - gp.actual_x1) > 1 and abs(gp.prev_y1 - gp.actual_y1) > 1):
                                        _, robot_angle = calculateDistAngle(gp.prev_x1,gp.prev_y1, gp.actual_x1, gp.actual_y1)
                                        gp.robot1_orientation = robot_angle * 180/math.pi
                                        print("video trackin pgm, gp.robot1_orientation = ",gp.robot1_orientation)
                                
                                robot_plotting()
                                time.sleep(1)
                                gp.prev_x1 = gp.actual_x1
                                gp.prev_y1 = gp.actual_y1
                                


                        if gp.goal_reached_1:
                                logLine="Goal reached\n"
                                gp.logFile.write(logLine)
                                break
                          
        ##                elif (key == ord("s") or run_started):
        ##                        run_started = True
        ##                        #TODO : call robot plotting
        ##                        if robot1_pos_fixed and robot2_pos_fixed and robot3_pos_fixed:
        ##                                print("calling robot_plotting from robot_video_tracking")
        ##                                mid_point_x1, mid_point_y1, mid_point_x2, mid_point_y2, mid_point_x3, mid_point_y3= robot_plotting(mid_point_x1, mid_point_y1, mid_point_x2, mid_point_y2, mid_point_x3, mid_point_y3)
        ##                        else:
        ##                                print("fix all 3 robot positions")
        ##                elif (gp.goal_reached_1 and gp.goal_reached_2 and gp.goal_reached_3):
        ##                        print("all goals reached")
        ##                        break

        gp.logFile.close()
        #vs.release()
        #cv2.destroyAllWindows()                 

        # if we are using a webcam, release the pointer
        if not args.get("video", False):
                vs.stop()

        # otherwise, release the file pointer
        else:
                vs.stop()

        # close all windows
        cv2.destroyAllWindows()
except KeyboardInterrupt:
    print("you cancelled the operation")
    logLine="you cancelled the operation"
    gp.logFile.write(logLine)
    gp.logFile.close()
    if vs:
        vs.stop()
    cv2.destroyAllWindows()

except AssertionError:
    print("Assertion Error")
    logLine="Assertion Error"
    gp.logFile.write(logLine)
    gp.logFile.close()
    if vs:
        vs.stop()
    cv2.destroyAllWindows()
    
except:
    print("Oops!",sys.exc_info()[0],"occured.")
    logLine="Oops!"+sys.exc_info()[0]+"occured."
    gp.logFile.write(logLine)
    gp.logFile.close()
    if vs:
        vs.stop()
    cv2.destroyAllWindows()
