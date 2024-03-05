"""

Pattern Search based path planner and optimizer

author: Aditi
Version: 4

"""

import numpy as np
import matplotlib.pyplot as plt
from calcdistance_v4 import calculateDistAngle
from calcdistance_v4 import calc_min_dist_to_obj
from movetowardsgoal import movetowardsgoal
import global_params as gp
from robot_movement import move_robot
from comm import send_mesg
import math
import time


# Parameters
# TODO

alpha = 2 # multiplier for distance to target
beta = 3 # multiplier for angle of movement
gamma = 4 # multiplier for size of robot so that it does not hit any obstacle

##show_animation = True # will be set to false during the hardware integration with robots
##grid_size = 25  # workspace grid size 
##robot_radius = 10.0  # robot radius - size of the robot. 
##incr_d = grid_size # make incremental step equal to grid size or less for smoother movement
##o_rad = 10.0 # obstacle radius



#next_x, next_y = [0,0]

def pattern_search_path (sx, sy, gx, gy, ox, oy, grid_size, robot_radius):
    #TODO
    #call function calculateDistAngle from source to target
    distToTarget,_ = calculateDistAngle(sx, sy, gx, gy)
    ix = sx
    iy = sy
    gix = gx
    giy = gy
    o_rad = gp.o_rad
    incr_d = gp.incr_d
    
##    if show_animation:
##        #show robots as BLUE triangles
##        plt.plot(ix, iy, ".b", markersize=10)
##        old_robot_x, old_robot_y = [ix, iy]
##        #show target as GREEN X
##        plt.plot(gix, giy, "Xg", markersize=10)

    movement = get_neighbouring_points()
    current_x, current_y = [sx, sy] # robot's current coordinates at start
    next_x, next_y = [0,0] 
    #obj_func = 0
    #Calculate the initial objective function for the starting point
    dist,angle = calculateDistAngle(gix, giy, sx, sy)
    obj_func = alpha * dist + beta * angle + gamma * robot_radius
    
    
    if (distToTarget >= grid_size or (distToTarget != 0 and distToTarget >= incr_d)):
        print("distToTarget = ",distToTarget)
        for i,_ in enumerate(movement):
            #print("i = ",i)
            possible_next_x = current_x + movement[i][0]
            possible_next_y = current_y + movement[i][1]
            if (possible_next_x >=0 and possible_next_y >= 0): # if the next step is not left of x axis or below y axis. Only first quadrant
                #print(possible_next_x," ",possible_next_y)
                dist,angle = calculateDistAngle(gix, giy, possible_next_x, possible_next_y)
                func = alpha * dist + beta * angle + gamma * robot_radius
                #print("func = ",func)
                #if obj_func == 0:
                #    obj_func = starting_func
                #    next_x = possible_next_x
                #    next_y = possible_next_y

                # optimize objective function

                #print("current_x = ",current_x)
                #print("current_y = ",current_y)
                
                #print("possibe_next_x = ",possible_next_x)
                #print("possibe_next_y = ",possible_next_y)
##                print("ox[0] - rad = ",ox[0]-o_rad)
##                print("ox[0] + rad = ",ox[0]+o_rad)
##                print("oy[0] - rad = ",oy[0]-o_rad)
##                print("oy[0] + rad = ",oy[0]+o_rad)
##
##                print("ox[1] - rad = ",ox[1]-o_rad)
##                print("ox[1] + rad = ",ox[1]+o_rad)
##                print("oy[1] - rad = ",oy[1]-o_rad)
##                print("oy[1] + rad = ",oy[1]+o_rad)
##
##                print("ox[2] - rad = ",ox[2]-o_rad)
##                print("ox[2] + rad = ",ox[2]+o_rad)
##                print("oy[2] - rad = ",oy[2]-o_rad)
##                print("oy[2] + rad = ",oy[2]+o_rad)
##
##                print("ox[3] - rad = ",ox[3]-o_rad)
##                print("ox[3] + rad = ",ox[3]+o_rad)
##                print("oy[3] - rad = ",oy[3]-o_rad)
##                print("oy[3] + rad = ",oy[3]+o_rad)
##
##                print("gix = ",gix)
##                print("giy = ",giy)
##
##                print("obj_func = ", obj_func)
##                print("func = ", func)
                
                if not ((ox[0] - o_rad <= possible_next_x <= ox[0] + o_rad) and (oy[0] - o_rad <= possible_next_y <= oy[0] + o_rad)):
                    if not ((ox[1] - o_rad <= possible_next_x <= ox[1] + o_rad) and (oy[1] - o_rad <= possible_next_y <= oy[1] + o_rad)):
                        if not ((ox[2] - o_rad <= possible_next_x <= ox[2] + o_rad) and (oy[2] - o_rad <= possible_next_y <= oy[2] + o_rad)):
                            if not ((ox[3] - o_rad <= possible_next_x <= ox[3] + o_rad) and (oy[3] - o_rad <= possible_next_y <= oy[3] + o_rad)):
                                if possible_next_x == gix and possible_next_y == giy:
                                    obj_func = func # if target is reached stop
                                if func <= obj_func and (possible_next_x > current_x or possible_next_y > current_y) and (possible_next_x <= gix and possible_next_y <= giy):
                                    #best values of next point
                                    obj_func = func
                                    next_x = possible_next_x
                                    next_y = possible_next_y
                                    _,current_angle= calculateDistAngle(current_x, current_y, possible_next_x, possible_next_y)
                                    gp.robot1_angle = current_angle
##                                    if gp.robot1_obj_picked:
##                                        _,gp.robot1_angle=calculateDistAngle(current_x, current_y, gp.g1x. gp.g1y)
                                    print("gp.robot1_angle = ",gp.robot1_angle*180/math.pi)
                                    resultLine=str(gp.stepno_r1)+","+str(obj_func) + '\n'
                                    gp.resultFile.write(resultLine)
                                    gp.stepno_r1 = gp.stepno_r1 + 1
                                    

                                    #print("gix = ",gix)
                                    #print("giy = ",giy)
                
                                    #print("possible_next_x = ",possible_next_x)
                                    #print("possible_next_y = ",possible_next_y)
                                    #print(calculateDistAngle(gix, giy, possible_next_x, possible_next_y))
                                    #print("********")
                                    #current_x, current_y = [next_x, next_y] # best value of next point
                                    #distToTarget,_ = calculateDistAngle(next_x, next_y, gix, giy)
##                                    print(distToTarget)
##                                    print("obj_func = ",obj_func)
##                                    print(next_x," ",next_y)
                                    #break
            #current_x, current_y = [next_x, next_y]
        #next_x, next_y = [temp_current_x, temp_current_y]
    else:
        next_x, next_y = [current_x, current_y]
        
    if next_x == 0 and next_y == 0:
        print("next_x and next_y have become zero &&&&&&&&&&&&&!!!!!&&&&&&&&&&")
    #Raise exception
    assert next_x + next_y != 0
    
    return next_x, next_y



def get_neighbouring_points():
    # 8 points: left, right, up, down, and 4 diagonal points
    # dx, dy
    #incr_d = 0.5 - moved it above for use in other place also
    incr_d = gp.incr_d
    neighbouring_points = [[incr_d, incr_d],
                [incr_d, 0],
              [0, incr_d],
              [incr_d, -incr_d],
            [-incr_d, incr_d] ,            
              [-incr_d, 0],
             [-incr_d, -incr_d],
              [0, -incr_d]]


    return neighbouring_points

def init():
    #Robot1 - start and goal position
    #draw obstacles as RED circles
##    print("inside init")
##    print("gp.o_rad = ",gp.o_rad)
    
    obs1=plt.Circle((gp.ox[0], gp.oy[0]), gp.o_rad, color='r')
    obs2=plt.Circle((gp.ox[1], gp.oy[1]), gp.o_rad, color='r')
    obs3=plt.Circle((gp.ox[2], gp.oy[2]), gp.o_rad, color='r')
    obs4=plt.Circle((gp.ox[3], gp.oy[3]), gp.o_rad, color='r')
    plt.gcf().gca().add_artist(obs1)
    plt.gcf().gca().add_artist(obs2)
    plt.gcf().gca().add_artist(obs3)
    plt.gcf().gca().add_artist(obs4)

 
    #draw objects as blue square
    obj1=plt.Rectangle((gp.objx[0], gp.objy[0]), gp.obj_wd, gp.obj_ht, color='b')
##    obj2=plt.Rectangle((gp.objx[1], gp.objy[1]), gp.obj_wd, gp.obj_ht, color='b')
##    obj3=plt.Rectangle((gp.objx[2], gp.objy[2]), gp.obj_wd, gp.obj_ht, color='b')
    
    plt.gcf().gca().add_artist(obj1)
##    plt.gcf().gca().add_artist(obj2)
##    plt.gcf().gca().add_artist(obj3)

    # set boolean for object pciked or not
    gp.robot1_obj_picked= False
    gp.robot2_obj_picked= False
    gp.robot3_obj_picked= False

    # set boolean for goal reached or not
    gp.goal_reached_1 = False
    gp.goal_reached_2 = False
    gp.goal_reached_3 = False
    


    # Robot should first pick the objects and then move towards goal.
    # First the feasible path is to the object and then the feasible path is
    # towards the target

    # Robot will pick the nearest object
    # for Robots find teh nearest object based on the cost of total travel to objects
    logLine="Calculating min dist to any object for Robots\n"
    gp.logFile.write(logLine)
    '''
    gp.obj1x, gp.obj1y = calc_min_dist_to_obj(gp.s1x, gp.s1y, gp.objx, gp.objy)
    print("gp.obj1x, gp.obj1y = ",gp.obj1x, gp.obj1y)
    logLine="Calculating min dist to any object for Robot-2\n"
    gp.logFile.write(logLine)
    gp.obj2x, gp.obj2y = calc_min_dist_to_obj(gp.s2x, gp.s2y, gp.objx, gp.objy)
    print("gp.obj2x, gp.obj2y = ",gp.obj2x, gp.obj2y)
    logLine="Calculating min dist to any object for Robot-3\n"
    gp.logFile.write(logLine)
    gp.obj3x, gp.obj3y = calc_min_dist_to_obj(gp.s3x, gp.s3y, gp.objx, gp.objy)
    print("gp.obj3x, gp.obj3y = ",gp.obj3x, gp.obj3y)
    '''
    sx= [gp.s1x]
    sy= [gp.s1y]
    targetx, targety = calc_min_dist_to_obj(sx, sy, gp.objx, gp.objy)
    gp.obj1x, gp.obj1y = (targetx[0],targety[0])
##    gp.obj2x, gp.obj2y = (targetx[1],targety[1])
##    gp.obj3x, gp.obj3y = (targetx[2],targety[2])
    
    gp.next_x1, gp.next_y1 = [gp.s1x,gp.s1y]
    

##    print("s1x, s1y = ",gp.s1x, gp.s1y)
##    print("gp.obj1x, gp.obj1y = ",gp.obj1x, gp.obj1y)
##    print("gp.obj2x, gp.obj2y = ",gp.obj2x, gp.obj2y)
##    print("gp.obj3x, gp.obj3y = ",gp.obj3x, gp.obj3y)
    
    if gp.show_animation:
        plt.grid(True)
        plt.axis("equal")
        #show robots as BLUE circle
        logLine="Plotting initial positions in init()\n"
        gp.logFile.write(logLine)
        plt.plot(gp.s1x, gp.s1y, ".b", markersize=10)
   
        #old_robot_x, old_robot_y = [ix, iy]
        #show target as GREEN X
        plt.plot(gp.g1x, gp.g1y, "Xg", markersize=10)
        plt.pause(0.01)

##        plt.plot(gp.s1x, gp.s1y, ".b", markersize=10)
##        plt.plot(gp.s2x, gp.s2y, ".b", markersize=10)
##        plt.plot(gp.s3x, gp.s3y, ".b", markersize=10)



def robot_plotting():
    print("Robot plotting function")
    logLine="Inside robot_plotting()\n"
    gp.logFile.write(logLine)

    
    #while(1):

    # Robot first go to object to pick it
    # starting point is the start poiint of robots

    if not (gp.robot1_obj_picked):
        #curent actual values of robot position on the ground4
        if not gp.simulation:
            gp.next_x1 = gp.actual_x1
            gp.next_y1 = gp.actual_y1
        logLine="gp.next_x1, gp.next_y1 = " + str(gp.next_x1) + ", " + str(gp.next_y1) + "\n"
        gp.logFile.write(logLine)
        if (gp.next_x1 < gp.obj1x - gp.incr_d or gp.next_y1 < gp.obj1y - gp.incr_d):
            # path generation robot 1 to pick object.
            # Receive return parameters in next_x1, next_y1
            if (gp.next_x1 > gp.obj1x - 2*gp.incr_d) and not gp.claw_opened1:
                r1_mesg="mr1sx00o" # open claw
                gp.old_robot_x1, gp.old_robot_y1 = [gp.obj1x, gp.obj1y]
                gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)
                gp.claw_opened1 = True
            gp.old_robot_x1, gp.old_robot_y1 = [gp.next_x1, gp.next_y1]
            gp.next_x1,gp.next_y1 = pattern_search_path(
            gp.next_x1, gp.next_y1, gp.obj1x, gp.obj1y, gp.ox, gp.oy, gp.grid_size, gp.robot_radius)
            #Send info to arduino
            r1_mesg="mr1"
            print("robot_plotting before gp.robot1_orientation = ",gp.robot1_orientation)
            gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)
            print("robot_plotting after gp.robot1_orientation = ",gp.robot1_orientation)
            r1_mesg=""

            if gp.show_animation:
                plt.plot(gp.old_robot_x1, gp.old_robot_y1,".r", markersize=10)
                plt.plot(gp.next_x1, gp.next_y1, ".b", markersize=10)
                #plt.plot(next_x, next_y, ">b", markersize=10)
                plt.pause(0.01)
        else:
            #pick object
            #r1_mesg="mr1sx00o" # open claw
            #gp.old_robot_x1, gp.old_robot_y1 = [gp.obj1x, gp.obj1y]
            #gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)             
            r1_mesg="mr1sx00c" # close claw
            gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)
            r1_mesg="mr1sx00c" # close claw
            gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)

            r1_mesg=""
            gp.robot1_obj_picked = True

    

    
    # Robot going towards goal if object picked
    # starting point is object position
    '''
    next_x1, next_y1 = [obj1x,obj1y]
    next_x2, next_y2 = [obj2x,obj2y]
    next_x3, next_y3 = [obj3x,obj3y]
    '''
    
    if gp.robot1_obj_picked:
        if not gp.simulation:
            gp.next_x1 = gp.actual_x1
            gp.next_y1 = gp.actual_y1
        logLine="gp.next_x1, gp.next_y1 = " + str(gp.next_x1) + ", " + str(gp.next_y1) + "\n"
        gp.logFile.write(logLine)
        if (gp.next_x1 < gp.g1x - gp.incr_d or gp.next_y1 < gp.g1y - gp.incr_d):
            # path generation robot 1.
            # Receive return parameters in next_x1, next_y1
            gp.old_robot_x1, gp.old_robot_y1 = [gp.next_x1, gp.next_y1]
            gp.next_x1,gp.next_y1 = pattern_search_path(
            gp.next_x1, gp.next_y1, gp.g1x, gp.g1y, gp.ox, gp.oy, gp.grid_size, gp.robot_radius)
            #Send info to arduino
            r1_mesg="mr1"
            print("robot_plotting before gp.robot1_orientation = ",gp.robot1_orientation)
            gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)
            print("robot_plotting after gp.robot1_orientation = ",gp.robot1_orientation)
            r1_mesg=""
            if gp.show_animation:
                plt.plot(gp.old_robot_x1, gp.old_robot_y1,".r", markersize=10)
                plt.plot(gp.next_x1, gp.next_y1, ".b", markersize=10)
                #plt.plot(next_x, next_y, ">b", markersize=10)
                plt.pause(0.01)
        else:
            if not gp.goal_reached_1:
                #drop object
                r1_mesg="mr1sx00o" # open claw
                gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)
                r1_mesg="mr1sx00o" # open claw
                gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)             

                # move back and turn left to move away
                r1_mesg="mr1fb00x"
                gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)
                r1_mesg="mr1sb00x"
                gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)

                #r1_mesg="mr1sl90x"
                #gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)
                r1_mesg="mr1sx00c"
                gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)
                #r1_mesg="mr1sf00x"
                #gp.robot1_orientation = move_robot(r1_mesg,gp.old_robot_x1, gp.old_robot_y1,gp.next_x1, gp.next_y1,gp.robot1_orientation)
                r1_mesg=""
                gp.goal_reached_1 = True
                print("goal reached !!!!")




    
    

if __name__ == '__main__':
    print(__file__ + " start!!")
    gp.simulation = True
    init()
    gp.timeFile.write(time.ctime())
    gp.timeFile.write('\n')
    startTime=time.time()
    while not gp.goal_reached_1:
        robot_plotting()
    print("came out of while loop after goal reached")
    if gp.show_animation:
        #plt.show()
        plt.savefig("sim1.png")
        plt.close()
    print("closing all files")
    gp.logFile.close()
    gp.timeFile.write(time.ctime())
    gp.timeFile.write('\n')
    endTime=time.time()
    elapsedTime = endTime-startTime
    gp.timeFile.write(str(elapsedTime))
    gp.resultFile.close()
    gp.timeFile.close()
    print(__file__ + " Done!!")
