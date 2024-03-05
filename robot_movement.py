import global_params as gp
import math
from comm import send_mesg

def move_robot(inmesg, old_x, old_y, next_x, next_y, current_orientation):
    new_orientation = current_orientation
    angle_to_target = gp.robot1_angle * 180/math.pi
    print("angle_to_target = ",angle_to_target)
    print("current_orientation = ", current_orientation)
    if len(inmesg) == 8:
        send_mesg(inmesg)
    else:
        robot_angle = int(angle_to_target - current_orientation)
        
        if robot_angle > 0:
            #construct the string
            #Turn left
            if abs(robot_angle) > 100:
                mesg = inmesg+"ml"+str(abs(99)).zfill(2)+"x"
                send_mesg(mesg)
                mesg = inmesg+"ml"+str(abs(robot_angle)-99).zfill(2)+"x"
                send_mesg(mesg)
            else:
                mesg = inmesg+"ml"+str(abs(robot_angle)).zfill(2)+"x"
                send_mesg(mesg)
                
            
            #move forward
            # TODO : By how much to move forward
            #dont move forward after turning. Let it be a different action
    
            mesg = inmesg + "mf00x"
            send_mesg(mesg)
##                mesg = inmesg+"sl45x"
##                send_mesg(mesg)

            new_orientation = int(angle_to_target)
        elif robot_angle < 0:
            #Turn right
            if abs(robot_angle) > 100:
                mesg = inmesg+"mr"+str(abs(99)).zfill(2)+"x"
                send_mesg(mesg)
                mesg = inmesg+"mr"+str(abs(robot_angle)-99).zfill(2)+"x"
                send_mesg(mesg)
            else:
                mesg = inmesg+"mr"+str(abs(robot_angle)).zfill(2)+"x"
                send_mesg(mesg)
            
            # move forward
            # TODO : By how much to move forward
            #dont move forward after turning. Let it be a different action
            mesg = inmesg + "mf00x"
            send_mesg(mesg)
##                mesg = inmesg+"sl45x"
##                send_mesg(mesg)
            new_orientation = int(angle_to_target)

        else:
            # move forward
            # TODO : By how much to move forward
            
            mesg = inmesg + "ff00x"
            send_mesg(mesg)
##                mesg = inmesg+"sl45x"
##                send_mesg(mesg)
            new_orientation = int(angle_to_target)


    return new_orientation

    
