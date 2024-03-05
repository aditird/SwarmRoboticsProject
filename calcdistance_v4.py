import math
import global_params as gp

def calculateDistAngle(x1,y1,x2,y2):  
     dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
     if (x2 - x1) != 0:
          rad = math.atan((y2 - y1)/(x2 - x1))
     else:
          rad = math.pi / 2

     if (x2 - x1) < 0 and (y2 - y1) > 0:
          rad = math.pi + rad

     if (x2 - x1) < 0 and (y2 - y1) < 0:
          rad = -1 * (math.pi -1 * rad)

     theta = rad * 180 / math.pi
     return dist, rad

def calc_min_dist_to_obj(sx, sy, objx, objy):
     # Robot will pick the nearest object
     # for Robot 1 find teh distances to all objects
     #print(sx[0])
     #print(sy[0])

     rows, cols = (3, 3) 
     dist_obj = [[0 for i in range(cols)] for j in range(rows)]
     
     print("object coordinates")
     print(objx[0],objy[0])
     print(objx[1],objy[1])
     print(objx[2],objy[2])

     print("robot start coordinates")
     print(sx[0], sy[0])
     
     dist_obj[0][0],_ = calculateDistAngle(sx[0], sy[0], objx[0], objy[0])
##     dist_obj[0][1],_ = calculateDistAngle(sx[0], sy[0], objx[1], objy[1])
##     dist_obj[0][2],_ = calculateDistAngle(sx[0], sy[0], objx[2], objy[2])

     #cost = sum of length of paths for all robots to all objects
     total_dist_to_objects = [0 for i in range(3)] #initialize

    
     total_dist_to_objects[0] = dist_obj[0][0]
##     total_dist_to_objects[1] = dist_obj[0][1]
##     total_dist_to_objects[2] = dist_obj[0][2]

     print("total dist to objects")
     print(total_dist_to_objects[0])
##     print(total_dist_to_objects[1])
##     print(total_dist_to_objects[2])
     
     #list_of_path = [total_dist_to_objects[0],total_dist_to_objects[1],total_dist_to_objects[2]]
     list_of_path = [total_dist_to_objects[0]]
     least_cost_path_index = list_of_path.index(min(list_of_path))

     targetx = [0 for i in range(3)] #initialize
     targety = [0 for i in range(3)] #initialize
     
     if least_cost_path_index == 0:
          targetx[0], targety[0] = (objx[0], objy[0])
##          targetx[1], targety[1] = (objx[1], objy[1])
##          targetx[2], targety[2] = (objx[2], objy[2])
##     elif least_cost_path_index == 1:
##          targetx[0], targety[0] = (objx[1], objy[1])
####          targetx[1], targety[1] = (objx[1], objy[1])
####          targetx[2], targety[2] = (objx[2], objy[2])
##     else: # least_cost_path_index == 2:
##          targetx[0], targety[0] = (objx[2], objy[2])
####          targetx[1], targety[1] = (objx[1], objy[1])
####          targetx[2], targety[2] = (objx[0], objy[0])
####     else:
####          targetx[0], targety[0] = (objx[1], objy[1])
####          targetx[1], targety[1] = (objx[0], objy[0])
####          targetx[2], targety[2] = (objx[2], objy[2])
##
##     targetx[0], targety[0] = (objx[0], objy[0])
     '''     

          
     
     if dist_obj1 <= dist_obj2:
          if dist_obj1 <= dist_obj3:
               if not gp.obj1_is_selected_to_pick:
                    targetx = objx[0]
                    targety = objy[0]
                    gp.obj1_is_selected_to_pick = True
                    print("gp.obj1_is_selected_to_pick = ",gp.obj1_is_selected_to_pick)
          elif dist_obj2 <= dist_obj3:
               if not gp.obj2_is_selected_to_pick:
                    targetx = objx[1]
                    targety = objy[1]
                    gp.obj2_is_selected_to_pick = True
                    print("gp.obj2_is_selected_to_pick = ",gp.obj2_is_selected_to_pick)
          else:
               if not gp.obj3_is_selected_to_pick:     
                    targetx = objx[2]
                    targety = objy[2]
                    gp.obj3_is_selected_to_pick = True
                    print("gp.obj3_is_selected_to_pick = ",gp.obj3_is_selected_to_pick)        
     else:
        if dist_obj2 <= dist_obj3:
             if not gp.obj2_is_selected_to_pick:
                 targetx = objx[1]
                 targety = objy[1]
                 gp.obj2_is_selected_to_pick = True
                 print("gp.obj2_is_selected_to_pick = ",gp.obj2_is_selected_to_pick)
        else:
             if not gp.obj3_is_selected_to_pick:
                 targetx = objx[2]
                 targety = objy[2]
                 gp.obj3_is_selected_to_pick = True
                 print("gp.obj3_is_selected_to_pick = ",gp.obj3_is_selected_to_pick)
     '''
     
     return targetx, targety

