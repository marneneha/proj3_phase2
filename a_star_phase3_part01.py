#Import important libraries 
import time
import copy
import cv2
# uncomment below line if running in colab
from google.colab.patches import cv2_imshow
import heapq as hq
import numpy as np
import math

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# function to draw the obstacles on a canvas map 
def obstacles_map(canvas):
    # rectangle 1 obstacle with the given dimensions, the thickness of 5mm is considered inwards
    cv2.fillPoly(canvas, pts = [np.array([(100,150), (150,150), (150,250), (100,250)])], color = (255, 0, 0))
    # rectangle 2 obstacle with the given dimensions, the thickness of 5mm is considered inwards
    cv2.fillPoly(canvas, pts = [np.array([(100,0), (150,0), (150,100), (100,100)])], color = (255, 0, 0))
    # heaxgon obstacle with the given dimensions, the thickness of 5mm is considered inwards
    cv2.fillPoly(canvas, pts = [np.array([(235,87),(300,50),(365,87),(365,162),(300,200),(235,162)])], color = (255, 0, 0))
    # traingle obstacle with the given dimensions, the thickness of 5mm is considered inwards
    cv2.fillPoly(canvas, pts = [np.array([(460,225), (510,125), (460,25)])], color = (255, 0, 0))
    # drawing the wall
    canvas[0:10,:]=(255,0,0)
    canvas[:,0:10]=(255,0,0)
    canvas[240:250,:]=(255,0,0)
    canvas[:,590:600]=(255,0,0)
    # convert to grayscale
    gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    # edge detection
    gray=np.uint8(gray)
    edged = cv2.Canny(gray, 200, 200, L2gradient =True)
    # cv2.imshow('edged',edged)
    # find and draw contours
    contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(canvas, contours, -1, (0, 0, 255), 3)
    # cv2.imshow('canvas', canvas)
    # cv2.waitKey(0)
    return canvas

# TO DO: Add 'offset' that would be including the robot radius and clearance, you can use your previous function as well


def coord_input(canvas, manual_input):
    if(manual_input):
    # initialize empty lists
        start_position = []
        goal_position = [] 
        # Get X and Y coordinates for the start node/position
        while True:
            state = input("Enter the X Coordinate of Start position: ")
            try:
                x = int(state)
            except ValueError:
                # if the coordinate entered is not integer, ask again
                print(bcolors.WARNING+"Dude! enter a valid integer"+bcolors.ENDC)
                continue
            if not (0 <= x < canvas.shape[1]):
                # to check if the coordinate value entered is greater than the canvas dimensions
                print(bcolors.WARNING+"Hey! the X value you entered is not in the map. Will you enter something between 0 and 600"+bcolors.ENDC)
                continue
            state = input("Enter the Y Coordinate of Start position: ")
            try:
                y = int(state)
            except ValueError:
            # if the coordinate entered is not integer, ask again
                print(bcolors.WARNING+"Dude! enter a valid integer"+bcolors.ENDC)
                continue
            if not (0 <= y < canvas.shape[0]):
            # to check if the coordinate value entered is greater than the canvas dimensions
                print(bcolors.WARNING+"Hey! the Y value you entered is not in the map. Will you enter something between 0 and 250"+bcolors.ENDC)
                continue   
            # (a & b).all()     
            if((canvas[canvas.shape[0]-1-y][x] == [255, 0, 0]).all() or 0<=x<5 or 0<=y<5 or (canvas[canvas.shape[0]-1-y][x] == [0, 0, 255]).all()):
            # to check if the entered coordinates of x and y lie inside the obstacle space
                print(bcolors.WARNING+"The entered start position is in the obstacle space, enter again"+bcolors.ENDC)
                continue      
            start_position = [x, y]
            break   
        # angle for start position
        while True:
            start_position_angle = input("Please enter the angle for start position: ")
            
            try:
                angle = int(start_position_angle)
                if angle < 0:
                  angle = angle % 360
                  angle = 360 + angle
                else:
                  angle = angle % 360
            except ValueError:
                print(bcolors.WARNING+"Hey! currently we only accept angle as integers"+bcolors.ENDC)
                continue
            start_position.append(angle)
            break
            
        # Get X and Y coordinates for the final state
        while True:
            state = input("Enter the X Coordinate of Goal position: ")
            try:
                x = int(state)
            except ValueError:
            # if the coordinate entered is not integer, ask again
                print(bcolors.WARNING+"Dude! enter a valid integer"+bcolors.ENDC)
                continue
            if not (0 <= x < canvas.shape[1]):
            # to check if the coordinate value entered is greater than the canvas dimensions
                print(bcolors.WARNING+"X Coordinate is out of bounds"+bcolors.ENDC)
                continue     
            state = input("Enter the Y Coordinate of Goal position: ")
            try:
                y = int(state)
            except ValueError:
            # if the coordinate entered is not integer, ask again
                print(bcolors.WARNING+"Dude! enter a valid integer"+bcolors.ENDC)
                continue
            if not (0 <= y < canvas.shape[0]):
                # to check if the coordinate value entered is greater than the canvas dimensions
                print(bcolors.WARNING+"Y Coordinate is out of bounds"+bcolors.ENDC)
                continue    
            if((canvas[canvas.shape[0]-1-y][x] == [255, 0, 0]).all() or 0<=x<5 or 0<=y<5 or (canvas[canvas.shape[0]-1-y][x] == [0, 0, 255]).all()):
            # to check if the entered coordinates of x and y lie inside the obstacle space
                print(bcolors.WARNING+"The entered goal node is in the obstacle space, enter again"+bcolors.ENDC)
                continue
            goal_position = [x, y]
            break  

    else:
        start_position = [50, 50, 0]
        goal_position = [550, 20]
        # step_size = 5
    return start_position, goal_position

def get_robot_radius_clearance(manual_input):
    robot_radius = 0
    if(manual_input):
        while True:
            try:
                clearance = int(input("Enter the robot's clearance: "))
                if clearance < 0:
                    print("Invalid, please try again!")
                else:
                    break
            except ValueError:
                print("Enter interger value instead!")
        while True:
            try:
                left_wheel = int(input("Enter the robot's left wheel velocity (in RPM): "))
                if left_wheel < 0:
                    print("Invalid, please try again!")
                else:
                    
                    break
            except ValueError:
                print("Enter interger value instead!")
        while True:
            try:
                right_wheel = int(input("Enter the robot's right wheel velocity (in RPM): "))
                if right_wheel < 0:
                    print("Invalid, please try again!")
                else:
                    
                    break
            except ValueError:
                print("Enter interger value instead!")
    else:
        clearance = 5  
        left_wheel = 2
        right_wheel = 4
    return robot_radius, clearance, left_wheel, right_wheel


def obstacle_checkpoint(next_node_width, next_node_height, canvas):
    # To check if the node is obstructed by the obstacle or the canvas boundaries
    if next_node_height >= canvas.shape[0] or next_node_width >= canvas.shape[1]:
        return False
    if canvas[math.floor(next_node_height)][math.floor(next_node_width)][0]==255 and canvas[math.floor(next_node_height)][math.floor(next_node_width)][2]==255:
        return False
    return True

def cost_to_goal(node, final):
    # Cost To Goal between present node and goal nodes using a Euclidean distance.
    x1, y1, _ = node
    x2, y2, _ = final
    euclidean_dist = np.sqrt((x1-x2)**2 + (y1-y2)**2)
    return euclidean_dist

def goal_node_check(node, final):
    # returns bool, false if present node in not the goal node, checks within the threshold distance of 1.5
    return np.linalg.norm(np.array(node[:2]) - np.array(final[:2])) < 1.5 and node[2] == final[2]

# heurastic function considers the euclidean distance
def h_cost_calc(node, goal_node):
    # print(node)
    # print(goal_node)
    x1,y1=node[0:2]
    x2,y2=goal_node[0:2]
    # print(x1, y1, x2, y2)
    dist = math.sqrt((x1-x2)**2+(y1-y2)**2)
    return dist

# the action function to convert the x,y acccording to the velocities and R and L     
def action(node, parent_node_g_cost, canvas, rpm1, rpm2, R, L):
    x_initial, y_initial, theta_initial = node
    dt = 0.1
    for i in np.arange(0, 0.5, dt):
      x = x_initial + (0.5*R*(rpm1+rpm2)*np.cos(np.deg2rad(theta_init)))*dt
      y = y_initial + (0.5*R*(rpm1+rpm2)*np.sin(np.deg2rad(theta_init)))*dt
      theta = theta_initial + np.rad2deg(((R/L)*(rpm2 - rpm1))*dt)
      g_cost = parent_node_g_cost + math.sqrt((x-x_init)**2+(y-y_init)**2)
    # cv2_imshow(canvas)    
    # print("x",x)
    # print("y",y)
    if not (0 <= x < canvas.shape[1] and 0 <= y < canvas.shape[0] and not obstacle_checkpoint(x, y, canvas)):
        return False, node, parent_node_g_cost, False

    new_node = (x, y, theta)
    # print(canvas)
    # cv2_imshow(canvas)
    return True, new_node, g_cost, False

def astar(start_position, final_position, canvas, rpm1, rpm2, R, L):
  # TO FILL
    output_video = cv2.VideoWriter('Proj3_neha_anukriti.avi', cv2.VideoWriter_fourcc(*'XVID'), 800, (canvas.shape[1], canvas.shape[0])) 
    print(bcolors.OKBLUE+"You entered all the values in input space. Hold on your seats! we are calculating path for you"+bcolors.ENDC)
    h_cost = (h_cost_calc(start_position, final_position))
    g_cost = 0
    f_cost = (h_cost+g_cost)
    # f_cost g_cost h_cost node parent_node
    open_list = []
    # open_list = [f_cost, h_cost, g_cost, start_position, start_position]#heap data structure of tuples 
    closed_list = {}#dict
    print("final position in astar", final_position)
    hq.heapify(open_list)
    hq.heappush(open_list, [f_cost, h_cost, g_cost, tuple(start_position), tuple(start_position)])

    glb_itr = 0
    prev_h_cost = float('inf')
    while(len(open_list)):
        glb_itr += 1
        # if glb_itr > 900:
        #   break
        # pop new node
        new_closed_list_element = hq.heappop(open_list)
        # print(new_closed_list_element)
        closed_list[tuple(new_closed_list_element[3])]=tuple(new_closed_list_element[4])
        # print(closed_list)
        parent_node = new_closed_list_element[3]
        parent_node_g_cost = new_closed_list_element[2]
        # add node in the closed list
        # print("final position in line 361", parent_node)
        # print("final position in line 361", final_position)
        # print("conditional value", h_cost_calc(parent_node, final_position))
        curr_h_cost = h_cost_calc(parent_node, final_position)
        if curr_h_cost <= ((R/2)*rpm2*0.1):
            final_parent_node = parent_node
            print(bcolors.FAIL+"Goal node has been reachead going to backtrack the path"+bcolors.ENDC)
            back_track(start_position, final_parent_node, output_video, closed_list, canvas)
            break
        prev_h_cost = curr_h_cost 
        # add visited node
        # print(bcolors.WARNING + "inside while"+ bcolors.ENDC)
        for obstacle_check, node, g_cost, visibility_check in [
        # action(parent_node, canvas, rpm1, rpm1, R, L),
        action(parent_node, parent_node_g_cost, canvas, 0, rpm1, R, L),
        action(parent_node, parent_node_g_cost, canvas, rpm1, 0, R, L),
        action(parent_node, parent_node_g_cost, canvas, rpm1, rpm1, R, L),
        action(parent_node, parent_node_g_cost, canvas, 0, rpm2, R, L),
        action(parent_node, parent_node_g_cost, canvas, rpm2, 0, R, L),
        action(parent_node, parent_node_g_cost, canvas, rpm2, rpm2, R, L),
        action(parent_node, parent_node_g_cost, canvas, rpm1, rpm2, R, L),
        action(parent_node, parent_node_g_cost, canvas, rpm2, rpm1, R, L)
        ]:
            # check if node is in obstacle spaxe
            # check if it is in canvas
            # print(bcolors.OKCYAN+"inside for"+ bcolors.ENDC)

            # print('Checking action')
            # print(obstacle_check, node, g_cost, visibility_check)
            if obstacle_check:
                # check if it is in closed list
                if(not closed_list.__contains__(node)):
                # if it is in OL then update the value
                # if it is not in OL update the OL                 
                    # print("node", node)
                    h_cost = (h_cost_calc(node, final_position))
                    # print("h_cost", h_cost)
                    # print("g_cost", g_cost)
                    # g_cost = (h_cost_calc(node, start_position))
                    f_cost = h_cost+g_cost
                    # print("f_cost", f_cost)
                    new_open_list_element = [f_cost, h_cost, g_cost, node, parent_node]
                    # print(bcolors.BOLD+"before node addition openlist was"+str(open_list)+bcolors.ENDC)
                    hq.heappush(open_list, new_open_list_element)
                    hq.heapify(open_list)
                    # print(bcolors.BOLD+"after node addition openlist was"+str(open_list)+bcolors.ENDC)
                    cv2.circle(canvas,(int(node[0]),int(node[1])),2,(0,255,0),-1)
                    output_video.write(canvas)
                    # cv2.imshow('canvas', canvas)
                    # cv2.waitKey(0)
        # print("closed_list", closed_list)

def back_track(start_position, final_parent_node, output_video, closed_list, canvas):
    # print(bcolors.FAIL+"closted list is"+str(closed_list)+bcolors.ENDC)
    child_node = final_parent_node
    parent_node = closed_list[child_node]
    # print("I AM HEREEEEE")
    # print(child_node)
    # print("final parent node in back track", parent_node)
    # parent_node = final_parent_node
    # child_node = closed_list[final_parent_node]
    # cv2.circle(canvas,(int(parent_node[0]),int(parent_node[1])),2,(255,255,255),-1)
    print("child_node", child_node)
    print("parent_node", parent_node)
    while(child_node != parent_node):
        # print ("i am in while condition now")
        cv2.circle(canvas,(int(child_node[0]),int(child_node[1])),2,(0,120,120),-1)
        # cv2.imshow('canvas',canvas)
        cv2_imshow(canvas)
        # cv2.waitKey(0)
        child_node = parent_node
        parent_node = closed_list[child_node]
        # time.sleep(1)
        output_video.write(canvas)
    # for k in closed_list:
    #     canvas[k[1], k[0]] = [255] * 3
    # cv2.imshow(canvas)
    output_video.release()

  # TO FILL

if __name__ == '__main__':
    
    canvas = 255*np.ones((250,600,3), dtype="uint8")    # Creating a blank canvas/map
    print(bcolors.OKCYAN+"Thanks for running the code It was written by Neha and anukriti"+bcolors.ENDC)
    # radiaus and clearnce input
    robot_radius, clearance, rpm1, rpm2 = get_robot_radius_clearance(manual_input=True)
    # load map
    canvas = obstacles_map(canvas)
    # start and goal node input with angles in terms of tuple
    start_position, final_position  = coord_input(canvas, manual_input=True) 
    # Changing the input Cartesian Coordinates of the Map to Image Coordinates:15
    # print("final_position", final_position)
    start_position[1] = canvas.shape[0]-1 - start_position[1]
    final_position[1] = canvas.shape[0]-1 - final_position[1]
    # print(initial_state, final_state)
    # print("start_position[1]", start_position[1])
    # print("final_position", final_position)
    # print("final_position[1]", final_position[1])
    # Converting to image coordinates
    # if start_position[2] != 0:
        # start_position[2] = 360 - start_position[2]
    # if final_position[2] != 0:
        # final_position[2] = 360 - final_position[2]

    # print("\nCoordinates  of start position: ", start_position)
    # print("Coordinates of goal position: ", final_position)
    start_time = time.time()
    cv2.circle(canvas,(int(start_position[0]),int(start_position[1])),2,(128,0,128),-1)
    cv2.circle(canvas,(int(final_position[0]),int(final_position[1])),2,(128,0,128),-1)
    # call the main astar algo   
    visited_node = []
    R = 38
    L = 3.54
    print("the final positions", start_position, final_position )
    astar(start_position,final_position,canvas, rpm1, rpm2, R, L)
    # total time taken    
    end_time = time.time()  
    print(bcolors.OKGREEN+"\nThe path was calculated in freaking ", end_time-start_time, "seconds"+bcolors.ENDC)
    cv2.imshow("A* Path Visualization", canvas)
    # cv2_imshow(canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
