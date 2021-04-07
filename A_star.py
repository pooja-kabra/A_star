# -*- coding: utf-8 -*-
"""
Created on Wed Apr  7 19:01:51 2021

@author: pooja
"""
import time
import pygame
import sys
import math
import copy
from collections import deque
# from queue import PriorityQueue
import numpy as np

# Note: Input nodes after approximating only

#-----------------------------------------------------------------------------
# Optimization results
#-----------------------------------------------------------------------------
# The deque class from collections module is used as the data structure to store the nodes because it is faster compared to the python's inbuilt list.
# String is fast in comparion but searching and conversion from list to string is time consuming(experimentally verified)
# Cannot implement binay search to improve searching speeds because we have to search in an unsorted array.
    
#-----------------------------------------------------------------------------
# Node Class
#-----------------------------------------------------------------------------
class Node:
    """ This class stores the attributes(like state and parent node) and methods(like check_obstacle_space, actions, generate children etc.) 
        for the each Node in the Map.""" 
    
    #-------------------------------------------------------------------------
    
    def __init__(self, state, parent, cost_to_come, cost_to_go, cost, distance_step, theta_step_rad):
        
        self.state = state                                                     # state is the list of x,y, theta coordinates of the node
        self.parent = parent                                                   # parent attribute stores the parent node of current node
        self.cost_to_come = cost_to_come
        self.cost_to_go = cost_to_go
        self.cost = cost
        self.distance_step = distance_step
        self.theta_step_rad = theta_step_rad

    #-------------------------------------------------------------------------
    
    def __repr__(self):
        return str(self.state)                                                 # This method returns the state information of the node
    
    #-------------------------------------------------------------------------
    
    def check_obstacle_space(self, pot_node):
        """Defining obstacle space constraints using half plane equations.
           Furthermore obstcales are inflated by 15 units(radius + clearance) to incorporate the mobile robot
           IMP_NOTE: For concave obstacles divide the obstacles into smaller convex obstacles and take 'OR' between them to find the constraints.
           Example obstacle 3 and 5."""

        x, y = pot_node[0], pot_node[1]

        # Boundary condition
        if (x < 0) or (x > 400) or (y < 0) or (y > 300): 
            return True
        
        # Obstacle 1 (Circle)
        elif (x-90)**2 + (y-70)**2 - 2500 <= 0:   
            return True
        
        # Obstacle 2 (Rectangle) 
        elif (y- 150.727 + 1.4377*x >= 0) and (y - 117.176 - 0.6960*x <= 0) and (y - 466.181 + 1.4419*x <= 0) and (y - 56.308 - 0.6959*x >= 0): 
            return True
        
        # Obstacle 3 (C section)
        elif (x >= 185 and x <= 225 and y <= 295 and y >= 215) or (x >= 225 and x <= 245 and y <= 295 and y >= 255) or (x >= 225 and x <= 245 and y <=245 and y >= 215):      
            return True
        
         # Obstacle 4 (Ellipse)
        elif ((x-246)**2)/75**2 + ((y-145)**2)/45**2 - 1 <= 0:                
            return True
        
        else:
            # Node in Freespace
            return False 
        
    #-------------------------------------------------------------------------
    
    def generate_child(self, dir):
        # This method performs the up action on the current node
        # Pot node in not an instance of node class, it is just a tuple of x,y,theta values (i.e similar to state of the node)
        pot_node = (self.state[0] + distance_step*(math.cos(self.state[2]+ dir*theta_step_rad)), self.state[1] + distance_step*(math.sin(self.state[2]+ dir*theta_step_rad)), self.state[2]+ dir*theta_step_rad)
        pot_node = self.approximate_node(pot_node)
        if not self.check_obstacle_space(pot_node):
            cost_to_come = self.cost_to_come + distance_step
            cost_to_go = ((self.state[0] - goal.state[0])**2 + (self.state[1] - goal.state[1])**2)**0.5
            cost = cost_to_come + cost_to_go
            new_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.cost_to_come, self.cost_to_go, self.cost, self.distance_step, self.theta_step_rad), cost_to_come, cost_to_go, cost, self.distance_step, self.theta_step_rad)
            new_node.state[0] = pot_node[0]
            new_node.state[1] = pot_node[1]
            new_node.state[2] = pot_node[2]
            return new_node
        return None

    #-------------------------------------------------------------------------
    
    def approximate_node(self, pot_node):
        x, y, theta_rad = pot_node[0], pot_node[1], pot_node[2]
        theta_deg = (theta_rad*180)/math.pi
        dec_x, dec_y = math.modf(x)[0], math.modf(y)[0]
        
        # X approximation
        if dec_x < 0.25:
            x = math.floor(x)
        elif 0.25 <= dec_x and dec_x <= 0.5:
            x = math.floor(x) + 0.5
        elif 0.5 < dec_x and dec_x < 0.75:
            x = math.floor(x) + 0.5
        else:
            x = math.ceil(x)
        
        # Y approximation
        if dec_y < 0.25:
            y = math.floor(y)
        elif 0.25 <= dec_y and dec_y <= 0.5:
            y = math.floor(y) + 0.5
        elif 0.5 < dec_y and dec_y < 0.75:
            y = math.floor(y) + 0.5
        else:
            y = math.ceil(y)

        # theta approximation
        theta_deg_norm = theta_deg/30
        theta_dec, theta_int = math.modf(theta_deg_norm)[0], math.modf(theta_deg_norm)[1]
        print(theta_int, theta_dec)
        if theta_dec < 0.5:
            theta_deg = theta_int*30
        else:
            theta_deg = (theta_int+1)*30
            
        theta_deg = theta_deg % 360
        theta_rad = (theta_deg*math.pi)/180
        return x, y, theta_rad

    #-------------------------------------------------------------------------
    
    def goal_threshold(self):
        # This method is used to check if robot is in the goal radius, because it may not reach the exact loaction due to limited action set
        x_goal = goal.state[0]
        y_goal = goal.state[1]
        
        if (self.state[0]-x_goal)**2 + (self.state[1]-y_goal)**2 - 100 <= 0:   # 10 pixel radius
            return True
        else:
            return False
    #-------------------------------------------------------------------------
    
    def generate_children(self):
        # This method applies the actions functions to generate the children nodes of the current node 
        
        children = []
        
        for direction in [0, 1, 2, -1, -2]:
            child = self.generate_child(direction)
            if child:
                children.append(child)
                
        return children
    
    #-------------------------------------------------------------------------
    
    def find_path(self, goal_node):
        print("Shortest Path: ")
        # Backtracking the parent node to find the shortest path
        # Print sequence GOAL node to START node
        current_node = goal_node
        path = []

        while(current_node.state[0:2] != self.state[0:2]):
            path.append(current_node.state)
            current_node = current_node.parent
            print(current_node)
            
        return path

#-----------------------------------------------------------------------------
# Main
#-----------------------------------------------------------------------------

if __name__== "__main__":

    global goal

    while(1):
        
        #---------------------------------------------------------------------
        # User Input
        #---------------------------------------------------------------------
        # Start node
        x1, y1, theta_s = map(int, input("Please input the X, Y, theta(in degrees) coordinates of the start node!\n").split())
        # Goal node
        x2, y2, theta_g = map(int, input("Please input the X, Y, theta(in degrees) coordinates of the goal node!\n").split())
        # Theta
        theta_step = int(input("Please enter the theta(in degrees) between consecutive actions\n")) ## 30
        theta_step_rad = math.radians(theta_step)
        # Step size
        distance_step = int(input("Please input the step size\n"))
        
        input_node = Node([x1, y1, math.radians(theta_s)] , None, 0, ((x1 - x2)**2 + (y1 - y2)**2)**0.5, ((x1 - x2)**2 + (y1 - y2)**2)**0.5, 
                          distance_step, theta_step_rad)
        
        goal =       Node([x2, y2, math.radians(theta_g)] , None, 9999999, 0, 9999999, 
                          distance_step, theta_step_rad)
        
        # obstacle check
        if goal.check_obstacle_space(goal.state) or input_node.check_obstacle_space(input_node.state):
            print("Input Coordinates are in obstacle space!")
        else:
            break    

    # Using python's inbuilt queue from collections module, because it performs faster enqueue and dequeue operations compared to a list
    # queue = deque()
    queue = []
    queue.append(input_node)

    # checking this array for visited
    visited_states = np.zeros((601,801,12))
      
    # maintaining for visualizing exploration in pygame
    visited_states1 = []
    
    if input_node.parent != None:
        visited_states1.append([input_node.state, input_node.parent.state])
    else:
        visited_states1.append([input_node.state, None])
    
    
    t = time.time()

    # A star's Implementation
    while(1):

        queue.sort(key = lambda x: x.cost)
        current_node = queue.pop(0)
        # print(current_node, end = '\n')
        
        #---------------------------------------------------------------------
        # Goal found!
        #---------------------------------------------------------------------
        if current_node.goal_threshold():
            print("Goal Found\n")      
            shortest = input_node.find_path(current_node)
            break

        #---------------------------------------------------------------------
        # Goal not found yet, explore on
        #---------------------------------------------------------------------
        children = current_node.generate_children() 

        for child in children:
            r = int(2 * child.state[1])                                        # node coordinates even after approximation will be
            c = int(2 * child.state[0])                                        # multiples of 0.5, to use as array indices, multiply by 2
            ang = int(math.degrees(child.state[2])/30)
     
            # if child.state not in visited_states:
            if visited_states[r][c][ang] == 0:
                visited_states[r][c][ang] = 1                                  # mark visited
                visited_states1.append([child.state, child.parent.state])
                queue.append(child)
        
            else:     
                parent_node = current_node.parent     
                if current_node.cost > parent_node.cost_to_come + current_node.distance_step + current_node.cost_to_go: # new cost is less
                    current_node.cost_to_come = parent_node.cost_to_come + current_node.distance_step                   # update cost to come
                    current_node.cost = current_node.cost_to_come + current_node.cost_to_go                             # update cost
    
    print("Execution time", time.time()-t)

#______________________Pygame Animation_______________________________________ 

    print("Running pygame animation..................")
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    counter = 0
    while True:

        # Map Generation in pygame
        screen.fill((0,0,0))
        
        # Inflated obstacles
        pygame.draw.circle(screen, (255,0,0), (180, 600-140), 100)
        pygame.draw.polygon(screen, (255,0,0), ((88.50, 600-174.214), (31.45, 600-256.24),(326.48 ,600-461.586), (383.43, 600-379.468)))
        pygame.draw.polygon(screen, (255,0,0), ((370,10), (490,10), (490, 170), (370, 170)))
        pygame.draw.ellipse(screen, (255,0,0), ((342,220, 300, 180)))

        # Goal threshold
        pygame.draw.circle(screen, (0,255,0), (2*goal.state[0], 600-2*goal.state[1]), 10)
        
        #Old obstacles
        pygame.draw.circle(screen, (0,0, 255), (180, 600-140), 70)
        pygame.draw.polygon(screen, (0,0, 255), ((96, 600-216), (73.06, 600-248.76),(318.8, 600-420.832), (341.74, 600-388.072)))
        pygame.draw.polygon(screen, (0,0, 255), ((400,40),(460,40),(460,60),(420,60),(420,120), (460,120), (460, 140), (400, 140)))
        pygame.draw.ellipse(screen, (0,0, 255), ((372,250, 240, 120)))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        if counter ==0:
            
            #-----------------------------------------------------------------
            # Draw Exploration
            #-----------------------------------------------------------------
            for node in visited_states1:
                # time.sleep(0.1)
                print(node)
                child = node[0]
                parent = node[1]

                if parent != None:  
                    pygame.draw.line(screen, (255,255,255), (child[0]*2, 600-child[1]*2), (parent[0]*2, 600-parent[1]*2))      
                
                # pygame.draw.circle(screen, (255,255,255), (state[0]*2, 600-state[1]*2), 1) 
                pygame.display.update()
                    
                
            #-----------------------------------------------------------------
            # Draw Shortest Path
            #-----------------------------------------------------------------
            for state in shortest:
                pygame.draw.circle(screen, (255, 0, 0), (state[0]*2, 600-state[1]*2), 5)
            pygame.display.update()
            
        counter +=1    
