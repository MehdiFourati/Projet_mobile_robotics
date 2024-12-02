import math 
import vision
import numpy as np


class Robot:
    def _init_(self):
        self.center_x = None
        self.center_y = None
        self.alpha = None
        self.robot_width = None
        self.front_prox = [0,0,0,0,0,0,0]  # Values front prox.horizontal

    def update_coordinates(self, center_x, center_y, alpha, robot_width):
        self.center_x = center_x
        self.center_y = center_y
        self.alpha = alpha
        self.robot_width = robot_width
        
    def update_front_prox(self, prox_values):
        self.front_prox = prox_values

def obstacle_dictionnary(obstacles):                                             
    """Returns a dictionnary with key : obstacle, value : coordinates"""
    obstacle_dict = {}
    for i, points_list in enumerate(obstacles):
            obstacle_dict[f'obstacle_{i+1}'] = points_list
    return obstacle_dict

def get_robot_coordinates(frame, robot):                    # to be removed 

    center_x, center_y, alpha, robot_width = vision.get_starting_position(frame)
    
    robot.update_coordinates(center_x, center_y, alpha, robot_width)

    return center_x,center_y

def get_goal_coordinates(frame, goal):                       # to be removed 

    center_x, center_y = vision.get_objective(frame)
    
    goal.update_coordinates(center_x, center_y)

    return goal

def calculate_distance(point1, point2):
    """Compute the euclidean distance between two given points"""

    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def naming_points(object_corners, robot, goal):
    """Returns a dictionary with key : point names, value : coordinates"""  
    
    point_list_named = {}
    i = 0

    for _, points_list in object_corners.items():
        for point in points_list:
            point_list_named[f'P{i}'] = point
            i += 1

    point_list_named['R']= robot.center_x, robot.center_y
    point_list_named['G']= goal

    return point_list_named

def InsideLine(point1, point2, point3): 
    """If the 3 given points are colinear, return true if the 3rd point lies in the segment of the first two points"""
    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3
    if ((x3 <= max(x1, x2)) and (x3 >= min(x1, y2)) and (y3 <= max(y1, y2)) and (y3 >= min(y1, y2))): 
        return True
    return False

def orientation(point1,point2,point3):
    """Defines the orientation of the three given points"""
    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3

    orientation = (x3-x2)*(y2-y1)-(x2-x1)*(y3-y2) # got it from this site : https://www.geeksforgeeks.org/orientation-3-ordered-points/

    # Check if the orientation is respectively collinear, clockwise, or counterclockwise. To be used later for intersections.
    if orientation == 0:
        return 0  
    elif orientation > 0:
        return -1
    else:
        return 1 

def isIntersection(point1,point2,point3,point4):
    """Function that checks if two line intersect with each other"""
    
    o1=orientation(point1,point2,point3)
    o2=orientation(point1,point2,point4)
    o3=orientation(point3,point4,point1)
    o4=orientation(point3,point4,point2)

    #If two points are equal, we do not consider them as intersecting since it is the case where 
    #the points belongs to the same object, and this case is taken care of later.
    if point1==point3 or point1==point4 or point2==point3 or point2==point4:
        return False
    
    # If o1 and o2 are different and o3 and o4 are different, then the points intersect
    if o1 != o2 and o3 != o4:
        return True
    
    # Checking colinearity in every case + if the third point is inside the segment
    if ((o1 == 0) and InsideLine(point1, point2, point3)): 
        return True
    if ((o2 == 0) and InsideLine(point1, point2, point4)): 
        return True
    if ((o3 == 0) and InsideLine(point3, point4, point1)): 
        return True
    if ((o4 == 0) and InsideLine(point3, point4, point2)): 
        return True
    
    return False 

def obstacle_points_named(object_corners):
    """Returns a dictionnary with key : ostacle names, value : point names """
    obstacle_names = {}  
    i = 0 
    for obstacle_id, points in object_corners.items():
        obstacle_names[obstacle_id] = []
        for _ in points:
            obstacle_names[obstacle_id].append(f'P{i}')
            i += 1

    return obstacle_names

def Point_connection(point1, point2,obstacle_corners,Robot,goal):
    """Returns true if two points are connected by a straight line"""
    
    points_named = naming_points(obstacle_corners,Robot,goal)
    obstacle_points = obstacle_points_named(obstacle_corners)


    coordinates = {v: k for k, v in points_named.items()}       #New dictionary inverting the keys and values of points_names
    

    # now let's get the names of the given points

    P1= coordinates[point1] 
    P2= coordinates[point2] 

    for _, point_list in obstacle_points.items():
        if P1 in point_list and P2 in point_list:
            #If the points are adjacent, they are connected
            if abs(point_list.index(P1)-point_list.index(P2))==1:
                return True
            #If the points are adjacent but one is the first item and the other is the last item in the list
            if point_list.index(P1)==len(point_list)-1 and point_list.index(P2)==0:
                return True
            if point_list.index(P2)==len(point_list)-1 and point_list.index(P1)==0:
                return True
            #If the points are in same object but are not adjacent, they are not connected
            return False

    #We go through all objects and check if the line between the two given point  
    #intersects with any other line between two other points from the same object
    for _, point_list in obstacle_corners.items():
        for i in range(len(point_list)):
            if i == (len(point_list) - 1): # the last point of a an obstacle
                if isIntersection(point1, point2, point_list[i], point_list[0]):
                    return False
            else:
                #Ignore the points if they are equal to point1 or point2
                if point_list[i] == point1 or point_list[i+1] == point2:                    #always used in order
                    continue
                else:
                    if isIntersection(point1, point2, point_list[i], point_list[i + 1]):
                        return False
                    
    if (P1 == 'R' or P2 == 'R'):
        #same as above
        for _, point_list in obstacle_corners.items():
            for i in range(len(point_list)):
                if i == (len(point_list) - 1):
                    if isIntersection(point1, point2, point_list[i], point_list[0]): 
                        return False
                else:
                    if point_list[i] == point1 or point_list[i+1] == point2:         
                        continue
                    else:
                        if isIntersection(point1, point2,point_list[i], point_list[i + 1]):
                            return False
    return True


def creating_adjacency_dictionnary(object_corners,robot,goal):
    """Returns a dictionnary with key: point names, values: point names that are connected to the key"""
    points_named = naming_points(object_corners,robot,goal)
    adjacency_dict = {}

    for P1, point1 in points_named.items():
        adjacency_dict[P1] = []
        for P2, point2 in points_named.items():
            if P1 != P2:
                if Point_connection(point1, point2, object_corners,robot,goal):
                    adjacency_dict[P1].append(P2)

    return adjacency_dict

def calculating_distances(adjacency_dict, points_named):
    """Returns a dictionnary with key: two connected points, value: distance between them"""
    distances = {}  

    for point, connected_points in adjacency_dict.items():
        for i in connected_points:
        
            dist = calculate_distance(points_named[point], points_named[i])
    
            distances[(point, i)] = dist
            distances[(i, point)] = dist
    
    return distances

def get_dist(distances, point1, point2):
    """Finds the distance stored in the above dictionnary for two given points"""
    for node, dist in distances.items():
        if node[0] == point1 and node[1] == point2:
            return dist
        
def dijkstra_algo(adjacency_dict, points_named):

    shortest_dist = {} 
    previous_nodes = {} 
    unvisited = list(points_named.keys())
    distances = calculating_distances(adjacency_dict, points_named)

    # We need to set every distance to infinity(~ unreachable at the start). We will simulate that using a very large value     
    for node in points_named.keys():
        shortest_dist[node] = math.inf
    shortest_dist['R'] = 0                # setting the start to zero

    while unvisited:
        current = unvisited[0]
        for node in unvisited:
            if shortest_dist[node] < shortest_dist[current]:
                current = node
        neighbors = adjacency_dict[current]
        for i in neighbors:
            test = shortest_dist[current] + get_dist(distances,current, i)
            if test < shortest_dist[i]:
                shortest_dist[i] = test
                previous_nodes[i] = current
        unvisited.remove(current)

    return previous_nodes


def finding_path(adjacency_list, point_named):
    """Finds shortest_path using Dijkstra algorithm and returns a list containing the points names of the path 
       from the robot to the goal"""
    previous_nodes = dijkstra_algo(adjacency_list, point_named)
    shortest_path = ['G']
    current_node = 'G'
    while current_node != 'R':
        current_node = previous_nodes[current_node]
        shortest_path.append(current_node)
    shortest_path.reverse()
    return shortest_path


def global_plan(raw_obstacle_corners,robot,goal):

    obstacle_corners = [[tuple(arr) for arr in sublist] for sublist in raw_obstacle_corners]   # converting in the right format
    obstacle_corners = obstacle_dictionnary(obstacle_corners)
    print(obstacle_corners)
    adj_list = creating_adjacency_dictionnary(obstacle_corners,robot,goal)
    obstacle_corners = naming_points(obstacle_corners,robot,goal)
    path = finding_path(adj_list,obstacle_corners)

    return path