import math 
import vision
import numpy as np
from math import sqrt


class Robot:
    def __init__(self):
        self.center_x = None
        self.center_y = None
        self.alpha = None
        self.robot_width = None

    def update_coordinates(self, center_x, center_y, alpha, robot_width):
        self.center_x = center_x
        self.center_y = center_y
        self.alpha = alpha
        self.robot_width = robot_width

class goal:
    def __init__(self):
        self.center_x = None
        self.center_y = None

    def update_coordinates(self, center_x, center_y):
        self.center_x = center_x
        self.center_y = center_y

def obstacle_dictionnary(obstacles):                                             
    """Returns a dictionnary with key : obstacle, value : coordinates"""
    obstacle_dict = {}
    for i, points_list in enumerate(obstacles):
        #if len(points_list) == 4:
            obstacle_dict[f'obstacle_{i+1}'] = points_list
    return obstacle_dict


def get_robot_coordinates(frame, robot):                   

    center_x, center_y, alpha, robot_width = vision.get_starting_position(frame)
    
    robot.update_coordinates(center_x, center_y, alpha, robot_width)

    return center_x,center_y

def get_goal_coordinates(frame, goal):                       

    center_x, center_y = vision.get_objective(frame)
    
    goal.update_coordinates(center_x, center_y)

    return goal

def calculate_distance(point1, point2):
    """Compute the euclidean distance between two given points"""

    return sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


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

def orientation(p1,p2,p3):
    """Defines the orientation of the three given points"""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    orientation_value = (x3-x2)*(y2-y1)-(x2-x1)*(y3-y2)       # got it from this site : https://www.geeksforgeeks.org/orientation-3-ordered-points/
    # Check if the orientation is respectively collinear, clockwise, or counterclockwise. To be used later for intersections.
    if orientation_value == 0:
        return 0  
    elif orientation_value > 0:
        return -1
    else:
        return 1 

def isIntersection(p1,p2,p3,p4):
    """Function that checks if two line intersect with each other"""
    
    o1=orientation(p1,p2,p3)
    o2=orientation(p1,p2,p4)
    o3=orientation(p3,p4,p1)
    o4=orientation(p3,p4,p2)

    #If two points are equal, we do not consider them as intersecting since it is the case where 
    #the points belongs to the same object, and this case is taken care of later.
    if p1==p3 or p1==p4 or p2==p3 or p2==p4:
        return False
    
    # If o1 and o2 are different and o3 and o4 are different, then the points intersect
    if o1 != o2 and o3 != o4:
        return True
    
    # Checking colinearity in every case + if the third point is inside the segment
    if ((o1 == 0) and InsideLine(p1, p2, p3)): 
        return True
    if ((o2 == 0) and InsideLine(p1, p2, p4)): 
        return True
    if ((o3 == 0) and InsideLine(p3, p4, p1)): 
        return True
    if ((o4 == 0) and InsideLine(p3, p4, p2)): 
        return True
    
    return False 

def obstacle_points_named(object_corners):
    """Returns a dictionnary with key : ostacle, value : point names """
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
    coordinates = {v: k for k, v in points_named.items()}       #New dictionary inverting the keys and values of points_names
    obstacle_points = obstacle_points_named(obstacle_corners)

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

    #We go through all objects and check if the line between point1 and point2 
    #intersects with any other line between two other points
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

def calculating_distances(adjacency_list, points_named):
    """Creating a dictionnary with 2 connected points as key, associated with the euclidean distance between them"""
    distances = {}  # Dictionary to store distances between connected points

    for point, connected_points in adjacency_list.items():
        for connected_point in connected_points:
        
            dist = calculate_distance(points_named[point], points_named[connected_point])
    
            distances[(point, connected_point)] = dist
            distances[(connected_point, point)] = dist
    
    return distances

def get_dist(distances, point1, point2):
    """Finds the distance stored in the above dictionnary for two given points"""
    for points, dist in distances.items():
        if points[0] == point1 and points[1] == point2:
            return dist
        
def dijkstra(adjacency_dict, points_named):
    shortest_dist = {} #store the best-known cost of visiting each point in the graph starting from start
    previous_nodes = {} #store the previous node of the current best known path for each node
    unvisited_nodes = list(points_named.keys())
    distances = calculating_distances(adjacency_dict, points_named)
    # We need to set every distance to infinity(~ unreachable at the start). We will simulate that using a very large value     
    infinity = 10e10
    for node in points_named.keys():
        shortest_dist[node] = infinity
    shortest_dist['R'] = 0

    while unvisited_nodes:
        current_min_node = unvisited_nodes[0]
        for node in unvisited_nodes: # Iterate over the nodes
            if shortest_dist[node] < shortest_dist[current_min_node]:
                current_min_node = node
        # We then retrieve the current node's neighbors and updates its distances
        neighbors = adjacency_dict[current_min_node]
        for neighbor in neighbors:
            test = shortest_dist[current_min_node] + get_dist(distances,current_min_node, neighbor)
            if test < shortest_dist[neighbor]:
                shortest_dist[neighbor] = test
                # We also update the best path to the current node
                previous_nodes[neighbor] = current_min_node
        unvisited_nodes.remove(current_min_node)

    return previous_nodes


def finding_path(adjacency_list, point_names):
    """Finds shortest_path using Dijkstra algorithm and returns a list containing the points names of the path 
       from the robot to the goal"""
    previous_nodes = dijkstra(adjacency_list, point_names)
    path = ['G']
    current_node = 'G'
    while current_node != 'R':
        current_node = previous_nodes[current_node]
        path.append(current_node)
    path.reverse()
    return path