import math 

def obstacle_dictionnary(obstacles):                                             
    """Create a dictionnary with key : obstacle and value : coordinates of all its vertices
    Input:
        - obstacles: list of the vertices of the obstacles
    Output:
        - obstacle_dict: dictionnary with key : obstacle, value : coordinates (x,y)
    """
    obstacle_dict = {}
    for i, points_list in enumerate(obstacles):
            obstacle_dict[f'obstacle_{i+1}'] = points_list
    return obstacle_dict


def calculate_distance(point1, point2):
    """Compute the euclidean distance between two given points
    Input:
        - point1, point2: list of one 2D point (x,y)
    Output:
        - euclidean distance between the two given points
    """
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def naming_points(object_corners, robot, goal):
    """Name all vertices and start and goal points
    Input:
        - object_corners: dictionnary with key : obstacle, value : coordinates
        - robot: instance of the class robot
        - goal: coordinates of the goal
    Output:
        - point_list_named: dictionnary with key : name of the point, value : coordinates (x,y)
    """  
    point_list_named = {}
    i = 0

    for _, points_list in object_corners.items():
        for point in points_list:
            point_list_named[f'P{i}'] = point
            i += 1

    point_list_named['R']= robot.center_x, robot.center_y
    point_list_named['G']= goal

    return point_list_named

def inside_line(point1, point2, point3): 
    """Check a point if between two others while the 3 points are colinear to each others
    Input:
        - point1, point2, point3: list of one 2D point (x,y)
    Output:
        - True if the 3rd point lies in the segment of the first two points, else False
    """
    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3
    if (min(x1, x2) <= x3 <= max(x1, x2) and min(y1, y2) <= y3 <= max(y1, y2)): 
        return True
    return False

def orientation(point1,point2,point3):
    """Check if a point if to the left, the right or on a line generated from two other points
    Input:
        - point1, point2, point3: list of one 2D point (x,y)
    Output:
        - 0 if colinear, -1 if to the right, 1 if to the left
    """
    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3

    orientation = (x3-x2)*(y2-y1)-(x2-x1)*(y3-y2) # (Source: https://www.geeksforgeeks.org/orientation-3-ordered-points/, last accessed 18.11.2024)

    # Check if the orientation is respectively collinear, clockwise, or counterclockwise. To be used later for intersections.
    if orientation == 0:
        return 0  
    elif orientation > 0:
        return -1
    else:
        return 1 

def is_intersection(point1,point2,point3,point4):
    """Check if two line intersect with each other
    Input:
        - point1, point2, point3, point4: list of one 2D point (x,y)
    Output:
        - True if intersection, else False"""
    
    ori1=orientation(point1,point2,point3)
    ori2=orientation(point1,point2,point4)
    ori3=orientation(point3,point4,point1)
    ori4=orientation(point3,point4,point2)

    #If two points are equal, we do not consider them as intersecting since it is the case where 
    #the points belongs to the same object, and this case is taken care of later.
    if point1==point3 or point1==point4 or point2==point3 or point2==point4:
        return False
    
    # If o1 and o2 are different and o3 and o4 are different, then the points intersect
    if ori1 != ori2 and ori3 != ori4:
        return True #they intersect
    
    # Checking colinearity in every case + if the third point is inside the segment
    if ((ori1 == 0) and inside_line(point1, point2, point3)): 
        return True
    if ((ori2 == 0) and inside_line(point1, point2, point4)): 
        return True
    if ((ori3 == 0) and inside_line(point3, point4, point1)): 
        return True
    if ((ori4 == 0) and inside_line(point3, point4, point2)): 
        return True
    
    return False 


def point_connection(point1, point2,obstacle_corners,points_named):
    """Check if two points can be linked without crossing an obstacle
    Input: 
        - point1, point2: list of one 2D point (x,y)
        - object_corners: dictionnary with key : obstacle, value : coordinates
        - points_named: dictionnary with key : name of the point, value : coordinates (x,y)
    Output:
        - True if two points can be connected by a straight line without crossing an obstacle, else False"""

    coordinates = {i: j for j, i in points_named.items()}   #New dictionary inverting the keys and values of points_names in order to get the names of the given points 

    p1 = coordinates[point1] 
    p2 = coordinates[point2] 

    for _, point_list in obstacle_corners.items():
        # Check if both points are in the same obstacle
        if p1 in point_list and p2 in point_list:
            # and if the points are adjacent, then they are connected for sure
            if (abs(point_list.index(p1) - point_list.index(p2)) == 1 or (point_list.index(p1) == 0 and point_list.index(p2) == len(point_list) - 1) or (point_list.index(p2) == 0 and point_list.index(p1) == len(point_list) - 1)):
                return True  

            # Now check if the line between point1 and point2 intersects any obstacle edges
        for i in range(len(point_list)):
            if i == len(point_list) - 1:  # Last point connects to the first
                if is_intersection(point1, point2, point_list[i], point_list[0]):
                    return False
            else:
                if point_list[i] == point1 or point_list[i + 1] == point2:
                    continue  # Skip if it's the same as the input points
                if is_intersection(point1, point2, point_list[i], point_list[i + 1]):
                    return False  

    return True  # connected



def creating_adjacency_dictionnary(object_corners,points_named):
    """Create a dictionnary with key: point name, values: names of all points that are connected to it
    Input: 
        - object_corners: dictionnary with key : obstacle, value : coordinates
        - points_named: dictionnary with key : name of the point, value : coordinates (x,y)
    Output:
        - adjacency_dict: dictionnary with key: point name, values: names of all points that are connected to it
    """
    adjacency_dict = {}

    for p1, point1 in points_named.items():
        adjacency_dict[p1] = []
        for p2, point2 in points_named.items():
            if p1 != p2:
                if point_connection(point1, point2, object_corners,points_named):
                    adjacency_dict[p1].append(p2)

    return adjacency_dict


def calculating_distances(adjacency_dict, points_named):
    """Create a dictionnary with key: two connected points, value: distance between them
    Input:
        - adjacency_dict: dictionnary with key: point name, values: names of all points that are connected to it
        - points_named: dictionnary with key : name of the point, value : coordinates (x,y)
    Output:
        - distance_dict: dictionnary with key: two connected points, value: distance between them
    """
    distance_dict = {}  

    for point, connected_points in adjacency_dict.items():
        for i in connected_points:
        
            dist = calculate_distance(points_named[point], points_named[i])
    
            distance_dict[(point, i)] = dist
            distance_dict[(i, point)] = dist
    
    return distance_dict


def get_dist(distance_dict, point1, point2):
    """Fetch the distance contained in the euclidean distance dictionnary for two given points
    Input: 
        - distance_dict: dictionnary with key: two connected points, value: distance between them
        - point1, point2: list of one 2D point (x,y)
    Output:
        - dist: euclidean distance between the two points
    """
    for point, dist in distance_dict.items():
        if point[0] == point1 and point[1] == point2:
            return dist
        
def dijkstra_algo(adjacency_dict, points_named):
    """Find the best next node
    Input:
        - adjacency_dict: dictionnary with key: point name, values: names of all points that are connected to it
        - points_named: dictionnary with key : name of the point, value : coordinates (x,y)
    Output:
        - previous_nodes: dictionnary with key : point name, value : point name right before the key in the path
    """
    # Used the template from this website : https://www.udacity.com/blog/2021/10/implementing-dijkstras-algorithm-in-python.html
    shortest_dist = {} # it will store the best known cost of getting to each node, key : point name, value : distance 
    previous_nodes = {} # it will store the best know path for each node, key : point name, value : best previous point name in the paths
    unvisited = list(points_named.keys())
    distance_dict = calculating_distances(adjacency_dict, points_named)

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
            try_value = shortest_dist[current] + get_dist(distance_dict,current, i)
            if try_value < shortest_dist[i]:
                shortest_dist[i] = try_value
                previous_nodes[i] = current
        unvisited.remove(current)


    return previous_nodes


def finding_path(adjacency_list, point_named):
    """Find the shortest path using Dijkstra algorithm
    Input:
        - adjacency_dict: dictionnary with key: point name, values: names of all points that are connected to it
        - points_named: dictionnary with key : name of the point, value : coordinates (x,y)
    Output:
        - shortest_path: list containing the points names of the shortest path from the robot to the goal
    """
    previous_nodes = dijkstra_algo(adjacency_list, point_named)
    shortest_path = ['G']
    current_node = 'G'
    while current_node != 'R':
        current_node = previous_nodes[current_node]
        shortest_path.append(current_node)
    shortest_path.reverse()
    return shortest_path


def global_plan(raw_obstacle_corners,robot,goal):
    """Compute the optimal path
    Input:
        - raw_obstacle_corners: array of the coordinates of all the obstacles' vertices
        - robot: instance of the class robot
        - goal: coordinates of the goal
    Output:
        - path: list containing the points names of the shortest path from the robot to the goal
        - points_named: dictionnary with key : name of the point, value : coordinates (x,y)
    """
    obstacle_corners = [[tuple(arr) for arr in sublist] for sublist in raw_obstacle_corners] # converting in the right format
    obstacle_corners = obstacle_dictionnary(obstacle_corners)
    points_named = naming_points(obstacle_corners,robot,goal)
    adj_list = creating_adjacency_dictionnary(obstacle_corners,points_named)
    path = finding_path(adj_list,points_named)

    return path, points_named # we return points_named for plotting