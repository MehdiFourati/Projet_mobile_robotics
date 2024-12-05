import cv2 as cv
import numpy as np
import math

def get_fop_coordinates(frame):
    """
    Find the coordinates of the field of play (FOP) in the raw image
    Input: 
        - frame: raw BGR frame
    Output: 
        - original_coordinates: coordinates of the 4 corner of the FOP in the raw image
        - new_coordinates:  coordinates of the smallest rectangle containing the FOP, adapted to have top left = (0, 0) 
    """
    # to avoid overwriting the input
    copy = frame.copy()

    # threshold on the background color and find the contour of the shape set to 0

    _, thresholded_blue = cv.threshold(copy[:,:,0], 127,255,cv.THRESH_BINARY_INV)

    contours, _ = cv.findContours(thresholded_blue, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    c = get_largest_contours(contours)
    
    # approximate the field of play to a polygon with 4 vertices
    epsilon = 0.05 # precision of polygonal approximation, smaller is more precise
    perimeter = cv.arcLength(c, True) # True means closed contour
    approximation = cv.approxPolyDP(c, epsilon * perimeter, True) # 2nd parameter: maximum distance from contour to approximation
    original_coordinates = [approximation[0][0], approximation[1][0], approximation[2][0], approximation[3][0]]

    # sort the vertices by the sum of their components. If camera set correctly [tl, bl, tr, br]
    original_coordinates = sorted(original_coordinates, key=sum)

    # get the maximal width and height of the field of play
    _, _, width, height = cv.boundingRect(c)

    # make a perspective transform from the old to the new coordinates
    original_coordinates = np.float32(original_coordinates)
    new_coordinates = np.float32([[0,0],[0,height],[width,0],[width,height]])

    return original_coordinates, new_coordinates


def get_fop(frame, original_coordinates, new_coordinates):
    """
    Return the corrected rectangular FOP 
    Input: 
        - frame: raw BGR frame
        - original_coordinates: coordinates of the 4 corner of the FOP in the raw image
        - new_coordinates:  coordinates of the smallest rectangle containing the FOP, adapted to have top left = (0, 0)  
    Output: 
        - fop: corrected rectangular FOP
    """
    matrix = cv.getPerspectiveTransform(original_coordinates, new_coordinates)
    fop = cv.warpPerspective(frame, matrix, (int(new_coordinates[3][0]), int(new_coordinates[3][1]))) 

    return fop


def get_robot_position(frame):
    """
    Find the position, angle and width of the robot
    Input: 
        - frame: corrected rectangular FOP
    Output: 
        - center_x, center_y: coordinates of the center of the robot
        - alpha: angle of the robot relative to the x axis, counterclockwise, expressed in radian in range (-pi, pi]
        - robot_width: integer width of the robot, in pixel
    """
    # to avoid overwriting the input
    copy = frame.copy()

    # threshold on the grayscale image and find the contour of the white shapes
    _, thresholded_blue = cv.threshold(copy[:,:,0], 127, 255,cv.THRESH_BINARY)
    contours, _ = cv.findContours(thresholded_blue, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    c = get_largest_contours(contours)

    # get the largest side of the tilted rectangle to get the robot width
    tilted_rect = cv.minAreaRect(c)
    robot_width = int(max(tilted_rect[1]))

    # approximate the contour roughly and more precisely
    epsilon = 0.03 # precision of polygonal approximation, smaller is more precise
    perimeter = cv.arcLength(c, True)
    curved_approximation = cv.approxPolyDP(c, epsilon * perimeter, True)
    epsilon = 0.04
    straight_approximation = cv.approxPolyDP(c, epsilon * perimeter, True)

    # get the curved part of Thymio by checking the difference between the two contours
    black_frame_curved = np.zeros(frame.shape, dtype = np.uint8)
    cv.drawContours(black_frame_curved, [curved_approximation], -1, (255, 255, 255), -1)
    black_frame_straight = np.zeros(frame.shape, dtype = np.uint8)
    cv.drawContours(black_frame_straight, [straight_approximation], -1, (255, 255, 255), -1)

    curved_side = black_frame_curved ^ black_frame_straight
    contours, _ = cv.findContours(curved_side[:,:,0], cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # get the center of the curved part in the front
    direction_x = 0
    direction_y = 0

    for c in contours:
        x, y = get_contour_center(c)
        direction_x += x
        direction_y += y

    direction_x = int(direction_x / len(contours))
    direction_y = int(direction_y / len(contours))

    # get the center of the robot
    center_x, center_y = get_contour_center(straight_approximation)

    # compute the angle interpreting y as the imaginary axis to get the complex argument
    alpha = np.angle([direction_x - center_x - (direction_y - center_y)*1j]) # minus sign for complex part because y-axis going downward
    
    return center_x, center_y, alpha, robot_width


def get_obstacles(frame, robot_width):
    """
    Find the polygonal approximation of the obstacles
    Input: 
        - frame: corrected rectangular FOP
        - robot_width: width of the robot
    Output: 
        - further_vertices: lists of arrays of vertices of each obstacles' polygonal approximation, computed to avoid crashing
    """
    # to avoid overwriting the input
    copy = frame.copy()

    # threshold on the background color and find the contour of the shapes set to 0
    _, thresholded = cv.threshold(copy[:,:,0],50,255,cv.THRESH_BINARY)
    contours, _ = cv.findContours(thresholded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    polygonal_obstacles = []

    # loop over all contour
    for c in contours:
        
        # remove too small or too large contours
        if cv.contourArea(c) > 50000 or cv.contourArea(c) < 2000: 
            continue
        
        # approximate the contour
        epsilon = 0.03 # precision of polygonal approximation, smaller is more precise
        perimeter = cv.arcLength(c, True)
        approximation = cv.approxPolyDP(c, epsilon * perimeter, True)

        polygonal_obstacles.append([approximation])

    # compute the vertices further away to avoid crashing
    further_vertices = get_further_vertices(copy, robot_width, polygonal_obstacles)
        
    return further_vertices


def get_objective(frame):
    """
    Find the objective
    Input: 
        - frame: corrected rectangular FOP
    Output: 
        - objective_x, objective_y: coordinates of the center of the objective
    """
    # to avoid overwriting the input
    copy = frame.copy()

    # threshold on the objective color
    _, thresholded_red = cv.threshold(copy[:,:,2],150,255,cv.THRESH_BINARY)
    
    # threshold on the grayscale image and remove Thymio from the possible zone of objective
    gray_frame = cv.cvtColor(copy, cv.COLOR_BGR2GRAY)
    _, thresholded_white = cv.threshold(gray_frame,127,255,cv.THRESH_BINARY)
    thresholded = thresholded_red ^ thresholded_white & thresholded_red

    contours, _ = cv.findContours(thresholded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    c = get_largest_contours(contours)

    # get the center point of the objective
    objective_x, objective_y = get_contour_center(c)

    return objective_x, objective_y


def get_largest_contours(contours):
    """
    Get the largest contour of the list given
    Input:
        - contours: list of contours
    Output:
        - largest_contour: largest contour of the list
    """
    largest_contour_area = 0
    largest_contour_indices = 0 

    for i in range(len(contours)):
        if cv.contourArea(contours[i]) > largest_contour_area: 
            largest_contour_area = cv.contourArea(contours[i])
            largest_contour_indices = i
    
    largest_contour = contours[largest_contour_indices]
    return largest_contour


def get_contour_center(contour):
    """
    Get the center point of the contour
    Input:
        - contour: array of vertices
    Output:
        - center_x, center_y: coordinates of the center of the contour
    """
    sum_x = 0
    sum_y = 0

    for point in contour:
        sum_x += point[0][0]
        sum_y += point[0][1]

    center_x = int(sum_x / len(contour))
    center_y = int(sum_y / len(contour))
    
    return center_x, center_y


def get_further_vertices(frame, robot_width, polygonal_obstacles):
    """
    Compute the vertices of the obstacles further away from them to avoid the robot crashing
    Input:
        - frame: image used to compute the polygonal approximation of the obstacles
        - robot_width: width of the robot
        - polygonal_obstacles: polygonal approximation of the obstacles
    Output:
        - further_vertices: lists of arrays of vertices of each obstacles' polygonal approximation, computed to avoid crashing
    """
    # create a mask with the obstacles white and the rest black
    _, black_frame = cv.threshold(frame[:,:,0],255,255,cv.THRESH_BINARY)
    obstacle_mask = black_frame.copy()
    for polygon in polygonal_obstacles:
        cv.drawContours(obstacle_mask, polygon, -1, (255, 255, 255), -1)

    further_vertices = []

    # loop over the obstacles
    for polygon in polygonal_obstacles:
        
        polygon_vertices = []

        # loop over the points of each obstacle
        for i in range(len(polygon[0])): # [0] because of the shape
            
            # get the coordinate of a point
            previous_x = polygon[0][i][0][0]
            previous_y = polygon[0][i][0][1]
            # get the next point
            origin_x = polygon[0][(i+1)%len(polygon[0])][0][0] 
            origin_y = polygon[0][(i+1)%len(polygon[0])][0][1]
            # get the second next point
            next_x = polygon[0][(i+2)%len(polygon[0])][0][0] 
            next_y = polygon[0][(i+2)%len(polygon[0])][0][1]
            
            # use the angle bisector theorem
            vect_next = math.sqrt((next_x - origin_x)**2 + (next_y - origin_y)**2)
            vect_previous = math.sqrt((previous_x - origin_x)**2 + (previous_y - origin_y)**2)
            vect_opposite = math.sqrt((next_x - previous_x)**2 + (next_y - previous_y)**2)
            previous_opposite = vect_opposite/(1+vect_next/vect_previous)
            next_opposite = vect_opposite/(1+vect_previous/vect_next)

            # get the coordinates of intersection between the bisector and the opposite side
            black_frame_1 = black_frame.copy()
            black_frame_2 = black_frame.copy()
            cv.circle(black_frame_1, (previous_x,previous_y), radius=int(previous_opposite), color=(255, 255, 255), thickness=2)
            cv.circle(black_frame_2, (next_x,next_y), radius=int(next_opposite), color=(255, 255, 255), thickness=2)            
            intersection = black_frame_1 & black_frame_2
            contours, _ = cv.findContours(intersection, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            center_x, center_y = get_contour_center(contours[0])

            # compute the line going trought the point computed above and the point of interest
            opposite_x = origin_x - 5*(center_x-origin_x) # 5* to make the line longer
            opposite_y = origin_y - 5*(center_y-origin_y)
            black_frame_line = black_frame.copy()
            cv.line(black_frame_line,(center_x + 5*(center_x-origin_x),center_y + 5*(center_y-origin_y)),(opposite_x,opposite_y),(255,255,255),2) # 5* to make the line longer
            
            # compute the intersections between the line and a circle of radius robot_width and center at the point of interest
            outside_obstacle = black_frame_line ^ obstacle_mask
            outside_obstacle = outside_obstacle - obstacle_mask
            width_mask = black_frame.copy()
            cv.circle(width_mask, (origin_x,origin_y), radius=int(robot_width), color=(255, 255, 255), thickness=2)

            # remove the intersections inside the obstacle to only have the one outside
            outside_point = width_mask & outside_obstacle
            outside_point = outside_point - (outside_point & obstacle_mask)
            outside_coordinates, _ = cv.findContours(outside_point, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            # if vertice is out of bounds throw it away
            try:
                polygon_vertices.append(outside_coordinates[0][0][0])
            except:
                continue

        further_vertices.append(polygon_vertices)

    return further_vertices


""" # load the image
img = cv.imread("nique.png")
img = cv.resize(img, (640,480), interpolation=cv.INTER_CUBIC) 

original_coordinates, new_coordinates = get_fop_coordinates(img)
fop = get_fop(img, original_coordinates, new_coordinates)
start_x, start_y, robot_angle, width = get_robot_position(fop)
obstacles = get_obstacles(fop, width)
objective_x, objective_y = get_objective(fop) """