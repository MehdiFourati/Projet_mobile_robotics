import cv2 as cv
import numpy as np
import math

from utils.utils_vision import get_largest_contours
from utils.utils_vision import get_contour_center
from utils.utils_vision import get_further_vertices


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

    # threshold on the background color and find the contour of the shape set to 0 (!TO OPTMIZE ON BACKGROUND!)
    
    #_, thresholded_blue = cv.threshold(copy[:,:,0],127,255,cv.THRESH_BINARY)
    #thresholded = thresholded_blue ^ thresholded_green
    #thresholded = thresholded ^ thresholded_red

    _, thresholded_green = cv.threshold(copy[:,:,1],127,255,cv.THRESH_BINARY)
    _, thresholded_red = cv.threshold(copy[:,:,2],127,255,cv.THRESH_BINARY)

    thresholded = thresholded_green & thresholded_red
    thresholded = (255 - thresholded)

    contours, _ = cv.findContours(thresholded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    c = get_largest_contours(contours)
    
    # approximate the field of play to a polygon with 4 vertices
    epsilon = 0.05 # precision of polygonal approximation, smaller is more precise (TO OPTIMIZE)
    perimeter = cv.arcLength(c, True)
    approximation = cv.approxPolyDP(c, epsilon * perimeter, True)
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


def get_starting_position(frame):
    """
    Find the starting position, angle and width of the robot
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
    gray_frame = cv.cvtColor(copy, cv.COLOR_BGR2GRAY)
    _, thresholded = cv.threshold(gray_frame,200,255,cv.THRESH_BINARY)
    contours, _ = cv.findContours(thresholded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    c = get_largest_contours(contours)

    # get the largest side of the tilted rectangle to get the robot width
    tilted_rect = cv.minAreaRect(c)
    robot_width = int(max(tilted_rect[1]))

    # approximate the contour roughly and more precisely
    epsilon = 0.01 # precision of polygonal approximation, smaller is more precise (TO OPTIMIZE)
    perimeter = cv.arcLength(c, True)
    curved_approximation = cv.approxPolyDP(c, epsilon * perimeter, True)
    epsilon = 0.02
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

    # threshold on the background color and find the contour of the shape set to 0
    _, thresholded = cv.threshold(copy[:,:,0],127,255,cv.THRESH_BINARY)
    contours, _ = cv.findContours(thresholded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    polygonal_obstacles = []

    # loop over all contour
    for c in contours:

        # remove too small or too large contours (TO OPTIMIZE)
        if cv.contourArea(c) > 50000 or cv.contourArea(c) < 10000: 
            continue

        # approximate the contour
        epsilon = 0.02 # precision of polygonal approximation, smaller is more precise (TO OPTIMIZE)
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
    _, thresholded_white = cv.threshold(gray_frame,200,255,cv.THRESH_BINARY)
    thresholded = thresholded_red ^ thresholded_white

    contours, _ = cv.findContours(thresholded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    c = get_largest_contours(contours)

    # get the center point of the objective
    objective_x, objective_y = get_contour_center(c)

    return objective_x, objective_y


def get_robot_center(frame):
    """
    Find the center of the robot
    Input: 
        - frame: corrected rectangular FOP
    Output: 
        - center_x, center_y: coordinates of the center of the robot
    """
    # to avoid overwriting the input
    copy = frame.copy()

    # threshold on the grayscale image and find the contour of the white shapes
    gray_frame = cv.cvtColor(copy, cv.COLOR_BGR2GRAY)
    _, thresholded = cv.threshold(gray_frame,170,255,cv.THRESH_BINARY)
    contours, _ = cv.findContours(thresholded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return 0, 0

    c = get_largest_contours(contours)

    center_x, center_y = get_contour_center(c)

    return center_x, center_y

""" # Vision example on an image

# load the image
img = cv.imread("testing_vision.jpg")
img = cv.resize(img, (640,480), interpolation=cv.INTER_CUBIC) 

# do the actual vision
_, _, _, robot_width = get_starting_position(img)

obstacles = get_obstacles(img, 62)

# draws everything and displays it
output = img.copy()

for obstacle in obstacles:
    for vertice in obstacle:
        cv.circle(output, (vertice[0],vertice[1]), radius=5, color=(0, 255, 0), thickness=-1)


cv.imshow("image", output)

# press any key to close all windows
cv.waitKey(0) 
cv.destroyAllWindows()  """

""" # create a mask with the obstacles white and the rest black
    _, black_frame = cv.threshold(copy[:,:,0],255,255,cv.THRESH_BINARY)
    obstacle_mask = black_frame.copy()
    for polygon in polygonal_obstacles:
        cv.drawContours(obstacle_mask, polygon, -1, (255, 255, 255), -1)

    # get the vertices further away to avoid crashing in the obstacle
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

            opposite_x = origin_x - 5*(center_x-origin_x)
            opposite_y = origin_y - 5*(center_y-origin_y)
            
            black_frame_line = black_frame.copy()
            cv.line(black_frame_line,(center_x +5*(center_x-origin_x),center_y+5*(center_y-origin_y)),(opposite_x,opposite_y),(255,255,255),1)
            
            outside_obstacle = black_frame_line ^ obstacle_mask
            outside_obstacle = outside_obstacle - obstacle_mask

            width_mask = black_frame.copy()
            cv.circle(width_mask, (origin_x,origin_y), radius=int(robot_width/1.8), color=(255, 255, 255), thickness=1)

            outside_point = width_mask & outside_obstacle
            outside_point = outside_point - (outside_point & obstacle_mask)

            outside_coordinates, _ = cv.findContours(outside_point, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            # if vertice is out of bounds throw it away
            try:
                polygon_vertices.append(outside_coordinates[0][0][0])
            except:
                continue

        further_vertices.append(polygon_vertices) """