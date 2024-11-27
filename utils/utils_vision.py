import cv2 as cv
import numpy as np
import math

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
            cv.line(black_frame_line,(center_x + 5*(center_x-origin_x),center_y + 5*(center_y-origin_y)),(opposite_x,opposite_y),(255,255,255),1) # 5* to make the line longer
            
            # compute the intersections between the line and a circle of radius robot_width/1.8 and center at the point of interest
            outside_obstacle = black_frame_line ^ obstacle_mask
            outside_obstacle = outside_obstacle - obstacle_mask
            width_mask = black_frame.copy()
            cv.circle(width_mask, (origin_x,origin_y), radius=int(robot_width/1.8), color=(255, 255, 255), thickness=1)
            outside_point = width_mask & outside_obstacle

            # remove the intersections inside the obstacle to only have the one outside
            outside_point = outside_point - (outside_point & obstacle_mask)
            outside_coordinates, _ = cv.findContours(outside_point, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            # if vertice is out of bounds throw it away
            try:
                polygon_vertices.append(outside_coordinates[0][0][0])
            except:
                continue

        further_vertices.append(polygon_vertices)

    return further_vertices