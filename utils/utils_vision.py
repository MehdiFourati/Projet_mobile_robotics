import cv2 as cv
import numpy as np

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
