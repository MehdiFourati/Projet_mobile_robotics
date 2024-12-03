import math
import numpy as np

def set_speed(left, right, aw, node):
    """
    Set the speed of the wheels of the robot
    Input:
        - left: speed of the left wheel
        - right: speed of the right wheel
        - aw: helper function to handle await (c.f. tdm-python git repository: https://github.com/epfl-mobots/tdm-python/blob/main/doc/lowlevel.md)
        - node: node to which the robot is associated (c.f. tdm-python git repository: https://github.com/epfl-mobots/tdm-python/blob/main/doc/lowlevel.md)
    """
    v = {"motor.left.target": [left],
        "motor.right.target": [right],
    }
    aw(node.set_variables(v))


def kidnapping(node):
    """
    Detects if the robot is being kidnapped
    Input:
        - node: node to which the robot is associated (c.f. tdm-python git repository: https://github.com/epfl-mobots/tdm-python/blob/main/doc/lowlevel.md) 
    Output:
        - True is the robot is being kidnapped, else False
    """
    default_value = [0,0,20] # [x, y, z], g = 20
    combined_error = sum(abs(curr - default) for curr, default in zip(node.v.acc, default_value))
    if combined_error > 10:
        return True
    else:
        return False
    

def get_linear_error(start_point, end_point, robot_center):
    """
    Get the euclidean distance between the robot and its optimal trajectory
    Input:
        - start_point: last point in the path the robot went through
        - end_point: next point in the path
        - robot_center: coordinates of the center of the robot
    Output:
        - linear_error: euclidean distance between the robot and its optimal trajectory
    """
    # compute the distance between the robot center and the optimal trajectory using the area of the parallelogram
    # (Source: https://www.includehelp.com/python/distance-between-point-and-a-line-from-two-points-in-numpy.aspx, last accessed 30.11.2024)
    linear_error = np.linalg.norm(np.cross(end_point-start_point, start_point-robot_center))/np.linalg.norm(end_point-start_point)

    # get the side of the line on which the point is using a dot product with the normal
    # if positive the point is on the left side, if negative the point is on the right side
    # (Source: https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located, last accessed 30.11.2024)
    side = (robot_center[0]-start_point[0])*(end_point[1]-start_point[1]) - (robot_center[1]-start_point[1])*(end_point[0]-start_point[0])

    return linear_error * np.sign(side)


def get_angular_error(start_point, end_point, robot_angle):
    """
    Get the angle between the robot direction and its optimal trajectory
    Input:
        - start_point: last point in the path the robot went through
        - end_point: next point in the path
        - robot_angle: angle of the robot
    Output:
        - angular_error: angle between the robot direction and its optimal trajectory, in radian between ]-pi;pi]
    """
    # compute the trajectory's angle
    alpha = np.angle([end_point[0] - start_point[0] - (end_point[1] - start_point[1])*1j]) # minus sign for complex part because y-axis going downward
    angular_error = alpha - robot_angle

    if angular_error > math.pi:
        angular_error -= 2*math.pi

    elif angular_error < -1 * math.pi:
        angular_error += 2*math.pi 

    return angular_error


def PI_controller(error, kp, ki):
    """
    Use a PI controller to compute the system input to correct the robot's trajectory
    Input:
        - error: difference between optimal trajectory and actual one 
        - kp: proportional gain of the controler
        - ki: integral gain of the controler
    Output:
        - system_input: input to give to the robot
    """
    system_input = kp * error[-1] + ki * np.sum(error)

    return system_input


def reached_linear_target(end_point, robot_center, delta):
    """
    Check if the robot reached its next target point
    Input:
        - end_point: next target point of the robot
        - robot_center: coordinates of the center of the robot
    Output:
        - True if the robot reach end_point, else False
    """
    # euclidean distance between robot center and next point in path
    remaining_path = np.sqrt((end_point[0]-robot_center[0])**2 + (end_point[1]-robot_center[1])**2)

    if remaining_path < delta:
        return True

    return False    