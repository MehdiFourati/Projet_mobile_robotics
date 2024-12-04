import math
import numpy as np
import time


# Global constants here

KP_LINEAR = 15 # linear proportional gain in PI controller
KI_LINEAR = 0.2 # linear integral gain in PI controller
KP_ANGULAR = 15 # angular proportional gain in PI controller
KI_ANGULAR = 0.2 # angular integral gain in PI controller
PATH_DELTA = 10 # acccepted difference in pixels between the actual robot's position and its goal
ANGULAR_DELTA = 0.15 # accepted difference in radian between the actual robot's angle and its goal
TURNING_SPEED = 100 # speed of the wheel when turning
STRAIGHT_SPEED = 150 # speed of the wheel when going straight
MAX_STRAIGHT_SPEED = 200 # maximum speed to be sent to the robot


class Robot:
    #Class containing informations about the robot's position
    
    # Initialize an instance of the class
    def __init__(self):
        self.center_x = 0 # x coordinates of the center of the robot
        self.center_y = 0 # y coordinates of the center of the robot
        self.alpha = 0 # angle of the robot relative to the x axis, counterclockwise, expressed in radian in range (-pi, pi]
        self.robot_width = 0 # width of the robot
        self.front_prox = [0,0,0,0,0,0,0]  # values of the frontal proximity sensors
        self.lspeed = 0 # speed of the left wheel
        self.rspeed = 0 # speed of the right wheel

    # Update the coordinates, orientation and width of the instance
    def update_coordinates(self, center_x, center_y, alpha, robot_width):
        self.center_x = center_x
        self.center_y = center_y
        self.alpha = alpha
        self.robot_width = robot_width
        
    # Update the values of the frontal proximity sensors
    def update_front_prox(self, prox_values):
        self.front_prox = prox_values

    def update_speed(self, left, right):
        self.lspeed = left
        self.rspeed = right


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


def compute_wheel_speed(initial_turn, start_point, end_point, next_point, error_linear, error_angle, robot, path, aw, node):
    
    robot_center = np.array([robot.center_x, robot.center_y])
    robot_angle = robot.alpha

    # do the initial orientation
    if initial_turn:
        
        # turning
        error_angle = get_angular_error(end_point, next_point, robot_angle)
        left_wheel_speed = -1 * np.sign(error_angle) * TURNING_SPEED
        right_wheel_speed = np.sign(error_angle) * TURNING_SPEED
        
        # first turn is done
        if get_angular_error(end_point, next_point, robot_angle) < ANGULAR_DELTA:
            initial_turn = False
            left_wheel_speed = 0
            right_wheel_speed = 0

        set_speed(int(left_wheel_speed),int(right_wheel_speed),aw,node) 

    # check if the next point as been reached
    if reached_linear_target(end_point, robot_center, PATH_DELTA):
        
        # if the last point has been reached
        if next_point[0] == 0 and next_point[1] == 0:
            print("Target reached")
            left_wheel_speed = 0
            right_wheel_speed = 0
            set_speed(int(left_wheel_speed),int(right_wheel_speed),aw,node)
            time.sleep(2)
        else:
            if get_angular_error(end_point, next_point, robot_angle) < ANGULAR_DELTA:
                # finished turning, ready to go straight
                path = path[1:]
                left_wheel_speed = 0
                right_wheel_speed = 0
                set_speed(int(left_wheel_speed),int(right_wheel_speed),aw,node)
            else:
                # turning
                error_angle = get_angular_error(end_point, next_point, robot_angle)
                left_wheel_speed = -1 * np.sign(error_angle) * TURNING_SPEED
                right_wheel_speed = np.sign(error_angle) * TURNING_SPEED
                set_speed(int(left_wheel_speed),int(right_wheel_speed),aw,node)
    else:
        # does not do the straight control if doing the initial turning
        if not initial_turn:
            # going straight
            error_linear = np.append(error_linear, get_linear_error(start_point, end_point,robot_center))
            error_angle = np.append(error_angle, get_angular_error(start_point, end_point, robot_angle))
            
            if error_linear[-1] < 5:
                linear_input = 0
            else:
                linear_input = PI_controller(error_linear, KP_LINEAR, KI_LINEAR)

            #if error_angle[-1] < 0.1:
            #    angular_input = 0
            #else:
            angular_input = PI_controller(error_angle, KP_ANGULAR, KI_ANGULAR)

            left_wheel_speed = STRAIGHT_SPEED + angular_input + linear_input
            right_wheel_speed = STRAIGHT_SPEED - angular_input - linear_input
            
            # limit the maximum speed of the wheels
            if left_wheel_speed > MAX_STRAIGHT_SPEED: left_wheel_speed = MAX_STRAIGHT_SPEED
            if right_wheel_speed > MAX_STRAIGHT_SPEED: right_wheel_speed = MAX_STRAIGHT_SPEED
            if left_wheel_speed < -MAX_STRAIGHT_SPEED: left_wheel_speed = -MAX_STRAIGHT_SPEED
            if right_wheel_speed < -MAX_STRAIGHT_SPEED: right_wheel_speed = -MAX_STRAIGHT_SPEED
            
            set_speed(int(left_wheel_speed),int(right_wheel_speed),aw,node)
        else:
            # never reaches this, here to avoid error
            pass
    return int(left_wheel_speed), int(right_wheel_speed), error_linear, error_angle, initial_turn, path