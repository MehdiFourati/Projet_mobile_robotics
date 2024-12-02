import math
import time
import numpy as np

program = """
onevent speed
    motor.left.target = event.args[0]
    motor.right.target = event.args[1]
"""

# Function to set speed of motors
def set_speed(left,right,aw,node):
    v = {"motor.left.target": [left],
        "motor.right.target": [right],
    }
    aw(node.set_variables(v))


def controlling_wheels_speed(left_wheel,right_wheel,aw,node):

    aw(node.register_events([("speed", 2)]))
    aw(node.compile(program))
    aw(node.send_events({"speed": [left_wheel,right_wheel]}))
    aw(node.run())

    return 

def kidnapping(node):
    
    default_value = [0,0,20]
    combined_error = sum(abs(curr - default) for curr, default in zip(node.v.acc, default_value))
    if combined_error > 10:
        return True
    else:
        return False
    
def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def aligning_thymio(robot,path,node,aw):                    #to be removed ? 

    x1, y1 = path[0]
    x2, y2 = path[1]

    seg_angle = math.atan2(y2 - y1, x2 - x1)
    normalized_angle = normalize_angle(seg_angle)
    rotation =  normalized_angle - robot.alpha 
    return

# PI controller state
angle_integral = 0
distance_integral = 0
last_time = None



def pi_controller(error, integral, kp, ki, dt):
    """
    Simple PI controller.
    """
    integral += error * dt
    output = kp * error + ki * integral
    return output, integral

""" def compute_wheel_speeds(robot_pose, target_point, wheelbase, kp_angle, ki_angle, kp_distance, ki_distance):
    
    global angle_integral, distance_integral, last_time

    x = robot_pose.center_x
    y = robot_pose.center_y
    alpha = robot_pose.alpha
    x_target, y_target = target_point

    current_time = time.time()

    if last_time is None:
        last_time = current_time
    dt = current_time - last_time
    last_time = current_time

    desired_angle = math.atan2(y_target - y, x_target - x)
    angle_error = normalize_angle(desired_angle - alpha)

    # Compute angular velocity (omega) using the PI angle controller
    omega, angle_integral = pi_controller(angle_error, angle_integral, kp_angle, ki_angle, dt)

    # Calculate distance error
    distance_error = math.sqrt((x_target - x)**2 + (y_target - y)**2)

    # Compute linear velocity (v) using the PI distance controller
    v, distance_integral = pi_controller(distance_error, distance_integral, kp_distance, ki_distance, dt)

    # Convert linear and angular velocities to wheel speeds
    v_L = v - (omega * wheelbase / 2)
    v_R = v + (omega * wheelbase / 2)

    return v_L, v_R """


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
    linear_error = np.linalg.norm(np.cross(end_point-start_point, start_point-robot_center))/np.linalg.norm(end_point-start_point)

    # if positive the point is on the left side, if negative the point is on the right side
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
    remaining_path = np.sqrt((end_point[0]-robot_center[0])**2 + (end_point[1]-robot_center[1])**2)

    if remaining_path < delta:
        return True

    return False    