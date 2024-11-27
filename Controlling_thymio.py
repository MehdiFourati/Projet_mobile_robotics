import global_planning
import math
import time

program = """
onevent speed
    motor.left.target = event.args[0]
    motor.right.target = event.args[1]
"""


def controlling_wheels_speed(left_wheel,right_wheel,aw,node):

    aw(node.register_events([("speed", 2)]))
    aw(node.compile(program))
    aw(node.send_events({"speed": [left_wheel,right_wheel]}))
    aw(node.run())

    return 

def kidnapping(node):
    
    default_value = [0,0,20]
    combined_error = sum(abs(curr - default) for curr, default in zip(node.v.acc, default_value))
    print(combined_error)
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

def compute_wheel_speeds(robot_pose, target_point, wheelbase, kp_angle, ki_angle, kp_distance, ki_distance):
    
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

    return v_L, v_R


    

    



