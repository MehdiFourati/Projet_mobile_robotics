import global_planning
from Controlling_thymio import Robot


# local_state with the differents states for local avoidance 
local_state = 1
DEFAULT = 0
FRONT = 1
FRONT_LEFT = 2
FRONT_RIGHT = 3
RIGHT_OBST = 4
LEFT_OBST = 5
CRITIC = 6
OBST_DETECTED = 7

# Different cste for threshold, speed, rotation threshold
rotationThr = 1000
obstThrH = 3000
speed = 150

# Function update proximity values for Thymio
def update_prox_values(node, aw):
    aw(node.wait_for_variables({"prox.horizontal"}))
    prox = list(node.v.prox.horizontal).copy()
    return prox

# Function to set speed of motors
def set_speed(node, left,right,aw):
    v = {"motor.left.target": [left],
        "motor.right.target": [right],
    }
    aw(node.set_variables(v))

# Function to update current local state
def update_local_state(robot_instance):

    global local_state, obstThrH
    
    obst_detected = [0,0,0,0,0]
    sum_obst = 0

    for i in range(5):  # Parcourt chaque capteur de proximité
        #if prox[i] > obstThrH:  # Si l'obstacle est détecté à l'un des capteurs
        if robot_instance.front_prox[i] > obstThrH:  # Si l'obstacle est détecté à l'un des capteurs
            local_state = OBST_DETECTED
            obst_detected[i] = 1
    #if rien tu sors
    for i in range(5):
        sum_obst = sum_obst + obst_detected[i]
    if sum_obst == 0:
        local_state = DEFAULT
        return
    
    #if obst detected 
    if obst_detected[2] == 1:
        local_state = FRONT
    elif obst_detected[0] == 1 or obst_detected[1] == 1: 
        local_state = LEFT_OBST
    elif obst_detected[3] == 1 or obst_detected[4] == 1:
        local_state = RIGHT_OBST  

# Front obstacle reactions function
def front_obst(node, robot_instance, aw):     

    global local_state
    if local_state not in {1, 2, 3}:
        return 0, 0

    close_left = (robot_instance.front_prox[0] + robot_instance.front_prox[1]) // 2
    close_right = (robot_instance.front_prox[3] + robot_instance.front_prox[4]) // 2

    if local_state == FRONT:
        if close_left > close_right:
            local_state = FRONT_LEFT
            
        elif close_right > close_left:
            local_state = FRONT_RIGHT
            
    if local_state == FRONT_LEFT:
        if close_left > rotationThr:
            set_speed(node, 2*speed, speed, aw)
            robot_instance.update_speed(2*speed,speed)
            robot_instance.update_front_prox(update_prox_values(node, aw))
            close_left = (robot_instance.front_prox[0] + robot_instance.front_prox[1]) // 2
            return 2*speed, speed

    elif local_state == FRONT_RIGHT:
        if close_right > rotationThr:
            set_speed(node, speed, 2*speed, aw)
            robot_instance.update_front_prox(update_prox_values(node, aw))
            robot_instance.update_speed(speed,2*speed)
            close_right = (robot_instance.front_prox[3] + robot_instance.front_prox[4]) // 2
            return speed, 2*speed
        
    local_state = DEFAULT
    return 0, 0

# Side obstacle reactions function 
def side_obst(node, robot_instance, aw):

    global local_state
    if local_state not in {4,5}:
        return 0, 0
    
    if local_state == LEFT_OBST:
        if robot_instance.front_prox[0] > rotationThr:
            set_speed(node, 2*speed, speed, aw)
            robot_instance.update_speed(2*speed,speed)
            robot_instance.update_front_prox(update_prox_values(node,aw))
            return 2*speed, speed
        local_state = DEFAULT

    if local_state == RIGHT_OBST:
        if robot_instance.front_prox[4] > rotationThr:
            set_speed(node, speed, 2*speed, aw)
            robot_instance.update_speed(speed,2*speed)
            robot_instance.update_front_prox(update_prox_values(node,aw))
            return speed, 2*speed
        local_state = DEFAULT

    return 0, 0
    
def local_avoidance(node, robot_instance, aw):

    # Updating new values for proximity sensors
    robot_instance.update_front_prox(update_prox_values(node,aw))  

    # Setting default speed 
    set_speed(node, robot_instance.lspeed, robot_instance.rspeed, aw)

    # Updating local state to do the corresponding avoidance
    update_local_state(robot_instance)  

    # Local avoidance functions for front and side obstacle called
    if local_state in(RIGHT_OBST, LEFT_OBST):
        aw(node.wait_for_variables({"prox.horizontal"}))    # Waiting for new Thymio's proximity data 
        return side_obst(node, robot_instance, aw)

    if local_state in(FRONT, FRONT_LEFT, FRONT_RIGHT ):
        aw(node.wait_for_variables({"prox.horizontal"}))    # Waiting for new Thymio's proximity data 
        return front_obst(node, robot_instance, aw)

def check_local_nav(node, robot_instance, aw):
    
    robot_instance.update_front_prox(update_prox_values(node,aw))  
    update_local_state(robot_instance)
    if  local_state == DEFAULT:
        return False
    return True
