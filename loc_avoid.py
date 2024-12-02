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
rotationThr = 2000
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
        return 
    
    close_left = (robot_instance.front_prox[0] + robot_instance.front_prox[1]) // 2
    close_right = (robot_instance.front_prox[3] + robot_instance.front_prox[4]) // 2

    if local_state == FRONT:
        if close_left > close_right:
            local_state = FRONT_LEFT
            
        elif close_right > close_left:
            local_state = FRONT_RIGHT
            
    if local_state == FRONT_LEFT:
        while close_left > rotationThr:
            set_speed(node, speed, -speed, aw)
            robot_instance.update_front_prox(update_prox_values(node, aw))
            close_left = (robot_instance.front_prox[0] + robot_instance.front_prox[1]) // 2
        local_state = DEFAULT

    elif local_state == FRONT_RIGHT:
        while close_right > rotationThr:
            set_speed(node, -speed, speed, aw)
            robot_instance.update_front_prox(update_prox_values(node, aw))
            close_right = (robot_instance.front_prox[3] + robot_instance.front_prox[4]) // 2
        local_state = DEFAULT

# Side obstacle reactions function 
def side_obst(node, robot_instance, aw):

    global local_state
    if local_state not in {4,5}:
        return
    
    if local_state == LEFT_OBST:
        while robot_instance.front_prox[0] > rotationThr:
            set_speed(node, speed, -speed, aw)
            robot_instance.update_front_prox(update_prox_values(node, aw))
        local_state = DEFAULT

    if local_state == RIGHT_OBST:
        while robot_instance.front_prox[4] > rotationThr:
            set_speed(node, -speed, speed, aw)
            robot_instance.update_front_prox(update_prox_values(node, aw))
        local_state = DEFAULT

def local_avoidance(node,robot_instance, aw):

    # Updating new values for proximity sensors
    robot_instance.update_front_prox(update_prox_values(node,aw))  

    # Setting default speed 
    #set_speed(node, speed, speed, aw)               #TO BE REMOVED

    # Updating local state to do the corresponding avoidance
    update_local_state(robot_instance)  

    # Local avoidance functions for front and side obstacle called
    front_obst(node,robot_instance, aw)    
    side_obst(node, robot_instance, aw)

    # Waiting for new Thymio's proximity data 
    aw(node.wait_for_variables({"prox.horizontal"}))    # Waiting for new Thymio's proximity data 
