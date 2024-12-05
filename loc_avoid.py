import global_planning
from Controlling_thymio import Robot
from Controlling_thymio import set_speed



# local_state with the differents states for local avoidance 
local_state = 1
DEFAULT = 0                                           # No obstacle case
FRONT = 1                                             # Front obstacle detected case
FRONT_LEFT = 2                                        # Front obstacle case with obstacle on the left
FRONT_RIGHT = 3                                       # Front obstacle case with obstacle on the right
RIGHT_OBST = 4                                        # Right obstacle on the side case
LEFT_OBST = 5                                         # Left obstacle on the side case


# Different cste for threshold, speed, rotation threshold
OBST_THR = 2500                                       # About 5.25 cm distance
SPEED = 150                                           # Normal speed of the robot
SPEEDROT1 = 450                                       # Speed rotation for front and side obstacle
SPEEDROT2 = 75                                        # Speed rotation for side obstacle
SPEED_ZERO = 0                                        # Speed zero rotation for front obstacle

# Function update proximity values for Thymio
def update_prox_values(node, aw):
    aw(node.wait_for_variables({"prox.horizontal"}))
    prox = list(node.v.prox.horizontal).copy()
    return prox

""" # Function to set speed of motors
def set_speed(node, left,right,aw):
    v = {"motor.left.target": [left],
        "motor.right.target": [right],
    }
    aw(node.set_variables(v)) """

# Function to update current local state
def update_local_state(robot_instance):

    global local_state, OBST_THR
    
    obst_detected = [0,0,0,0,0]                      # list to know which sensor is detecting obstacle 
    sum_obst = 0                                     # sum of nbr prox detecting obstacle

    for i in range(5):  # Check every 5 front proximity sensors
        if robot_instance.front_prox[i] > OBST_THR:  # If one proximity value exceeds threshold
            obst_detected[i] = 1                     # we updated the list corresponding to which sensor detects obstacle

    for i in range(5):                               # We calculate how many prox sensors are detecting smth
        sum_obst = sum_obst + obst_detected[i]
    if sum_obst == 0:                                # If true, it means no obstacle have been detected by the sensors
        local_state = DEFAULT
        return
    
    if obst_detected[2] == 1:                               # if an obst is detected 
        local_state = FRONT                                 # Front state if obstacle in front of the robot 
    elif obst_detected[0] == 1 or obst_detected[1] == 1: 
        local_state = LEFT_OBST                             # Side left obstacle state
    elif obst_detected[3] == 1 or obst_detected[4] == 1:    
        local_state = RIGHT_OBST                            # Side right obstacle state

# Front obstacle reactions function, return updated speed 
def front_obst(node, robot_instance, aw):     

    global local_state
    if local_state not in {1, 2, 3}:                        # If not corresponding state we return
        return 0, 0

    # Variables that represent the average of left and right proximity values for following state conditions
    close_left = (robot_instance.front_prox[0] + robot_instance.front_prox[1]) // 2
    close_right = (robot_instance.front_prox[3] + robot_instance.front_prox[4]) // 2

    if local_state == FRONT:                                # Checking in which front state the robot is
        if close_left > close_right:
            local_state = FRONT_LEFT                        # Front obstacle more on the left  
            
        elif close_right > close_left:
            local_state = FRONT_RIGHT                       # Front obstacle more on the right
            
    if local_state == FRONT_LEFT:
        if close_left > OBST_THR:
            set_speed(node, SPEEDROT1, SPEED_ZERO, aw)
            robot_instance.update_speed(SPEEDROT1, SPEED_ZERO)
            return SPEEDROT1, SPEED_ZERO

    elif local_state == FRONT_RIGHT:
        if close_right > OBST_THR:
            set_speed(node, SPEED_ZERO, SPEEDROT1, aw)
            robot_instance.update_speed(SPEED_ZERO, SPEEDROT1)
            return SPEED_ZERO, SPEEDROT1
        
    local_state = DEFAULT
    return 0, 0

# Side obstacle reactions function, return last state of speed 
def side_obst(node, robot_instance, aw):

    global local_state
    if local_state not in {4,5}:                                 #If not corresponding state we return
        return 0, 0
    
    if local_state == LEFT_OBST:
        if robot_instance.front_prox[0] > OBST_THR:
            set_speed(node, SPEEDROT1, SPEEDROT2, aw)
            robot_instance.update_speed(SPEEDROT1, SPEEDROT2)
            return SPEEDROT1, SPEEDROT2
        local_state = DEFAULT

    if local_state == RIGHT_OBST:
        if robot_instance.front_prox[4] > OBST_THR:
            set_speed(node, SPEEDROT2, SPEEDROT1, aw)
            robot_instance.update_speed(SPEEDROT2, SPEEDROT1)
            return SPEEDROT2, SPEEDROT1
        local_state = DEFAULT

    return 0, 0
    
def local_avoidance(node, robot_instance, aw):

    robot_instance.update_front_prox(update_prox_values(node,aw))   # Updating new values for proximity sensors 
 
    set_speed(node, robot_instance.lspeed, robot_instance.rspeed, aw) # Setting speed

    update_local_state(robot_instance)                              # REMOVE Updating local state to do the corresponding avoidance

    if local_state in(RIGHT_OBST, LEFT_OBST):                       # Local avoidance functions for front and side obstacle called 
        return side_obst(node, robot_instance, aw)

    if local_state in(FRONT, FRONT_LEFT, FRONT_RIGHT ): 
        return front_obst(node, robot_instance, aw)

def check_local_nav(node, robot_instance, aw):
    
    robot_instance.update_front_prox(update_prox_values(node,aw))   # update proximity values 
    update_local_state(robot_instance)                              # we update local state
    if  local_state == DEFAULT:
        return False                                                # no local obstacle detected
    return True                                                     # local obstacle detected