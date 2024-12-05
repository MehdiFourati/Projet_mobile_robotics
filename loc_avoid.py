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
speedRot1 = 450
speedRot2 = 75

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
    
    obst_detected = [0,0,0,0,0]                      # list to know which sensor is detecting obstacle 
    sum_obst = 0                                     # sum of nbr prox detecting obstacle

    for i in range(5):  # Check every 5 front proximity sensors
        if robot_instance.front_prox[i] > obstThrH:  # If one proximity value exceeds threshold
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
        if close_left > rotationThr:
            set_speed(node, speedRot1, 0, aw)
            robot_instance.update_speed(speedRot1, 0)
            return speedRot1, 0

    elif local_state == FRONT_RIGHT:
        if close_right > rotationThr:
            set_speed(node, 0, speedRot1, aw)
            robot_instance.update_speed(0,speedRot1)
            return 0, speedRot1
        
    local_state = DEFAULT
    return 0, 0

# Side obstacle reactions function, return last state of speed 
def side_obst(node, robot_instance, aw):

    global local_state
    if local_state not in {4,5}:                                 #If not corresponding state we return
        return 0, 0
    
    if local_state == LEFT_OBST:
        if robot_instance.front_prox[0] > rotationThr:
            set_speed(node, speedRot1, speedRot2, aw)
            robot_instance.update_speed(speedRot1,speedRot2)
            return speedRot1, speedRot2
        local_state = DEFAULT

    if local_state == RIGHT_OBST:
        if robot_instance.front_prox[4] > rotationThr:
            set_speed(node, speedRot2, speedRot1, aw)
            robot_instance.update_speed(speedRot2, speedRot1)
            return speedRot2, speedRot1
        local_state = DEFAULT

    return 0, 0
    
def local_avoidance(node, robot_instance, aw):

    robot_instance.update_front_prox(update_prox_values(node,aw))   # Updating new values for proximity sensors 
 
    set_speed(node, robot_instance.lspeed, robot_instance.rspeed, aw) # Setting speed

    update_local_state(robot_instance)                              # Updating local state to do the corresponding avoidance

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