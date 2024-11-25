# Local avoidance py file, Fatih

class Robot:
    def __init__(self):
        self.center_x = None
        self.center_y = None
        self.alpha = None
        self.robot_width = None

    def update_coordinates(self, center_x, center_y, alpha, robot_width):
        self.center_x = center_x
        self.center_y = center_y
        self.alpha = alpha
        self.robot_width = robot_width

    def get_sensor_value(self, number, prox_value):
        return self.prox_value[number]
    
    def get_sensors(self, prox):
        return prox
    

def update_local_state(sensors, obstThr):
    for i in range(len(sensors)):
        if sensors[i] > obstThr:
            return True  # obstacle detected, state 1 
    return False