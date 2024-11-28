# Local avoidance py file, Fatih

class Robot2:
    def __init__(self):
        self.center_x = None
        self.center_y = None
        self.alpha = None
        self.robot_width = None
        #////////////////////
        self.front_prox = [0,0,0,0,0,0,0]  # Values front prox.horizontal

    def update_coordinates(self, center_x, center_y, alpha, robot_width):
        self.center_x = center_x
        self.center_y = center_y
        self.alpha = alpha
        self.robot_width = robot_width
    #////////////////////
    def update_front_prox(self, prox_values):
        self.front_prox = prox_values
    