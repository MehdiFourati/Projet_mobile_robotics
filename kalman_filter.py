import numpy as np
from Controlling_thymio import Robot

def convert_input(robot):

    linear_velocity = (robot.lspeed + robot.rspeed)/2
    angular_velocity = (robot.lspeed - robot.rspeed)/robot.robot_width

    speed_x = np.cos(robot.alpha) * linear_velocity
    speed_y = np.sin(robot.alpha) * linear_velocity

    return [speed_x[0], speed_y[0], angular_velocity]


class KalmanFilter:
    
    def __init__(self, x0):
        self.F = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) # state transition model
        self.H = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) # observation model
        self.Q = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) # process noise covariance
        self.R = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) # measurement noise covariance
        self.P = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) # initial error covariance  
        self.x = x0 # initial state, shape [x,y,alpha]

    # u shape [speed_x, speed_y, speed_angular]
    def predict(self, converted_input):
        self.x = self.F @ self.x + converted_input
        self.P = self.F @ self.P @ self.F.transpose() + self.Q

    # z shape [x,y,alpha]
    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.transpose() + self.R   
        K = self.P @ self.H.transpose() @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P