import numpy as np
from Controlling_thymio import Robot

DISTANCE_WHEEL = 90 # distance between the wheels in mm
CAMERA_FPS = 30 # fps of the video feed
DISTANCE_MM_S = 0.43

def convert_input(robot):

    linear_velocity = (robot.lspeed + robot.rspeed)/2 * DISTANCE_MM_S / CAMERA_FPS * robot.robot_width / DISTANCE_WHEEL
    angular_velocity = np.arcsin(float((robot.lspeed - robot.rspeed) * DISTANCE_MM_S / CAMERA_FPS * 2 / DISTANCE_WHEEL))

    angle = float(robot.alpha)

    speed_x = np.cos(angle) * linear_velocity
    speed_y = np.sin(angle) * linear_velocity
    speed_angular = angular_velocity

    return [speed_x, speed_y, speed_angular]


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

        if self.x[2][0] > np.pi:
             self.x[2][0] -= 2*np.pi
        elif self.x[2][0] < -np.pi:
            self.x[2][0] += 2*np.pi

    # z shape [x,y,alpha]
    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.transpose() + self.R   
        K = self.P @ self.H.transpose() @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

        if self.x[2][0] > np.pi:
             self.x[2][0] -= 2*np.pi
        elif self.x[2][0] < -np.pi:
            self.x[2][0] += 2*np.pi