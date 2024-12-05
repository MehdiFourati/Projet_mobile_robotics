import numpy as np

DISTANCE_WHEEL = 90 # distance between the wheels in mm
CAMERA_FPS = 30 # fps of the video feed
DISTANCE_MM_S = 1 # distance done in mm/s per 1 PMW

def convert_input(robot):
    """
    Convert B*u, the effect of the system input
    Input:
        robot: an instance of the class Robot
    """
    # convert the speed values of the wheel to pixel
    linear_velocity = (robot.lspeed + robot.rspeed)/2 * DISTANCE_MM_S / CAMERA_FPS * robot.robot_width / DISTANCE_WHEEL
   
    # linear approximation of the angle based on experimentation 
    delta_angle = (0.1667 * (robot.rspeed - robot.lspeed) + 8.33) * np.pi / 180

    angle = float(robot.alpha)
    delta_x = np.sin(angle) * linear_velocity
    delta_y = np.cos(angle) * linear_velocity
    
    return [delta_x, delta_y, delta_angle]

def check_angle(angle):
    """
    Check that the type of the angle is the right one
    Input:
        - angle: angle of the robot
    """
    if isinstance(angle, np.ndarray):
        return float(angle[0])
    else:
        return float(angle)


class KalmanFilter:
    
    def __init__(self, x0):
        self.F = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) # state transition model
        self.H = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) # observation model
        self.Q = np.array([[2.0, 0, 0], [0, 2.0, 0], [0, 0, 0.017]]) # process noise covariance
        self.R = np.array([[5.0, 0, 0], [0, 5.0, 0], [0, 0, 0.15]]) # measurement noise covariance
        self.P = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) # initial error covariance  
        self.x = x0 # initial state, shape [x,y,alpha]

    def predict(self, converted_input):
        """
        Use the Kalman filter to make a prediction of the future state
        Input:
            - converted_input: B*u, the effect of the system input
        """
        self.x = self.F @ self.x + converted_input
        self.P = self.F @ self.P @ self.F.transpose() + self.Q

        # to keep the angle bewteen )-pi;pi]
        if self.x[2][0] > np.pi:
             self.x[2][0] -= 2*np.pi
        elif self.x[2][0] < -np.pi:
            self.x[2][0] += 2*np.pi
 
    def update(self, z):
        """
        Update the Kalman filter using observations to make better estimations
        Input:
            - z: observation of the robot, shape [x,y,alpha]
        """
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.transpose() + self.R   
        K = self.P @ self.H.transpose() @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

        # to keep the angle bewteen )-pi;pi]
        if self.x[2][0] > np.pi:
             self.x[2][0] -= 2*np.pi
        elif self.x[2][0] < -np.pi:
            self.x[2][0] += 2*np.pi