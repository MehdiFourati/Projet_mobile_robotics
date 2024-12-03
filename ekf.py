import numpy as np
import time
import matplotlib.pyplot as plt
wheel_base = 90 #mm


#Constants
LEFT_WHEEL_VAR = 8.5
RIGHT_WHEEL_VAR = 10.9
ANGLE_VAR = 0.05
POSX_VAR = 5
POSY_VAR = 5.2



class ExtendedKalmanFilter:
    
    #how to initialize the matrices ???
    def __init__(self,x0=np.array([0,0,0,0,0],dtype=float)):
        #[position =(posx,posy),orientation,speed=(speed_left,spped_right)]
        self.x = x0
        
        #measurement matrix
        self.H_camera = np.eye(5)
        #measurement matrix without camera
        self.H_no_cam = np.array([[0,0,0,1,0],
                                  [0,0,0,0,1]])
       
        
        #set time
        self.set_time_t(time.time())
        #process noise covariance matrix 
        self.Q = np.diag([5*2, 5.2*2, 0.05*4, 8.5*4, 10.9*4]) 
        #self.Q = np.eye(5) 

        #measurement noise covariance matrix
        self.R_camera = np.diag([POSX_VAR, POSY_VAR, ANGLE_VAR, LEFT_WHEEL_VAR, RIGHT_WHEEL_VAR])  
  
        self.R_no_cam = np.diag([LEFT_WHEEL_VAR, RIGHT_WHEEL_VAR])  
        #initialize the state covariance matrix)
        self.P = self.Q
        
    def last_time_t(self):
        return self.t
    
    def set_time_t(self,t):
        self.t = t
        
    def state_transition_f(self,dt):
        angle = float(self.x[2])
        wheel_l = float(self.x[3])
        wheel_r = float(self.x[4])


        
        angular_velocity = (wheel_r - wheel_l)/wheel_base
        linear_velocity = (wheel_r + wheel_l)/2
        a_priori_estimate = np.array([
        float(self.x[0]) + np.cos(angle) * linear_velocity * dt,  #posx
        float(self.x[1]) + np.sin(angle) * linear_velocity * dt,  #posy
        float(self.x[2]) + angular_velocity * dt,                #angle
        float(self.x[3]),
        float(self.x[4])

    ])

        return a_priori_estimate
    
    def jacobian_f(self,dt):
        angle = float(self.x[2])
        wheel_l = float(self.x[3])
        wheel_r = float(self.x[4])
        
        #angular_velocity = (wheel_r - wheel_l)/wheel_base
        linear_velocity = (wheel_r + wheel_l)/2


        
        jacobian = np.array([
        [1, 0, np.sin(angle) * linear_velocity * dt, -0.5 * np.cos(angle) * dt, -0.5 * np.cos(angle) * dt],  #d(posx)/dx
        [0, 1,  np.cos(angle) * linear_velocity * dt, 0.5 * np.sin(angle) * dt, 0.5 * np.sin(angle) * dt],  #d(posy)/dy
        [0, 0, 1, -dt / (wheel_base), dt / (wheel_base)],  #d(angle)/dtheta (maybe divide by 2)
        [0, 0, 0, 1, 0],  
        [0, 0, 0, 0, 1]   
    ],dtype=float)

        return jacobian
        
        
        
        
    def predict(self,dt):
        #predict the state
        a_priori_x = self.state_transition_f(dt)
        #predict the state covariance matrix
        jacobian = self.jacobian_f(dt)
        a_priori_P = jacobian @ self.P @ jacobian.T + self.Q
        #make the angle between -pi and pi
        a_priori_x[2] = (a_priori_x[2] + np.pi) % (2 * np.pi) - np.pi
        self.x = a_priori_x
        self.P = a_priori_P
        return self.x,self.P
    

        
    def update(self,camera_on,z):
        if camera_on:
            #residual y = z - h(x) where z is the measurement
            #z = [posx,posy,angle,speed_left,speed_right]
            y = z -  self.x @ self.H_camera
            #residual covariance matrix
            S = self.H_camera @ self.P @ self.H_camera.T + self.R_camera
            #kalman gain
            K = self.P @ self.H_camera.T @ np.linalg.inv(S)
            #state update (a posteriori estimate)
            self.x = self.x + K @ y
            #state covariance update
            self.P = self.P - K @ self.H_camera @ self.P
        else:
            #residual y = z-h(x)
            #z = [speed_left,speed_right]
            y = z -  self.x @ self.H_no_cam
            #residual covariance matrix
            S = self.H_no_cam @ self.P @ self.H_no_cam.T + self.R_no_cam
            #kalman gain
            K = self.P @ self.H_no_cam.T @ np.linalg.inv(S)
            #state update
            self.x = self.x + K @ y
            #state covariance update
            self.P = self.P - K @ self.H_no_cam @ self.P
        #make the angle between -pi and pi
        self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi
        return self.x,self.P
    
    
def apply_kalman(kalman: ExtendedKalmanFilter(np.array([0,0,0,0,0])),camera_on,position=None,speed=None,dt=None):
    #if position is None/ [0,0,0] then camera is off:
    #camera_on = position is not None
    if dt is None:
        dt = time.time() - kalman.last_time_t()
        kalman.set_time_t(time.time())
    if camera_on:
        z = np.array([position[0],position[1],position[2],speed[0],speed[1]])
       
    else:
        z= np.array([speed[0],speed[1]])

        
    #predict the state
    kalman.predict(dt)
    #update the state
    x,P = kalman.update(camera_on,z)
    x = [int(x[i]) for i in range(5) if i != 2]
    print(x)
    return x,P
    
        
        

  

    
