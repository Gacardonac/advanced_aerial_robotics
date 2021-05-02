#MagicBox
#The magic box is a cuboid object that can generate any force and torque.
#Run the scene magicbox.ttt in CoppeliaSim

#Check if python is connecting to Coppelia
import sim
import numpy as np
import math
import time
import matplotlib.pyplot as plt


sim.simxFinish(-1)  # Close opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

if clientID != -1:
    print('Connected')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)

    print('Simulation time in milliseconds: ', sim.simxGetLastCmdTime(clientID))
    
    # Get Object position
    name = 'MagicBox'
    err_code, cuboid = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_blocking)
    res, position = sim.simxGetObjectPosition(clientID, cuboid, -1, sim.simx_opmode_blocking)        
    print(name, 'is at [x,y,z]=', position)
    
    #res, position = sim.simxAddForce(clientID, name,[0,0,10],[0,0,0])
    

    res = sim.simxSetFloatSignal(clientID,'fx', 0.0,sim.simx_opmode_oneshot_wait)
    print(res)
    
    # Now close the connection to CoppeliaSim:
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
    print('Disconnected')
else:
    print('Failed connecting to remote API server')

##################################################################
########################### ROBOT ################################
##################################################################
class robot():
    
    def __init__(self, frame_name, motor_names=[], client_id=0):  
        # If there is an existing connection
        if client_id:
                self.client_id = client_id
        else:
            self.client_id = self.open_connection()
            
        self.motors = self._get_handlers(motor_names) 
        
        # Robot frame
        self.frame =  self._get_handler(frame_name)
            
        
    def open_connection(self):
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim 
        
        if clientID != -1:
            print('Robot connected')
        else:
            print('Connection failed')
        return clientID
        
    def close_connection(self):   
        self.wrench([0, 0, 0, 0, 0, 0]) 
        sim.simxGetPingTime(self.client_id)  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        sim.simxFinish(self.client_id)  # Now close the connection to CoppeliaSim:
        print('Connection closed')
    
    def isConnected(self):
        c,result = sim.simxGetPingTime(self.client_id)
        # Return true if the robot is connected
        return result > 0         
        
    def _get_handler(self, name):
        err_code, handler = sim.simxGetObjectHandle(self.client_id, name, sim.simx_opmode_blocking)
        return handler
    
    def _get_handlers(self, names):
        handlers = []
        for name in names:
            handler = self._get_handler(name)
            handlers.append(handler)
        
        return handlers

    def send_motor_velocities(self, vels):
        for motor, vel in zip(self.motors, vels):
            err_code = sim.simxSetJointTargetVelocity(self.client_id, 
                                                      motor, vel, sim.simx_opmode_streaming)      
            
    def set_position(self, position, relative_object=-1):
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)        
        sim.simxSetObjectPosition(clientID, self.frame, relative_object, position, sim.simx_opmode_oneshot)                
        
    def simtime(self):
        return sim.simxGetLastCmdTime(self.client_id)
    
    def get_position(self, relative_object=-1):
        # Get position relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, position = sim.simxGetObjectPosition(self.client_id, self.frame, relative_object, sim.simx_opmode_blocking)        
        return np.array(position)
    
    
    
    def get_velocity(self, relative_object=-1):
        # Get velocity relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, velocity, omega = sim.simxGetObjectVelocity(self.client_id, self.frame, sim.simx_opmode_blocking)        
        return np.array(velocity), np.array(omega)
    
    
    
    def get_object_position(self, object_name):
        # Get Object position in the world frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, -1, sim.simx_opmode_blocking)
        return np.array(position)
    
    def get_object_relative_position(self, object_name):        
        # Get Object position in the robot frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, self.frame, sim.simx_opmode_blocking)
        return np.array(position)
    
    def set_signal(self, signal, value):
        return sim.simxSetFloatSignal(clientID, signal, value, sim.simx_opmode_oneshot_wait)
        
    def wrench(self, w):
        names = ['fx', 'fy', 'fz', 'Mx', 'My', 'Mz']
        for ni, wi in zip(names, w):
            self.set_signal(ni, wi)
        
    def get_orientation(self, relative_object=-1):
    # Get position relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, euler = sim.simxGetObjectOrientation(self.client_id, self.frame, relative_object, sim.simx_opmode_blocking)        
        return np.array(euler)

# Send force to a robot
r = robot('MagicBox')  # Create an instance of our robot
r.wrench([0,0,0,0,0,0])

r.close_connection()  

######################################################################
######################### PD-CONTROL #################################
#####################################################################
# # Send force to a robot
r = robot('MagicBox')  # Create an instance of our robot
d = robot('DesiredBox')

m = 8 #kg mass of the block plus the mass of the prop
g = 9.81
I = 80

log = []
log2 = []
try:
    
    while True:
        # Robot state
        p = r.get_position()
        rad1 = r.get_orientation()
        theta = rad1
        v, ω = r.get_velocity()
        # Desired state
        p_d = d.get_position()
        rad2 = d.get_orientation()
        theta_d = (rad2)
    
        #print(theta_d)
        v_d, ω_d = d.get_velocity()
    
    
        ep = p_d - p
        ev = v_d - v
        eω = ω_d - ω
        etheta = theta_d - theta
    
        kp, kd = 0.03, 0.2
        a = kp * ep + kd * ev
        kp, kd = 0.03, 0.3
        a1 = kp*etheta + kd*eω
    
        '''
        Alternative: YOu could use get quaternions then use a from quaternions to euler to get the euler angles
        '''
        #Rotation Matrix
        rx = np.array([[1,0,0],[0,math.cos(theta[0]),math.sin(-theta[0])],[0,math.sin(theta[0]),math.cos(theta[0])]])
        ry = np.array([[math.cos(theta[1]),0,math.sin(theta[1])],[0,1,0],[math.sin(-theta[1]),0,math.cos(theta[1])]])
        rz = np.array([[math.cos(theta[2]),math.sin(-theta[2]),0],[math.sin(theta[2]),math.cos(theta[2]),0],[0,0,1]])
    
        #Multiplication of roation matrix
        tait_bryan = rz*ry*rx
        inv = np.linalg.inv(tait_bryan) 
        f = m * a
        f[2] += m*g
        # 1- Change the force from world frame to body frame
        force = np.dot(inv,f)
        torque = a1*0.00667
    
        r.wrench([force[0], force[1], force[2], torque[0],torque[1],torque[2]])
    
    
        log.append(ep)
        log2.append(etheta)
    
        time.sleep(0.01)
except KeyboardInterrupt:
    r.close_connection()
    d.close_connection()
    
    plot1 = plt.figure(1)
    log = np.array(log)
    plt.plot(log[:,0], label='$e_x$')
    plt.plot(log[:,1], label='$e_y$')
    plt.plot(log[:,2], label='$e_z$')
    plt.title('Position')
    plt.legend()

    plot2 = plt.figure(2)
    log2 = np.array(log2)
    plt.plot(log2[:,0], label='$e_x$')
    plt.plot(log2[:,1], label='$e_y$')
    plt.plot(log2[:,2], label='$e_z$')
    plt.title('Angular Orientation')
    plt.legend()
plt.show()








































