import numpy as np

def compute_gtg_control(x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the linear and angular speed to reach a given goal 
        #This functions receives the goal's position (x_target, y_target) [m] 
        #  and robot's position (x_robot, y_robot, theta_robot) [m, rad] 
        #This functions returns the robot's speed (v, w) [m/s] and [rad/s] 

        kvmax = 0.2 #linear speed maximum gain  
        kwmax = 1.0 #angular angular speed maximum gain 

        av = 1.0 #Constant to adjust the exponential's growth rate   
        aw = 2.0 #Constant to adjust the exponential's growth rate 

        ed = np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2) 

        #Compute angle to the target position 

        theta_target = np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta = theta_target-theta_robot 

        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
        #Compute the robot's angular speed 
        kw = kwmax*(1-np.exp(-aw*e_theta**2))/abs(e_theta) #Constant to change the speed  
        w = kw*e_theta 

        if abs(e_theta) > np.pi/8: 
            #we first turn to the goal 
            v = 0 #linear speed  

        else: 
            # Make the linear speed gain proportional to the distance to the target position 
            kv = kvmax*(1-np.exp(-av*ed**2))/abs(ed) #Constant to change the speed  
            v = kv*ed #linear speed  

        return v, w