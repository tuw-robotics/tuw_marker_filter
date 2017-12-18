'''
Created on Jan 6, 2017

@author: max
'''

import numpy as np
import math


def angle_difference(alpha0, angle1):
    return math.atan2(math.sin(alpha0-angle1), math.cos(alpha0-angle1))
            
class Vehicle(object):
    '''
    classdocs
    '''
    def __init__(self):
        '''
        Constructor
        '''
    
        self.dt = 0.1
        self.alpha = np.array([2, 0.5 , 2, 0.2])
        self.P = np.matrix([[ 0.3, 0 , 0],[ 0, 0.3 , 0],[ 0, 0 , 0.1]])
        self.Q = np.matrix( [[ 2,  0,  0],  [ 0,  1, 0],   [ 0, 0, 1]]);    
    
    def set_odom(self, pose):
        if hasattr(self, 'x') == False: 
            self.x = pose
        self.odom = pose
        
    def define_map(self, m):
        #self.m = m
        return
        
    def get_marker(self, nr):
        if hasattr(self, 'm') == False: 
            return [False, 0, 0]
        for i in range(self.m.shape[0]): # 
            if nr == int(self.m[i,0]):
                return [True, self.m[i,1], self.m[i,2]]
        return [False, 0, 0]
    
    def prediction(self, u):
        if hasattr(self, 'x') == False: 
            return False

        dt = self.dt
        v = u[0,0]
        w = u[1,0]
        theta = self.x[2,0]
        s = np.sin(theta)
        c = np.cos(theta)
        s_dt = np.sin(theta + w * dt)
        c_dt = np.cos(theta + w * dt)
        
        M = np.matrix( [[self.alpha[0]*v*v + self.alpha[1]*w*w, 0], [0, self.alpha[2]*v*v + self.alpha[3]*w*w]]);
        G = np.matrix( [[0, 0, 0], [0, 0, 0], [0, 0, 0]]);
        V = np.matrix( [[0, 0], [0, 0]]);
        if(np.fabs(w) > 0.0):
            r = v/w;
            dx = -r * s + r * s_dt
            dy = +r * c - r * c_dt
            da = w * dt
            
            G = np.matrix( [[1, 0, -r * c + r * c_dt], 
                            [0, 1, -r * s + r * s_dt], 
                            [0, 0,             1]]);
            
            V = np.matrix( [[( -s + s_dt ) /w,  r * ( s - s_dt ) / w + r * c_dt * dt], 
                            [(  c - c_dt ) /w, -r * ( c - c_dt ) / w + r * s_dt * dt],
                            [               0,                                    dt]]);
        else:
            dx = v * np.cos(theta) * dt
            dy = v * np.sin(theta) * dt
            da = 0.0
            
            G = np.matrix( [[1, 0, -v * s * dt], 
                            [0, 1,  v * c * dt], 
                            [0, 0,           1]]);
            
            V = np.matrix( [[dt * c, -0.5 * dt * dt * v * s], 
                            [dt * s,  0.5 * dt * dt * v * c],
                            [     0,                     dt]]);
        
        self.x[0:3,0] = self.x[0:3,0] + np.matrix([[ dx, dy , da]]).transpose()  
        P_last = np.copy(self.P[0:3, 0:3])
        self.P[0:3, 0:3] = G * P_last * G.transpose() + V * M * V.transpose() 
        
        return True
        
    def add_marker(self, z, pos):
        if hasattr(self, 'x') == False: 
            return False
        x = self.x[0,0]
        y = self.x[1,0]
        theta = self.x[2,0]
        s = np.sin(theta);
        c = np.cos(theta);
        zx = c * z[0,1] + -s * z[0,2] + x
        zy = s * z[0,1] +  c * z[0,2] + y
        ztheta = z[0,3] + theta
        zw = np.matrix([[z[0,0], zx, zy , ztheta]])
        if hasattr(self, 'm'):
            self.m = np.concatenate((self.m,zw), axis=0)
        else:
            self.m = zw
        print "Map"
        print self.m
                 
        
    def measurments(self, z):
        if hasattr(self, 'x') == False: 
            return False
        for i in range(len(z)):
            self.corretion(z[i,:])
            [nr, pos, obserations] = self.associate(z[i,0])  
            if obserations == 1 :
                self.add_marker(z[i,:], pos)
            print [nr, pos, obserations]
                    

    #  @param nr MarkerID, nr < 0 means no id
    #  @return [nr, pos, obserations]
    def associate(self, nr):
        nr = long(nr)
        if nr < 0 :
            #print "no data association"
            return np.array([nr, -1, -1], dtype=np.int32)
        if hasattr(self, 'association'): 
            for i in range(len(self.association)):
                if nr == self.association[i,0]:
                    self.association[i,2] = self.association[i,2] + 1
                    return self.association[i,:]            
            self.association = np.append(self.association, [[nr,len(self.association), 0]], axis=0) 
        else :
            self.association = np.array([[nr,0, 0]], dtype=np.long)
        return self.associate(nr)  
                
    def corretion(self, z):
        if hasattr(self, 'x') == False: 
            return False
        
        nr = int(z[0,0]);        
        [exists, mx, my] = self.get_marker(nr)   
        if exists == False: 
            return False      
        
        [zx, zy] = [z[0,1], z[0,2]]           # measurment
        zt = math.atan2(zy, zx)
        zr = math.sqrt(zx*zx + zy*zy)
        [ux, uy, ut] = [self.x[0,0], self.x[1,0], self.x[2,0]] # predicted robot pose
        dx = mx - ux
        dy = my - uy
        mq = dx * dx + dy * dy
        mr = math.sqrt(mq)
        mt = angle_difference(math.atan2(dy, dx), ut)
        
        H = np.matrix( [[-dx/mr, - dy/mr,  0], 
                        [ dy/mq ,  -dx/mq , -1], 
                        [    0 ,      0 ,  0]]);
        
        
        S = H * self.P * H.transpose() + self.Q
        K = self.P * H.transpose() * np.linalg.inv(S)
        mz = np.matrix([[mr], [mt], [1]])
        rz = np.matrix([[zr], [zt], [1]])
        self.x = self.x + K * (rz - mz)
        self.P = (np.identity(3) - K * H) * self.P
        
                
        print ([mr, zr, mt, zt])
        return True    
