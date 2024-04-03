import numpy as np
from math import pi
import math


class Kynematic:
    def __init__(self,
                 length: float,
                 width: float,
                 legs_length: np.ndarray):
        self.length = length
        self.width = width
        self.l1 = legs_length[0]
        self.l2 = legs_length[1]
        self.l3 = legs_length[2]
        self.l4 = legs_length[3]

    def calculate_body_ik(self, omega, phi, psi, xm, ym, zm):
        sHp=np.sin(pi/2)
        cHp=np.cos(pi/2)

        Lo=np.array([0,0,0,1])

        Rx = np.array([[1,0,0,0],
                   [0,np.cos(omega),-np.sin(omega),0],
                   [0,np.sin(omega),np.cos(omega),0],[0,0,0,1]])
        Ry = np.array([[np.cos(phi),0,np.sin(phi),0],
                    [0,1,0,0],
                    [-np.sin(phi),0,np.cos(phi),0],[0,0,0,1]])
        Rz = np.array([[np.cos(psi),-np.sin(psi),0,0],
                    [np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])
        Rxyz=Rx@Ry@Rz

        T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
        Tm = T+Rxyz

        return([Tm @ np.array([[cHp, 0, sHp, -self.length/2],[0, 1, 0, 0],[-sHp, 0, cHp, self.width/2],[0, 0, 0, 1]]),
                Tm @ np.array([[cHp, 0, sHp, self.length/2],[0, 1, 0, 0],[-sHp, 0, cHp, self.width/2],[0, 0, 0, 1]]),
                Tm @ np.array([[cHp, 0, sHp, -self.length/2],[0, 1, 0, 0],[sHp, 0, -cHp, -self.width/2],[0, 0, 0, 1]]),
                Tm @ np.array([[cHp, 0, sHp, self.length/2],[0, 1, 0, 0],[sHp, 0, -cHp, -self.width/2],[0, 0, 0, 1]])
                ])

    def legIK(self, point: np.ndarray):
        (x,y,z)=(point[0],point[1],point[2])
        F=math.sqrt(x**2+y**2-self.l1**2)
        G=F-self.l2  
        H=math.sqrt(G**2+z**2)
        theta1=-math.atan2(y,x)-math.atan2(F,-self.l1)
        
        D=(H**2-self.l3**2-self.l4**2)/(2*self.l3*self.l4)
        theta3=math.acos(D) 
        
        theta2=math.atan2(z,G)-math.atan2(self.l4*math.sin(theta3),self.l3+self.l4*math.cos(theta3))
        
        return np.array([theta1,theta2,theta3])

    def calculate_leg_point(self, thetas: np.ndarray):
        sin = np.sin(thetas)
        cos = np.cos(thetas)
        cos12 = np.cos(thetas[1] + thetas[2])
        sin12 = np.sin(thetas[1] + thetas[2])

        t0=np.array([0,0,0,1])
        t1 = t0 + np.array([-self.l1 * cos[0], self.l1 * sin[0], 0, 0])
        t2 = t1 + np.array([-self.l2 * sin[0], -self.l2 * cos[0], 0, 0])
        t3 = t2 + np.array([-self.l3 * sin[0] * cos[1],
                            -self.l3 * cos[0] * cos[1],
                            self.l3 * sin[1],
                            0])
        t4 = t3 + np.array([-self.l4 * sin[0] * cos12,
                            -self.l4 * cos[0] * cos12,
                            self.l4 * sin12,
                            0])              
        return np.array([t0,t1,t2,t3,t4])
    
    def get_angles(self, leg_position, orientation, center):
        (omega,phi,psi) = orientation
        (xm,ym,zm) = center
        body2legs_mats = self.calculate_body_ik(omega, phi, psi, xm, ym, zm)
        angles = np.zeros((4, 3))
        extended_leg_position = np.ones((4, 4))
        extended_leg_position[:, :3] = leg_position.T
        for i, (mat, pos) in enumerate(zip(body2legs_mats, extended_leg_position)):
            angles[i] = self.legIK(np.linalg.inv(mat) @ pos)
        return angles
