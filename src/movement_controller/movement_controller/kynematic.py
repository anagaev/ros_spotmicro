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

    def legIK(self, point: np.ndarray, leg_index: int):
        (x,y,z)=(point[0],point[1],point[2])
        D = (x**2 + y**2 + z**2 - self.l1**2 - self.l2**2 - self.l3**2)/(2*self.l2*self.l3)

        if leg_index < 2:
            theta3 = math.atan2(math.sqrt(1-D**2),D)
        else:
            theta3 = math.atan2(-math.sqrt(1-D**2),D)
        
        theta2 = math.atan2(z, math.sqrt(x**2 + y**2 - self.l1**2)) - math.atan2(self.l3*math.sin(theta3), self.l2 + self.l3*math.cos(theta3))  

        # After using the equations, there seem to be two errors:
        #   1. The first y4 should not have a negative sign
        #   2. The entire equation should be multiplied by -1
        # The equation for q1 below reflects these changes 
        theta1 = math.atan2(y, x) + math.atan2(math.sqrt(x**2 + y**2 - self.l1**2), -self.l1)
        
        return np.array([theta1,theta2,theta3])

    def calculate_leg_point(self, thetas: np.ndarray):
        sin = np.sin(thetas)
        cos = np.cos(thetas)
        
        t01 = np.array([
            [cos[0], -sin[0], 0.0, -self.l1 * cos[0]],
            [sin[0], cos[0], 0.0, -self.l1 * sin[0]],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        t12 = np.array([[ 0,  0, -1,  0],
                     [-1,  0,  0,  0],
                     [ 0,  1,  0,  0],
                     [ 0,  0,  0,  1]])
        
        t23 = np.array([
            [cos[1], -sin[1], 0.0, self.l2 * cos[1]],
            [sin[1], cos[1], 0.0, self.l2 * sin[1]],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        t34 = np.array([
            [cos[2], -sin[2], 0.0, self.l3 * cos[2]],
            [sin[2], cos[2], 0.0, self.l3 * sin[2]],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        t0 = np.array([0.0, 0.0, 0.0, 1.0])
        t1 = (t01 @ t12)[:4, -1]
        t2 = t1
        t3 = (t01 @ t12 @ t23)[:4, -1]
        t4 = (t01 @ t12 @ t23 @ t34)[:4, -1]
        return np.array([t0, t1, t2, t3, t4])
    
    def get_angles(self, leg_position, orientation, center):
        (omega,phi,psi) = orientation
        (xm,ym,zm) = center
        body2legs_mats = self.calculate_body_ik(omega, phi, psi, xm, ym, zm)
        angles = np.zeros((4, 3))
        extended_leg_position = np.ones((4, 4))
        extended_leg_position[:, :3] = leg_position.T
        for i, (mat, pos) in enumerate(zip(body2legs_mats, extended_leg_position)):
            angles[i] = self.legIK(np.linalg.inv(mat) @ pos, 0)
        return angles
