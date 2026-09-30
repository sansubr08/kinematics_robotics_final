
''' ####################
    EN605.613 - Introduction to Robotics
    Assignment 4
    Vehicle Kinematics
    -------
    For this assignment you must implement the forward and inverse kinematics functions for the Mechanum class.

    ==================================
    Copyright 2024,
    The Johns Hopkins University Whiting School of Engineering
    All Rights Reserved.
    #################### '''

import numpy as np

class Mecanum:
    """
    Kinematic implementation of a robot with 4 Mecanum wheels.
    Assume all wheels have identical sizes and pitch angles.
    Assume the body is symmetrical with the center of mass in the center.

    Wheel order used throughout this class: [front-left, front-right, back-left, back-right].
    """

    def __init__(self,length,width,wheel_radius,roller_angle,roller_radius):
        """
        Constructor for the Mecanum kinematics class. Sets the vehicle properties.

        Input
        :param length: The length of the vehicle
        :param width: The width of the vehicle
        :param wheel_radius: The radius of the wheels
        :param roller_angle: The pitch angle of the rollers on each wheel
        :param roller_radius: The radius of the rollers on each wheel

        """
        self.L = length
        self.W = width
        self.R = wheel_radius
        self.alpha = roller_angle
        self.r = roller_radius

    def forward(self,x,u):
        """
        Computes the forward kinematics for the Mecanum system.

        Input
        :param x: The starting state (position) of the system. This is [x,y,theta].
        :param u: The control input to the system. This is the wheel rates for each wheel [psi_1,psi_2,psi_3,psi_4]

        Output
        :return: v: The resulting velocity vector for the system. This is [Vx,Vy,Vtheta]

        """
        psi_fl, psi_fr, psi_bl, psi_br = u

        cot_a = 1.0 / np.tan(self.alpha)
        K = (self.W + self.L * cot_a) / 2.0

        # Body-frame velocity: this is the pseudo-inverse solution of the
        # (over-determined, 4 wheel speeds -> 3 DOF) wheel-speed equations.
        # The three columns of that system are mutually orthogonal, so the
        # least-squares solution decouples into these three independent sums.
        Vx_body = (self.R / 4.0) * (psi_fl + psi_fr + psi_bl + psi_br)
        Vy_body = (self.R / (4.0 * cot_a)) * (-psi_fl + psi_fr + psi_bl - psi_br)
        w = (self.R / (4.0 * K)) * (-psi_fl + psi_fr - psi_bl + psi_br)

        # Rotate the body-frame velocity into the world frame using the
        # vehicle's current heading (x[2]) so that the returned [Vx,Vy,Vtheta]
        # can be directly integrated into the world-frame state x.
        theta = x[2]
        c, s = np.cos(theta), np.sin(theta)
        Vx = c * Vx_body - s * Vy_body
        Vy = s * Vx_body + c * Vy_body

        return np.array([Vx, Vy, w])

    def inverse(self,x,v):

        """
        Computes the inverse kinematics for the Mecanum system.

        Input
        :param x: The starting state (position) of the system.This is [x,y,theta].
        :param v: The desired velocity vector for the system. This is [Vx,Vy,Vtheta]

        Output
        :return: u: The necessary control inputs to achieve the desired velocity vector.This is the wheel rates for each wheel [psi_1,psi_2,psi_3,psi_4]

        """
        Vx, Vy, w = v

        # Rotate the desired world-frame velocity into the body frame using
        # the vehicle's current heading (x[2]) before computing wheel rates.
        theta = x[2]
        c, s = np.cos(theta), np.sin(theta)
        Vx_body = c * Vx + s * Vy
        Vy_body = -s * Vx + c * Vy

        cot_a = 1.0 / np.tan(self.alpha)
        K = (self.W + self.L * cot_a) / 2.0

        psi_fl = (Vx_body - cot_a * Vy_body - K * w) / self.R
        psi_fr = (Vx_body + cot_a * Vy_body + K * w) / self.R
        psi_bl = (Vx_body + cot_a * Vy_body - K * w) / self.R
        psi_br = (Vx_body - cot_a * Vy_body + K * w) / self.R

        return np.array([psi_fl, psi_fr, psi_bl, psi_br])


def main():

    x0 = np.array([0,0,0])

    length = 0.3
    width =  0.15
    wheel_radius = 0.05
    roller_radius = 0.01
    roller_angle = 45/180*np.pi

    mecanum = Mecanum(length,width,wheel_radius,roller_angle,roller_radius)

    u0 = np.array([2,2,2,2])

    print(f'Simulating wheel inputs of {u0} for 3 seconds')
    dt = 0.1
    t = 0
    x = np.array(x0, dtype=float)
    while t<3:
        v = mecanum.forward(x,u0)
        x = x + v * dt
        t+=dt
        print(f'{t:.1f}:{x}')

    v_desired = np.array([0,0,45/180*np.pi])
    print(f'Rotating in place with {v_desired} for 1 seconds')
    t=0
    while t<1:
        u = mecanum.inverse(x,v_desired)
        v = mecanum.forward(x,u)
        x = x + v * dt
        t+=dt
        print(f'{t:.1f}:{x}')

    v_desired = np.array([0.5,0.5,0])
    print(f'Translating with  {v_desired} for 2 seconds')
    t=0
    while t<2:
        u = mecanum.inverse(x,v_desired)
        v = mecanum.forward(x,u)
        x = x + v * dt
        t+=dt
        print(f'{t:.1f}:{x}')

if __name__ == '__main__':
    main()
