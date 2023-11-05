import math
import numpy as np
from mpmath import sec
import matplotlib.pyplot as plot


class Actuator:
    #  additional resource to consider: https://forum.ansys.com/uploads/attachments/3e825b35-df61-45f5-9693-a81101684b62/6f8fd9d8-ea1d-4478-9a91-aa81014cd231_nut-factor.pdf
    def __init__(self, force, vmax, scd=30, ratio=3, b=30*np.pi/180, t=0.8, d=4.48, mu=0.24):
        self.w = force  # actuator force in N
        self.vmax = vmax  # maximum actuator speed in mm/s
        self.scd = scd  # distance between motor shaft and actuator shaft in mm
        self.ratio = ratio # ratio between motor and actuator shaft
        self.b = b  # metric thread angle in rad
        self.t = t  # thread pitch (currently t = 0.8 for M5) in mm OR screw lead
        self.d = d  # pitch centre diameter of thread (currently d = 4.48 for M5) in mm
        self.mu = mu  # thread coefficient of friction (conservative value)

    def rod_torque_nut(self):
        # using equation D.1.1 in mecheng data formula book with V-thread modification
        # this is for lead screws; for ball screws use the one below
        tanphi = self.t/(np.pi*self.d)
        p = self.w * (tanphi + self.mu * (1 / math.cos(self.b))) / (1 - self.mu * (1 / math.cos(self.b)) * tanphi)
        return p * self.d * 10**(-3) / 2

    def rod_torque(self):
        # https://my.mech.utah.edu/~me7960/lectures/Topic4-BallscrewCalculations.pdf
        # efficiency is estimated at 0.9 (https://monroeengineering.com/blog/pros-and-cons-of-using-ball-screws/#:~:text=Ball%20screws%20offer%20a%20high,just%2020%25%20to%2025%25.)
        # screw lead = pitch when the thread is single start i.e. 1 pitch per rotation (https://www.thomsonlinear.com/en/support/tips/difference-between-screw-pitch-and-lead)
        return self.w * self.t * 10**(-3) / (2 * np.pi * 0.9)

    def motor_torque(self):
        return self.rod_torque() / self.ratio

    def rod_rpm(self):
        return 60 * self.vmax / self.t

    def motor_rpm(self):
        return self.rod_rpm() * self.ratio

    def motor_power(self):
        return self.motor_torque() * (self.motor_rpm()*2*np.pi/60)


act1 = Actuator(force=600, vmax=30)

print(act1.rod_rpm())






