import math
import numpy as np
from mpmath import sec
import matplotlib.pyplot as plot


class Actuator:
    #  additional resource to consider: https://forum.ansys.com/uploads/attachments/3e825b35-df61-45f5-9693-a81101684b62/6f8fd9d8-ea1d-4478-9a91-aa81014cd231_nut-factor.pdf
    def __init__(self, force, vmax, scd=30, ratio=3, b=30*np.pi/180, t=0.8, d=4.48, mu=0.24):
        self.w = force  # actuator force
        self.vmax = vmax  # maximum actuator speed
        self.scd = scd  # distance between motor shaft and actuator shaft
        self.ratio = ratio # ratio between motor and actuator shaft
        self.b = b  # metric thread angle
        self.t = t  # thread pitch (currently t = 0.8 for M5)
        self.d = d  # pitch centre diameter of thread (currently d = 4.48 for M5)
        self.mu = mu  # thread coefficient of friction (conservative value)

    def rod_torque(self):
        # using equation D.1.1 in mecheng data formula book with V-thread modification
        tanphi = self.t/(np.pi*self.d)
        p = self.w * (tanphi + self.mu * (1 / math.cos(self.b))) / (1 - self.mu * (1 / math.cos(self.b)) * tanphi)
        return p * self.d * 10**(-3) / 2

    def motor_torque(self):
        return self.rod_torque() / self.ratio

    def rod_rpm(self):
        return 60 * self.vmax / self.t

    def motor_rpm(self):
        return self.rod_rpm() * self.ratio


act1 = Actuator(force=600, vmax=30)

print(act1.motor_torque())






