import math
import numpy as np
from mpmath import sec
import matplotlib.pyplot as plot


def rotate_point_2D(p, angle):
    pprime = np.array([p[0] * math.cos(angle) - p[1] * math.sin(angle), p[0] * math.sin(angle) + p[1]])
    return pprime


def point_line_dist(lp1, lp2, point):
    a = lp2 - lp1
    b = lp1 - point
    d = np.linalg.norm(np.cross(a, b)) / np.linalg.norm(a)
    return d


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


class Engine2DWithGimbal:
    # base coordinates: normal global right-hand x-y with 0 gimbal angle
    def __init__(self,
                 F,
                 theta,
                 omega,
                 alpha,
                 m=4.12,
                 d_pivot=0.225+0.0625,
                 Ig=4.558*10**2,  # from fusion 360
                 mdot=1.5,
                 v_exhaust=2000,
                 r_engine=0.076,
                 h_topring=-0.225,  # 62.5 is axial dist between CoG and top ring, 225 is between top ring and pivot,
                 h_mount=0.01,
                 r_mount=0.095,
                 ):
        self.F = F  # actuator force, N
        self.theta = theta  # gimbal angle, rad
        self.omega = omega  # gimbal angular velocity, rad/s
        self.alpha = alpha  # gimbal angular acceleration, rad/s^2
        self.m = m  # engine mass, kg
        self.d_pivot = d_pivot  # distance between CoG and pivot point, m
        self.Ip = Ig + self.m * self.d_pivot**2  # rotational inertia of engine about pivot point, kgm^2
        self.mdot = mdot  # propellant mass flow rate, kg/s
        self.v_ex = v_exhaust  # propellant exhaust velocity, m/s
        self.r_engine = r_engine  # radius of the actuator engine mounts, m
        self.h_topring = h_topring  # axial distance between pivot point and actuator engine mounts, m
        self.h_mount = h_mount  # axial distance between pivot point and actuator stationary mounts, m
        self.r_mount = r_mount  # radius of the actuator stationary mounts, m

    def engine_mounting_point(self):
        p = [self.r_engine, self.h_topring]
        return rotate_point_2D(p, self.theta)

    def stand_mounting_point(self):
        p = [self.r_mount, self.h_mount]
        return rotate_point_2D(p, self.theta)

    def actuator_moment(self):
        # actuator moment about pivot point, direction based on right hand rule, i.e. anticlockwise is positive
        moment_arm = point_line_dist(self.engine_mounting_point(), self.stand_mounting_point(), np.array([0,0]))
        return - self.F * moment_arm






act1 = Actuator(force=600, vmax=30)

print(act1.rod_rpm())






