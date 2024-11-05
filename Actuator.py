import math
import numpy as np
import time
from mpmath import sec
import matplotlib.pyplot as plot
from matplotlib.animation import FuncAnimation


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
    def __init__(self, force, vmax, t=2, d=8, range=55, kv=500, V=22.2, A=16):
        self.w = force  # actuator force in N
        self.vmax = vmax  # maximum actuator speed in mm/s
        self.t = t  # thread pitch (currently t = 0.8 for M5) in mm OR screw lead
        self.d = d  # pitch centre diameter of thread (currently d = 4.48 for M5) in mm
        self.range = range  # actuator linear range in mm
        self.kv = kv # motor kv in rpm/V
        self.V = V # motor supplied voltage, 6s = 22.2V
        self.A = A # motor supplied current

    def rod_torque(self):
        # https://my.mech.utah.edu/~me7960/lectures/Topic4-BallscrewCalculations.pdf
        # efficiency is estimated at 0.9 (https://monroeengineering.com/blog/pros-and-cons-of-using-ball-screws/#:~:text=Ball%20screws%20offer%20a%20high,just%2020%25%20to%2025%25.)
        # screw lead = pitch when the thread is single start i.e. 1 pitch per rotation (https://www.thomsonlinear.com/en/support/tips/difference-between-screw-pitch-and-lead)
        return self.w * self.t * 10**(-3) / (2 * np.pi * 0.9)
    
    def motor_torque(self):
        # using kt = 1/kv_si approximation, and T = kt*A
        kv_si = self.kv * 2*np.pi/60
        kt = 1/kv_si
        return kt*self.A

    def force_from_torque(self):
        return 2*np.pi*0.9*self.motor_torque()/(self.t * 10**(-3))

    def range_revs(self):
        return self.range/self.t

    def rpm(self):
        return 60 * self.vmax / self.t

    def motor_power(self):
        #return self.V*self.A
        return self.motor_torque() * (self.rpm()*2*np.pi/60)

act2 = Actuator(force=200, vmax=300)
print("Setup: ML4108 500kv motor with 2mm lead SFK0802 screw, specs: 200N, 0.3 m/s max")
print("Motor torque required for 200N: ", round(act2.rod_torque(),3), "Nm")
print("Motor torque supplied with kv/kt estimation: ", round(act2.motor_torque(),3), "Nm")
print("Max force with estimated max motor torque: ", round(act2.force_from_torque(),0), 'N')
print("Motor rpm needed for specced 0.3m/s max speed: ", act2.rpm(), "RPM")
print("Shaft power required: ", round(act2.motor_power(),0), "W")

class Engine2DWithGimbal:
    # base coordinates: normal global right-hand x-y with 0 gimbal angle
    def __init__(self,
                 F,
                 theta,
                 omega,
                 m=4.12,
                 h_topring=-0.225,
                 d_pivot=0.225+0.0625, # 62.5 is axial dist between CoG and top ring, 225 is between top ring and pivot,
                 Ig=4.558*10**2,  # from fusion 360
                 mdot=1.5,
                 v_exhaust=2000,
                 r_engine=0.076,
                 h_mount=0.01,
                 r_mount=0.095,
                 ):
        self.F = F  # actuator force, N
        self.theta = theta  # gimbal angle, rad
        self.omega = omega  # gimbal angular velocity, rad/s
        self.m = m  # engine mass, kg
        self.h_topring = h_topring  # axial distance between pivot point and actuator engine mounts, m
        self.d_pivot = d_pivot  # distance between CoG and pivot point, m
        self.Ip = Ig + self.m * self.d_pivot**2  # rotational inertia of engine about pivot point, kgm^2
        self.mdot = mdot  # propellant mass flow rate, kg/s
        self.v_ex = v_exhaust  # propellant exhaust velocity, m/s
        self.r_engine = r_engine  # radius of the actuator engine mounts, m
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

    def v_combustion(self):
        # velocity of fluid going axially in combustion chamber
        P = 40 * 100000   # pressure 40 bar
        R = 0.2  # placeholder
        T = 3300  # Temperature
        rho = P/(R*T)
        Qdot = self.mdot / rho
        A = np.pi * 0.06**2
        v = Qdot/A
        return v

    def deltatheta(self):
        l_chamber = 0.2
        return self.v_combustion() * l_chamber * self.omega

    def resistive_moment(self):
        # damping-like moment from inertia of exhaust (need to ask+research+think about this)
        F = self.mdot * self.v_ex * math.sin(self.deltatheta())
        M = F * self.d_pivot
        return M

    def alpha(self):
        Msum = self.actuator_moment() + self.resistive_moment()
        return Msum / self.Ip

    def update(self, dt):
        self.omega = self.omega + dt * self.alpha()
        self.theta = self.theta + dt * self.omega
        return Engine2DWithGimbal(F=self.F, theta=self.theta, omega=self.omega)


def print_state(engine, t):
        print(round(t, 2), " s, ",
              round(engine.theta, 3), " rad, ",
              round(engine.omega, 3), " rad/s, ",
              round(engine.alpha(), 3), " rad/s^2, ",
              round(engine.actuator_moment(), 3), " Nm act., ",
              round(engine.resistive_moment(), 3), " Nm res., "
              )


act1 = Actuator(force=600, vmax=30)



engine = Engine2DWithGimbal(F=600,theta=0,omega=0)

engine2 = Engine2DWithGimbal(F=600,theta=0,omega=0.5)

#print(engine2.actuator_moment())
#print(engine2.resistive_moment())

t = 0
dt = 0.05

fig, ax = plot.subplots()
xdata, ydata = [], []
actuator1, = ax.plot([], [])
ax.set_aspect('equal')

def init():
    ax.set_xlim(0, 2 * np.pi)
    ax.set_ylim(-1, 1)
    #ax.set_xlim(-2*engine.r_mount, 2*engine.r_mount)
    #ax.set_ylim(-2*engine.d_pivot, 2*engine.h_mount)
    return actuator1,

def update(frame):
    xdata.append(frame)
    ydata.append(np.sin(frame))
    actuator1.set_data(xdata, ydata)
    return actuator1,


ani = FuncAnimation(fig, update, frames=np.linspace(0, 30, int(30/dt)),init_func=init, blit=True)
#plot.show()

while t < 30:
    engine = engine.update(dt)
    #print_state(engine, t)
    t += dt
