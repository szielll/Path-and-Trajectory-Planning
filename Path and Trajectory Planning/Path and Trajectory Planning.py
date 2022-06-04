# Path and Trajectory Planning

import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

# link lenths in mm
a1 = float(input("a1 = "))
a2 = float(input("a2 = "))
a3 = float(input("a3 = "))
a4 = float(input("a4 = "))

# link mm to meters converter
def mm_to_meter(a):
    m = 1000 # 1 meter = 1000 mm
    return a/m

a1 = mm_to_meter(a1)
a2 = mm_to_meter(a2)
a3 = mm_to_meter(a3)
a4 = mm_to_meter(a4)

# link limits converted to meters
lm1 = float(input("lm1 = "))
lm1 = mm_to_meter(lm1)

# Create Links
SCARA_V2 = DHRobot([
    PrismaticDH(0,0,(180/180)*np.pi,a1,qlim=[0,lm1]),
    PrismaticDH(0,a2,(0/180)*np.pi,0,qlim=[0,0]),
    RevoluteDH(0,a3,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
    RevoluteDH(0,a4,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
], name='SCARA_V2')

print(SCARA_V2)

# degrees to radian converter
def deg_to_rad(T):
    return (T/180.0)*np.pi


# q Paths
q0 = np.array([0,0,0,0])
q1 = np.array([mm_to_meter(float(input("d1 = "))),0,
                deg_to_rad(float(input("T2 = "))),
                deg_to_rad(float(input("T3 = "))),
                ])
q2 = np.array([mm_to_meter(float(input("d1 = "))),0,
                deg_to_rad(float(input("T2 = "))),
                deg_to_rad(float(input("T3 = "))),
                ])
q3 = np.array([mm_to_meter(float(input("d1 = "))),0,
                deg_to_rad(float(input("T2 = "))),
                deg_to_rad(float(input("T3 = "))),
                ])

# Trajectory commands
traj1 = rtb.jtraj(q0,q1,50)
traj2 = rtb.jtraj(q1,q2,50)
traj3 = rtb.jtraj(q2,q3,50)

x1 = -1.0
x2 = 1.0
y1 = -1.0
y2 = 1.0
z1 = -1.0
z2 = 1.0

SCARA_V2.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2])
SCARA_V2.plot(traj2.q,limits=[x1,x2,y1,y2,z1,z2])
SCARA_V2.plot(traj3.q,limits=[x1,x2,y1,y2,z1,z2],block=True)

#SCARA_V2.teach(jointlabels=1)
