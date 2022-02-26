# -*- coding: utf-8 -*-
"""
Created on Fri Dec 17 11:10:47 2021

@author: pendl
"""

from sympy import *
import math
import numpy as np
import matplotlib.pyplot as plt
import time
import pylab as pl
import numpy as np
from IPython import display

     
# Assumptions:
# 1. Assuming COM lies at joints
# 2. r is along gravity'
# 3. Robot body symmetric about vertical axis


#Length of Head - Lh, Shoulders - Ls, backbone - Lb, Humerus- Lh, Elbow - Le, Pelvis - Lp, Femur - Lf, Tibia- Lt, Ankle- La
zero = 1e-10
t, p, sl, el, sr, er = symbols('t p sl el sr er')
Lhead = 0.1 
Ls = 0.2    
Lb  = 0.5   
Lh = 0.3    
Le = 0.3    
Lp = 0.3     
Lf = 0.5    
Lt = 0.45   
La = 0.2    

#Transformation matrix T is given by:

def Rz(theta):
    return Matrix([[cos(theta), -sin(theta), 0, 0], 
            [sin(theta), cos(theta), 0, 0], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1]])

def Tz(d):
    return Matrix([[1, 0, 0, 0], 
            [0, 1, 0, 0], 
            [0, 0, 1, d], 
            [0, 0, 0, 1]])

def Tx(a):
    return Matrix([[1, 0, 0, a], 
            [0, 1, 0, 0], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1]])

def Rx(alpha):
    return Matrix([[1, 0, 0, 0], 
            [0, cos(alpha), -sin(alpha), 0], 
            [0, sin(alpha), cos(alpha), 0], 
            [0, 0, 0, 1]])

def Ry(alpha):
    return Matrix([[cos(alpha), 0, sin(alpha), 0], 
            [0, 1, 0, 0], 
            [-sin(alpha), 0, cos(alpha), 0], 
            [0, 0, 0, 1]])




#Transformation of pelvis joint in world frame:
Tworld_p = Rz(pi/2)
Tp_world = transpose(Tworld_p)
                
#Taking Pelvis as origin
Tp = Matrix([[1, 0, 0, 0], 
            [0, 1, 0, 0], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1]])



def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
    
wrist_rpos =[]
def drawRightArm(ax,q):
    global Tp, Tp_sr, Tp_er, Tp_wr, wrist_rpos
# =============================================================================
#     """
#     ax: plot axes
#     q: joint values
#     """
# =============================================================================
    Tsp = Tp
    Tsp_sr = Tp_sr.subs({p: q[0], sr: q[1], er: q[2] })
    Tsp_er = Tp_er.subs({p: q[0], sr: q[1], er: q[2] })
    Tsp_wr = Tp_wr.subs({p: q[0], sr: q[1], er: q[2] })
    wrist_rpos.append(Tsp_wr[0:3,3])

    ax.plot3D(     Tsp[0,3], Tsp[1,3] ,Tsp[2,3], 'r')
    ax.plot3D( [   Tsp[0,3], Tsp_sr[0,3]], [     Tsp[1,3], Tsp_sr[1,3] ], [   Tsp[2,3] ,Tsp_sr[2,3]],'r')
    ax.plot3D( [Tsp_sr[0,3], Tsp_er[0,3]], [  Tsp_sr[1,3], Tsp_er[1,3] ], [Tsp_sr[2,3] ,Tsp_er[2,3]],'g')
    ax.plot3D( [Tsp_er[0,3], Tsp_wr[0,3] ], [ Tsp_er[1,3], Tsp_wr[1,3] ], [Tsp_er[2,3] ,Tsp_wr[2,3]],'b')

wrist_lpos =[]
def drawLeftArm(ax,q):
    global Tp, Tp_sl, Tp_el, Tp_wl, wrist_lpos 
# =============================================================================
#     """
#     ax: plot axes
#     q: joint values
#     """
# =============================================================================
    Tsp = Tp
    Tsp_sl = Tp_sl.subs({p: q[0], sl: q[1], el: q[2] })
    Tsp_el = Tp_el.subs({p: q[0], sl: q[1], el: q[2] })
    Tsp_wl = Tp_wl.subs({p: q[0], sl: q[1], el: q[2] })
    wrist_lpos.append(Tsp_wl[0:3,3])

    ax.plot3D( Tsp[0,3],    Tsp[1,3] ,Tsp[2,3],'r', marker=10)
    ax.plot3D( [   Tsp[0,3], Tsp_sl[0,3]], [    Tsp[1,3], Tsp_sl[1,3] ], [   Tsp[2,3] ,Tsp_sl[2,3]],'r')
    ax.plot3D( [Tsp_sl[0,3], Tsp_el[0,3]], [ Tsp_sl[1,3], Tsp_el[1,3] ], [Tsp_sl[2,3] ,Tsp_el[2,3]],'g')
    ax.plot3D( [Tsp_el[0,3], Tsp_wl[0,3]], [ Tsp_el[1,3], Tsp_wl[1,3] ], [Tsp_el[2,3] ,Tsp_wl[2,3]],'b')

    ax.scatter3D(0.5,0.5,0.5)
    ax.scatter3D(-0.1,-0.2,-0.1)


s=0.1 #Side length of Box
def drawBox(ax3, xc, yc, zc, s):
    ax3.plot3D( xc, yc, zc, 'violet', marker=10)
    # 1 --- 2 
    ax3.plot3D( [xc+s/2, xc-s/2] , [yc+s/2, yc+s/2], [zc-s/2, zc-s/2], 'blue')
    # 2 --- 3
    ax3.plot3D( [xc-s/2, xc-s/2] , [yc+s/2, yc+s/2], [zc-s/2, zc+s/2], 'violet')
    # 3 --- 4
    ax3.plot3D( [xc-s/2, xc+s/2] , [yc+s/2, yc+s/2], [zc+s/2, zc+s/2], 'violet')
    # 4 --- 1
    ax3.plot3D( [xc+s/2, xc+s/2] , [yc+s/2, yc+s/2], [zc+s/2, zc-s/2], 'violet')
    
    # 5 --- 6
    ax3.plot3D( [xc+s/2, xc-s/2] , [yc-s/2, yc-s/2], [zc-s/2, zc-s/2], 'blue')
    # 6 --- 7
    ax3.plot3D( [xc-s/2, xc-s/2] , [yc-s/2, yc-s/2], [zc-s/2, zc+s/2], 'violet')
    # 7 --- 8 
    ax3.plot3D( [xc-s/2, xc+s/2] , [yc-s/2, yc-s/2], [zc+s/2, zc+s/2], 'violet')
    # 8 --- 5
    ax3.plot3D( [xc+s/2, xc+s/2] , [yc-s/2, yc-s/2], [zc+s/2, zc-s/2], 'violet')

    # 5 --- 1
    ax3.plot3D( [xc+s/2, xc+s/2] , [yc-s/2, yc+s/2], [zc-s/2, zc-s/2], 'violet')
    # 8 --- 4
    ax3.plot3D( [xc+s/2, xc+s/2] , [yc-s/2, yc+s/2], [zc+s/2, zc+s/2], 'violet')
    # 6 --- 2
    ax3.plot3D( [xc-s/2, xc-s/2] , [yc-s/2, yc+s/2], [zc-s/2, zc-s/2], 'violet')
    # 7 --- 3
    ax3.plot3D( [xc-s/2, xc-s/2] , [yc-s/2, yc+s/2], [zc+s/2, zc+s/2], 'violet')



def J_inv(Js):
    Jp = ((Js.T*Js)).inv()*Js.T
    return Jp

fig = plt.figure(figsize=plt.figaspect(0.5))
ax3 = fig.add_subplot(1, 2, 2, projection='3d')
ax3.set_xlabel('X')
ax3.set_ylabel('Y')
ax3.set_zlabel('Z')
ax3.axis('auto')


ax3.set_title('Initial configuration of the Robot')



def getT(a,d,th,al):
# =============================================================================
#     """
#     # Get Transformation (a, d, theta-Rz, alpha-Rx)
#     """
# =============================================================================
    return Rz(th)*Tz(d)*Tx(a)*Rx(al)



#Left arm Transformation matrix
# Get Transformation (a, d, theta-Rz, alpha-Rx)
Tp_sl = Tworld_p*getT(Lb, Ls, p+pi, pi)
Tsl_el = getT(Lh, zero, sl, -pi/2)
Tel_wl = getT(Le, zero, el, zero)

Tp_wl = Tp_sl*Tsl_el*Tel_wl

# Right arm Transformation matrix
Tp_sr = Tworld_p*getT(Lb, Ls, p, pi)
Tsr_er = getT(Lh, zero, sr, -pi/2)
Ter_wr = getT(Le, zero, er, zero)

Tp_wr = Tp_sr*Tsr_er*Ter_wr

# Left Wrist position wrt Hip
xl = Tp_wl[0,3]
yl = Tp_wl[1,3]
zl = Tp_wl[2,3]

#Rotation of Z wrt hip
Tp_el = Tp_sl * Tsl_el
zp_p = Matrix([[0],[0],[1]])
zp_el = Tp_sl[0:3,2]
zp_wl = Tp_el[0:3,2]


# Right Wrist position wrt Hip
xr = Tp_wr[0,3]
yr = Tp_wr[1,3]
zr = Tp_wr[2,3]

#Rotation of Z wrt hip
Tp_er = Tp_sr * Tsr_er
zp_p = Matrix([[0],[0],[1]])
zp_er = Tp_sr[0:3,2]
zp_wr = Tp_er[0:3,2]

# Jacobian for left arm
Jl = Matrix( [[diff(xl,p), diff(xl,sl), diff(xl,el)],
            [diff(yl,p),  diff(yl,sl), diff(yl,el)],
            [diff(zl,p),  diff(zl,sl), diff(zl,el)], 
            [zp_p, zp_el, zp_wl]] )
Jl

# Jacobian for right arm
Jr = Matrix( [[diff(xr,p), diff(xr,sr), diff(xr,er)],
            [diff(yr,p),  diff(yr,sr), diff(yr,er)],
            [diff(zr,p),  diff(zr,sr), diff(zr,er)], 
            [zp_p, zp_er, zp_wr]] )
Jr


#Testing Forward Kinematics
ql=Matrix([[0],[0],[0]])
qr=Matrix([[0],[0],[0]])

drawLeftArm(ax3,ql)

drawRightArm(ax3,qr)

drawBox(ax3, xc=0.4, yc=0, zc=0.5, s=s)

# Forward Kinematics equation for Right arm:
Tp_er 

# Forward Kinematics equation for Left arm:
Tp_el

#Checking if Inverse Jacobian is computing for left arm: 
ql = Matrix([ [0.1], [0.1], [0.1]])

Jsl = Jl.subs({p: ql[0], sl: ql[1], el: ql[2]})
pprint(Jsl)
Jpl = J_inv(Jsl)

#Checking if Inverse Jacobian is computing for Rigth Arm: 
qr = Matrix([ [0.1], [0.1], [0.1]])

Jsr = Jr.subs({p: qr[0], sr: qr[1], er: qr[2]})
pprint(Jsr)
Jpr = J_inv(Jsr)



#Reaching the box with constant velocity in X,Y and Z directions
SETLIMITS = False
THRESHOLD_DISTANCE = 0.05

ql = Matrix([ [0.1], [0.1], [0.1]])
qr = Matrix([ [0.1], [0.1], [0.1]])
q_l =  Matrix([ [0.01], [0.01], [0.01]])
q_r =  Matrix([ [0.01], [0.01], [0.01]])

ql_list = [ql]
qr_list = [qr]
q_llist=[]

cx=0.3
cy=0
cz=0.0

dist = 10

fig = plt.figure()
ax3 = plt.axes(projection='3d')
ax3.set_xlabel('X-axis')
ax3.set_ylabel('Y-axis')
ax3.set_zlabel('Z-axis')
ax3.axis('auto')
ax3.set_title('Robot reaching for the box')

distl = 1e2
Kp=0.3
maxV=0.5
while(distl>THRESHOLD_DISTANCE):

    Jsl = Jl.subs({p: ql[0], sl: ql[1], el: ql[2]})
    Jsr = Jr.subs({p: qr[0], sr: qr[1], er: qr[2]})
    try:
        #Using Jacobian Pseudo-inverse, in case Jacobian is non square matrix.
        Jpl = J_inv(Jsl)
    except:
        ql = (ql + q_l).evalf()
        print("Singularity / rank-loss")
        continue

    try:
        #Using Jacobian Pseudo-inverse, in case Jacobian is non square matrix.
        Jpr = J_inv(Jsr)
    except:
        qr = (qr + q_r).evalf()
        print("Singularity / rank-loss")
        continue
    
    #Step 2: inverse velocity kinematics problem that generates joint velocities for a circle
    #Circle equation:
    Tsp_wl = Tp_wl.subs({p: ql[0], sl: ql[1], el: ql[2]})
    Tsp_wr = Tp_wr.subs({p: qr[0], sr: qr[1], er: qr[2]})
    Vxl = cx -  Tsp_wl[0,3].evalf()
    Vyl = cy+s/2 - Tsp_wl[1,3].evalf()
    Vzl = cz - Tsp_wl[2,3].evalf()
    Vxl = min(Vxl, maxV)
    Vyl = min(Vyl, maxV)
    Vzl = min(Vzl, maxV)


    Vxr = cx -  Tsp_wr[0,3].evalf()
    Vyr = cy-s/2 - Tsp_wr[1,3].evalf()
    Vzr = cz - Tsp_wr[2,3].evalf()
    Vxr = min(Vxr, maxV)
    Vyr = min(Vyr, maxV)
    Vzr = min(Vzr, maxV)

    distl = sqrt(Vxl**2 + Vyl**2 + Vzl**2)

    #V is the end effector's desiered linear velocity and angular velocity of end effector rotation.
    # V = [vx, vy, vz, roll', pitch', yaw'] 
    Vl = Matrix([ [Vxl], [Vyl], [Vzl], [0], [0], [0]])
    Vr = Matrix([ [Vxr], [Vyr], [Vzr], [0], [0], [0]])
    q_l=Jpl*Kp*Vl
    q_r=Jpr*Kp*Vr

    ql = (ql + q_l).evalf()
    qr = (qr + q_r).evalf()
    ql_list.append(ql)
    qr_list.append(qr)

    #Step 3: Plug in the join velocities generated in Step 2 to forward velocity kinematics developed in Step 
    # 1 and plot output of the inverse kinematics problem that should be a circle as specified in Figure 1 

    display.clear_output(wait=True)
    display.display(plt.gcf())
    #print("dq left arm: ",q_l)
    #print("dq right arm: ",q_r)
    #print("ql left arm: ",q_l)
    #print("qr left arm: ",q_r)
    #print("End-effector(x,y,z): ", x,y,z)
    #print("Alpha : ", al)

    # Visualize 
    #--------------------


    # 3D plot
    ax3.cla()
    drawLeftArm(ax3,ql)
    drawRightArm(ax3,qr)
    drawBox(ax3, xc=cx, yc=cy, zc=cz, s=0.1)
    ax3.set_xlabel('X-axis')
    ax3.set_ylabel('Y-axis')
    ax3.set_zlabel('Z-axis')
    ax3.set_title('Robot Reaching for the box')
    #--------------------

    time.sleep(0.01)

plt.show()

ql_reach_goal_pos = ql_list[-1]
qr_reach_goal_pos = qr_list[-1]


#Joint positions and Torque:
# =============================================================================
# Step 1- Python code that parametrically calculates matrix g(q)
# Step 2- Python code that parametrically calculates total joint torque (gravity + external force)
# Step 3- If robot does task in  200 seconds, plot the joint torques required over time (between t=0 and t=200 s). 
#(Plot 6 graphs. One of each joint: 1,2, 4, 5, 6, and 7)

# =============================================================================


#Find Torque due to gravity
#Tau = r x F
# or 
# Tau = mgh

g=9.8 # Divide by 100 to convert the arm distance from cm to meters.
Mhead= 2.71
Mbackbone = 13.55
Mshoulder = 5.42
Mhumerous = 8.13
Melbow = 8.13
Mpelvis = 4.065
MFemur = 13.55
Mtibia = 12.195
Mankle = 5.42
Mobject = 4.53592 # 10lbs
Mmotor = 0.453592 # 1lb

# Assumptions:
# 1. Assuming COM lies at joints
# 2. r is along gravity

def GetTau(J,G,side,ql):
    global p, sl, el, sr, er
    Tau = []
    Tau1max = 0
    Tau2max = 0
    Tau3max = 0

    for q in ql:
        Fx = 0
        Fy = 5
        Fz = 0
        F_ext = Matrix([Fx, Fy, Fz, 0, 0, 0])
        if (side == 'l'):
            Js = J.subs({p: q[0], sl: q[1], el: q[2]})
            G_qs = G.subs({p: q[0], sl: q[1], el: q[2]})
        elif (side == 'r'):
            Js = J.subs({p: q[0], sr: q[1], er: q[2]})
            G_qs = G.subs({p: q[0], sr: q[1], er: q[2]})
        
        #Find Torque needed for external force
        Tau_extl = J_inv(Js)*F_ext
        Tau_extl = np.array(Tau_extl)
        Tau_extl[Tau_extl>20]=20
        Tau_extl[Tau_extl<-35]=-35
        G_qs = np.array(G_qs.evalf())
        Tau_net= (Tau_extl+G_qs)
        Tau1max = abs(Tau_net[0]) if abs(Tau_net[0])>Tau1max else Tau1max
        Tau2max = abs(Tau_net[1]) if abs(Tau_net[1])>Tau2max else Tau2max
        Tau3max = abs(Tau_net[2]) if abs(Tau_net[2])>Tau3max else Tau3max

        Tau.append(Tau_net) 

    return Tau, Tau1max, Tau2max, Tau3max



#Left Arm
Tau1l = (Mpelvis+Mshoulder+Melbow+2*Mmotor)*g*Tp[2,3]

Tau2l = (Mshoulder+Melbow+1*Mmotor)*g*Tp_sl[2,3]

Tau3l = (Melbow)*g*Tp_el[2,3]

# G(q) matrix:
G_ql = Matrix([Tau1l, Tau2l, Tau3l]) 

Taul,_,_,_ = GetTau(Jl,G_ql,'l',ql_list)


#Right Arm
Tau1r = (Mpelvis+Mshoulder+Melbow+2*Mmotor)*g*Tp[2,3]

Tau2r = (Mshoulder+Melbow+1*Mmotor)*g*Tp_sr[2,3]

Tau3r = (Melbow)*g*Tp_er[2,3]

# G(q) matrix:
G_qr = Matrix([Tau1r, Tau2r, Tau3r]) 

Taur,_,_,_ = GetTau(Jr,G_qr,'r',qr_list)


#Maximum joint Torque used in Workspace

qsweep = [Matrix([0,0,0])]
i=0
while i<=2*pi:
    qsweep.append( Matrix([i,0,0]) )
    qsweep.append( Matrix([0,i,0]) )
    qsweep.append( Matrix([0,0,i]) )
    i+=0.3
_,tl1,tl2,tl3 = GetTau(Jl,G_ql,'l',qsweep)
_,tr1,tr2,tr3 = GetTau(Jr,G_qr,'r',qsweep)

print("Max left arm joint Torques in Workspace(pelvis, shoulder, elbow): ", tl1,tl2,tl3)
print("Max right arm joint Torques in Workspace(pelvis, shoulder, elbow): ", tr1,tr2,tr3)

#Plot Joint Torque and Joint Position graph


def plotGraph(Tau, q_list):
    global wrist_lpos
    #%matplotlib inline
    f,plts = plt.subplots(6,1,figsize=(15,35))
    plts[0].plot(np.array(Tau)[:,0])
    plts[1].plot(np.array(Tau)[:,1])
    plts[2].plot(np.array(Tau)[:,2])

    plts[3].plot(np.array(q_list)[:,0])
    plts[4].plot(np.array(q_list)[:,1])
    plts[5].plot(np.array(q_list)[:,2])
    i=1
    for ax in plts.flat:
        if(i<=3):
            ax.set(xlabel='time(s)', ylabel='Torque(Nm)')
            ax.set_title("Joint " + str(i) + ": Torque vs Time graph")
        elif(i<=6):
            ax.set(xlabel='time(s)', ylabel='Joint angle(radians)')
            ax.set_title("Joint " + str(i) + ": Angle vs Time graph")
        i=i+1

#Left Arm Torque and Position Plot
plotGraph(Taul,ql_list)        

#Right Arm Torque and Position Plot
plotGraph(Taur,qr_list)


#Left wrist position:
plt.plot(np.array(wrist_lpos)[:,:,-1])



#Right wrist position:
plt.plot(np.array(wrist_rpos)[:,:,-1])

###############################################################################
###############################################################################
#Handling mode:
    
SETLIMITS = False
THRESHOLD_DISTANCE = 0.05

ql = Matrix([ ql_reach_goal_pos ])
qr = Matrix([ qr_reach_goal_pos ])
q_l =  Matrix([ [0.01], [0.01], [0.01]])
q_r =  Matrix([ [0.01], [0.01], [0.01]])

# Move the object above along Z axis till some distance.
Fxl = 0.8
Fyl = 0
Fzl = 0.5

dist = 10

fig = plt.figure()
ax3 = plt.axes(projection='3d')
ax3.set_xlabel('X-axis')
ax3.set_ylabel('Y-axis')
ax3.set_zlabel('Z-axis')
ax3.axis('auto')

distl = 1e2
Kp=0.8
while(distl>THRESHOLD_DISTANCE):

    Jsl = Jl.subs({p: ql[0], sl: ql[1], el: ql[2]})
    Jsr = Jr.subs({p: qr[0], sr: qr[1], er: qr[2]})
    try:
        #Using Jacobian Pseudo-inverse, in case Jacobian is non square matrix.
        Jpl = J_inv(Jsl)
    except:
        ql = (ql + q_l).evalf()
        print("Singularity / rank-loss")
        continue

    try:
        #Using Jacobian Pseudo-inverse, in case Jacobian is non square matrix.
        Jpr = J_inv(Jsr)
    except:
        qr = (qr + q_r).evalf()
        print("Singularity / rank-loss")
        continue
    
    #Step 2: inverse velocity kinematics problem that generates joint velocities 

    Tsp_wl = Tp_wl.subs({p: ql[0], sl: ql[1], el: ql[2]})
    Tsp_wr = Tp_wr.subs({p: qr[0], sr: qr[1], er: qr[2]})

    cx = Tsp_wl[0,3].evalf()
    cy = Tsp_wl[1,3].evalf() + 0.1/2
    cz = Tsp_wl[2,3].evalf() 

    Vxl = Fxl -  Tsp_wl[0,3].evalf()
    Vyl = Fyl - Tsp_wl[1,3].evalf()
    Vzl = Fzl - Tsp_wl[2,3].evalf()


    Vxr = Fxl -  Tsp_wr[0,3].evalf()
    Vyr = Fyl - Tsp_wr[1,3].evalf()
    Vzr = Fzl - Tsp_wr[2,3].evalf()

    distl = sqrt(Vxl**2 + Vyl**2 + Vzl**2)

    #V is the end effector's desiered linear velocity and angular velocity of end effector rotation.
    # V = [vx, vy, vz, roll', pitch', yaw'] 
    Vl = Matrix([ [Vxl], [Vyl], [Vzl], [0], [0], [0]])
    Vr = Matrix([ [Vxr], [Vyr], [Vzr], [0], [0], [0]])
    q_l=Jpl*Kp*Vl
    q_r=Jpr*Kp*Vr

    ql = (ql + q_l).evalf()
    qr = (qr + q_r).evalf()
    ql_list.append(ql)
    qr_list.append(qr)

    #Step 3: Plug in the join velocities generated in Step 2 to forward velocity kinematics developed in Step 1 and plot output of the inverse kinematics problem  

    display.clear_output(wait=True)
    display.display(plt.gcf())
    

   
    # 3D plot
    ax3.cla()
    drawLeftArm(ax3,ql)
    drawRightArm(ax3,qr)
    drawBox(ax3, xc=cx, yc=cy, zc=cz, s=0.1)
    ax3.set_xlabel('X-axis')
    ax3.set_ylabel('Y-axis')
    ax3.set_zlabel('Z-axis')
    ax3.set_title('Robot moving the box')
    #--------------------

    time.sleep(0.01)

plt.show()


## Joint angles and joint torques in handling mode
#Left Arm
Tau1l = (Mpelvis+Mshoulder+Melbow+Mobject+2*Mmotor)*g*Tp[2,3]

Tau2l = (Mshoulder+Melbow+Mobject+Mmotor)*g*Tp_sl[2,3]

Tau3l = (Melbow+Mobject)*g*Tp_el[2,3]

# G(q) matrix:
G_ql = Matrix([Tau1l, Tau2l, Tau3l]) 

Taul,_,_,_ = GetTau(Jl,G_ql,'l',ql_list)


#Right Arm
Tau1r = (Mpelvis+Mshoulder+Melbow+Mobject+2*Mmotor)*g*Tp[2,3]

Tau2r = (Mshoulder+Melbow+Mobject+Mmotor)*g*Tp_sr[2,3]

Tau3r = (Melbow+Mobject)*g*Tp_er[2,3]

# G(q) matrix:
G_qr = Matrix([Tau1r, Tau2r, Tau3r]) 

Taur,_,_,_ = GetTau(Jr,G_qr,'r',qr_list)


#Left Arm Torque and Position Plot
plotGraph(Taul,ql_list)


#Right Arm Torque and Position Plot
plotGraph(Taur,qr_list)