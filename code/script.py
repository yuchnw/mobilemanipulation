#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 16 15:59:00 2018

@author: yuchen
"""

import modern_robotics as mr
import numpy as np
from math import sin, cos
import matplotlib.pyplot as plt
import logging

logging.basicConfig(filename = "runNewtask.log", level = logging.INFO, format = '%(asctime)s  %(processName)s %(name)s  %(message)s')
logging.info("Running newTask version")

l = 0.47/2
r = 0.0475
w = 0.3/2
Hpsu = np.array([[-1/(l + w), 1/(l + w), 1/(l + w), -1/(l + w)],[1, 1, 1, 1],[-1, 1, -1, 1]])

Integral = 0

result = []
param = []
# errors = np.array([[0,0,0,0,0,0]])
error1 = []
error2 = []
error3 = []
error4 = []
error5 = []
error6 = []
error = []

B = np.array([[0, 0, 1, 0, 0.033, 0],[0, -1,0, -0.5076, 0 ,0],[0, -1, 0, -0.3526, 0 ,0],[0, -1, 0, -0.2176, 0 ,0],[0, 0, 1, 0, 0, 0]]).T
M0e = np.array([[1, 0, 0, 0.033],[0, 1, 0, 0],[0, 0, 1, 0.6546],[0, 0, 0, 1]])
#Base of arm frame in chassis base frame
Tb0=np.array([[1, 0, 0, 0.1662],[0, 1, 0, 0],[0, 0, 1, 0.0026],[0, 0, 0, 1]])
#Define the function F that makes Vb=F.u
F=(r/4)*Hpsu
#Make the function F6 such that Vb6=F6.u
F6=np.array([[0]*4,[0]*4])
F6=np.append(F6,F,axis=0)
F6=np.append(F6,np.array([[0]*4]),axis=0)

def TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k):
	"""
	Tse_i: The initial configuration of the end-effector in the reference trajectory == Tse_initial.
	Tsc_i: The cube's initial configuration == Tsc_initial
	Tsc_f: The cube's desired final configuration == Tsc_final
	Tce_g: The end-effector's configuration relative to the cube when it is grasping the cube == Tce_grasp
	Tce_s: The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube == Tce_standoff
	k: The number of trajectory reference configurations per 0.01 seconds
	"""

	Tse_si = np.dot(Tsc_i,Tce_s) # The end-effector's standoff configuration above the initial cube relative to the world frame
	N = 2*k/0.01
	traj1 = mr.CartesianTrajectory(Tse_i,Tse_si,2,N,5) # Move from initial position to the initial standoff position
	for i in range(int(N)):
		param.append([traj1[i][0][0],traj1[i][0][1],traj1[i][0][2],traj1[i][1][0],traj1[i][1][1],traj1[i][1][2],\
			traj1[i][2][0],traj1[i][2][1],traj1[i][2][2],traj1[i][0][3],traj1[i][1][3],traj1[i][2][3],0])
	Tse_gi = np.dot(Tsc_i,Tce_g) # The end-effector's configuration relative to the world when it is grasping the initial cube
	traj2 = mr.CartesianTrajectory(Tse_si,Tse_gi,2,N,5) # Move from initial standoff position to grasp position
	for i in range(int(N)):
		param.append([traj2[i][0][0],traj2[i][0][1],traj2[i][0][2],traj2[i][1][0],traj2[i][1][1],traj2[i][1][2],\
			traj2[i][2][0],traj2[i][2][1],traj2[i][2][2],traj2[i][0][3],traj2[i][1][3],traj2[i][2][3],0])
	temp = param[-1]
	temp[-1] = 1
	for i in range(100):
		param.append(temp)
	traj3 = mr.CartesianTrajectory(Tse_gi,Tse_si,2,N,5) # Move from grasp position back to initial standoff position
	for i in range(int(N)):
		param.append([traj3[i][0][0],traj3[i][0][1],traj3[i][0][2],traj3[i][1][0],traj3[i][1][1],traj3[i][1][2],\
			traj3[i][2][0],traj3[i][2][1],traj3[i][2][2],traj3[i][0][3],traj3[i][1][3],traj3[i][2][3],1])
	Tse_sf = np.dot(Tsc_f,Tce_s) # The end-effector's standoff configuration above the final cube relative to the world frame
	N = 5*k/0.01
	traj4 = mr.CartesianTrajectory(Tse_si,Tse_sf,5,N,5) # Move from initial standoff position to final standoff position
	for i in range(int(N)):
		param.append([traj4[i][0][0],traj4[i][0][1],traj4[i][0][2],traj4[i][1][0],traj4[i][1][1],traj4[i][1][2],\
			traj4[i][2][0],traj4[i][2][1],traj4[i][2][2],traj4[i][0][3],traj4[i][1][3],traj4[i][2][3],1])
	Tce_g2 = np.array([[-0.5, 0, 0.8660254, 0.015],[0, 1, 0, 0],[-0.8660254,0, -0.5,0.],[0, 0, 0, 1]])
	Tse_gf = np.dot(Tsc_f,Tce_g2) # The end-effector's configuration relative to the world when it is grasping the final cube
	N = 2*k/0.01
	traj5 = mr.CartesianTrajectory(Tse_sf,Tse_gf,2,N,5) # Move from final standoff position to final grasp position
	for i in range(int(N)):
		param.append([traj5[i][0][0],traj5[i][0][1],traj5[i][0][2],traj5[i][1][0],traj5[i][1][1],traj5[i][1][2],\
			traj5[i][2][0],traj5[i][2][1],traj5[i][2][2],traj5[i][0][3],traj5[i][1][3],traj5[i][2][3],1])
	temp = param[-1]
	temp[-1] = 0
	for i in range(100):
		param.append(temp)
	traj6 = mr.CartesianTrajectory(Tse_gf,Tse_sf,2,N,5) # Move from final grasp position to final standoff position
	for i in range(int(N)):
		param.append([traj6[i][0][0],traj6[i][0][1],traj6[i][0][2],traj6[i][1][0],traj6[i][1][1],traj6[i][1][2],\
			traj6[i][2][0],traj6[i][2][1],traj6[i][2][2],traj6[i][0][3],traj6[i][1][3],traj6[i][2][3],0])
	np.savetxt("/home/yuchen/Documents/traj.csv", param, delimiter=",")

def NextState(currentConfig,control,dt,maxSpeed,gripper):
	"""
	currentConfig: Chassis phi, Chassis x, Chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, Gripper
	control: Arm joint speed(thetadot), Wheel speed(wdot)
	dt: timestamp
	maxSpeed: Maximum angular speed of the arm joints and the wheels
	"""

	phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,crap = currentConfig[0]
	j1dot,j2dot,j3dot,j4dot,j5dot,w1dot,w2dot,w3dot,w4dot = control[0]
	Ts = np.array([[cos(phi), -sin(phi), 0, x],[sin(phi), cos(phi), 0, y],[0, 0, 0, 0.0963],[0, 0, 0, 1]])
	dtheta = np.array([[w1dot],[w2dot],[w3dot],[w4dot]])*dt
	Vb = np.dot(Hpsu,dtheta)*r/4
	Vb6 = np.array([[0,0,Vb[0],Vb[1],Vb[2],0]]).T
	Tbb = mr.MatrixExp6(mr.VecTose3(Vb6))
	Tb = np.dot(Ts,Tbb)

	phiphi = np.arccos(Tb[0][0])
	xx = Tb[0][3]
	yy = Tb[1][3]

	""" Update new joint angles and wheel speeds """
	jj1 = j1 + j1dot * dt
	jj2 = j2 + j2dot * dt
	jj3 = j3 + j3dot * dt
	jj4 = j4 + j4dot * dt
	jj5 = j5 + j5dot * dt

	ww1 = w1 + w1dot * dt
	ww2 = w2 + w2dot * dt
	ww3 = w3 + w3dot * dt
	ww4 = w4 + w4dot * dt

	result.append([phiphi,xx,yy,jj1,jj2,jj3,jj4,jj5,ww1,ww2,ww3,ww4,gripper])

	return phiphi,xx,yy,jj1,jj2,jj3,jj4,jj5,ww1,ww2,ww3,ww4,gripper

def Convert(phi,x,y,j1,j2,j3,j4,j5):
	"""
	Given the current configurations of robot chassis and joints,
	return the Jacobian matrices.
	"""

	thetalist = np.array([j1,j2,j3,j4,j5])
	Tsb = np.array([[cos(phi), -sin(phi), 0, x],[sin(phi), cos(phi), 0, y],[0, 0, 1, 0.0963],[0, 0, 0, 1]])
	Tb0 = np.array([[1, 0, 0, 0.1662],[0, 1, 0, 0],[0, 0, 1, 0.0026],[0, 0, 0, 1]])
	Ja = mr.JacobianBody(B,thetalist)
	T0e = mr.FKinBody(M0e,B,thetalist)
	X = np.dot(np.dot(Tsb,Tb0),T0e)
	Jb = np.dot(mr.Adjoint(np.dot(np.linalg.inv(T0e),Tb0)), F6)

	return X,Ja,Jb

def FeedbackControl(Xd,Xd_next,X,Kp,Ki,dt,Ja,Jb):
	"""
	Xd: Current end-effector configuration
	Xd_next: End-effector configuration at next timestamp
	X: Current actual end-effector configuration(Tse)
	Kp, Ki: PI controller gains
	Ja, Jb: Jacobian matrices of robot arm and robot chassis
	"""
	global Integral
	"""
    V(t) = [Ad_X^(-1)*Xd]*Vd(t) + Kp*Xerr(t) + Ki*Integral[Xerr(t)]dt
	"""
	Vd = mr.se3ToVec(mr.MatrixLog6(np.dot(np.linalg.inv(Xd),Xd_next))/dt)
	term_1 = np.dot(mr.Adjoint(np.dot(np.linalg.inv(X),Xd)),Vd)
	Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(np.linalg.inv(X),Xd)))
	term_2 = np.dot(Kp,Xerr)
	Integral = Integral + Xerr*dt
	term_3 = np.dot(Ki,Integral)
	Vt = term_1 + term_2 + term_3
	Je = np.append(Jb,Ja,axis=1)
	v = np.dot(np.linalg.pinv(Je,1e-4), Vt)
	return v,Xerr

def createT(a):
	""" Helper function to parse an element of array list to an individual array """
	output = np.array([[a[0],a[1],a[2],a[9]],[a[3],a[4],a[5],a[10]],[a[6],a[7],a[8],a[11]],[0,0,0,1]])
	return output

def main():
	logging.info("Generating animation csv file...")
	Tse_i = np.array([[0, 0, 1, 0],[0, 1, 0, 0],[-1, 0, 0, 0.5],[0, 0, 0, 1]])

	""" Default positions """
	Tsc_i = np.array([[1, 0, 0, 1],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
	Tsc_f = np.array([[0, 1, 0, 0],[-1,0, 0,-1],[0, 0, 1, 0],[0, 0, 0, 1]])

	""" New Task Positions """
	# Tsc_i = np.array([[0, -1, 0, 1],[1, 0, 0, 1],[0, 0, 1, 0],[0, 0, 0, 1]])
	# Tsc_f = np.array([[1, 0, 0, 2],[0,1, 0,0],[0, 0, 1, 0],[0, 0, 0, 1]])

	Tce_g = np.array([[-0.5, 0, 0.8660254, 0.015],[0, 1, 0, 0],[-0.8660254,0, -0.5,0.02],[0, 0, 0, 1]])
	Tce_s = np.array([[0, 0, 1, -0.3],[0, 1, 0, 0],[-1, 0, 0, 0.5],[0, 0, 0, 1]])
	k = 1
	TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k)

	""" Initial parameters """
	dt = 0.01
	phi=-np.pi/5
	x=0.1
	y=0.2
	w1=0
	w2=0
	w3=0
	w4=0
	j1=0
	j2=0
	j3=0.2
	j4=-1.6
	j5=0
	Kp = 3 * np.identity(6)
	Ki = 0.1 * np.identity(6)
	tconfig = np.array([[phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,0]])
	result.append(tconfig[0])
	maxSpeed = np.array([[1000,1000,1000,1000,1000,1000,1000,1000,1000]])

	""" Loop through every line of trajectory to set up the chassis configurations and control parameters """
	
	for i in range(len(param)-1):
		X,Jarm,Jbase = Convert(phi,x,y,j1,j2,j3,j4,j5)
		Xd = createT(param[i])
		Xd_next = createT(param[i+1])
		v,Xerr = FeedbackControl(Xd,Xd_next,X,Kp,Ki,dt,Jarm,Jbase)
		tcontrol = np.array([np.append(v[-5:],v[:4])])
		g = param[i][-1]
		phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,g = NextState(np.array([result[-1]]),tcontrol,dt,maxSpeed,g)
		error1.append(Xerr[0])
		error2.append(Xerr[1])
		error3.append(Xerr[2])
		error4.append(Xerr[3])
		error5.append(Xerr[4])
		error6.append(Xerr[5])
		error.append([Xerr[0],Xerr[1],Xerr[2],Xerr[3],Xerr[4],Xerr[5]])
	np.savetxt("/home/yuchen/Documents/Fall2018/ME449/Project/Chassis.csv", result, delimiter=",")
	np.savetxt("/home/yuchen/Documents/Fall2018/ME449/Project/Error.csv", error, delimiter=",")

	""" Generate Error Plots """
	logging.info("Writing error plot data...")
	t=np.arange(0,16.99,0.01)
	plt.plot(t,error1,label='error1')
	plt.plot(t,error2,label='error2')
	plt.plot(t,error3,label='error3')
	plt.plot(t,error4,label='error4')
	plt.plot(t,error5,label='error5')
	plt.plot(t,error6,label='error6')
	plt.legend()
	plt.title("Error Plot")
	plt.show()

	logging.info("Done")

if __name__ == '__main__':
	main()