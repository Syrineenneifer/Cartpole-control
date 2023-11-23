import scipy.linalg
import numpy as np
from math import sin, cos, pi

def lqr( A, B, Q, R ):
	x = scipy.linalg.solve_continuous_are( A, B, Q, R )
	k = np.linalg.inv(R) * np.dot( B.T, x )
	return k

l = 0.5
m = 0.5
M = 0.5
b = 0.1
g = 9.82

def getA(theta, thetadot, xdot, u) :
	A = np.array([[ 0, 1, 0, 0 ],
	          	[ 0, (-4*b)/(4*(M+m)-3*m*(cos(theta))**2), (4*l*m*sin(theta)*thetadot)/(4*(M+m)-3*m*(cos(theta))**2), -(m*(3*(2*l*m*(thetadot**2)*cos(theta)+8*g*M+5*g*m)*((sin(theta))**2)-24*(b*xdot-u)*cos(theta)*sin(theta)-2*l*(M+m)*(thetadot**2)*cos(theta)-12*g*M-3*g*m))/((m*(3*((cos(theta))**2)-4)-4*M)**2)],
	          	[ 0, (-6*b*cos(theta))/(l*(4*(M+m)-3*m*(cos(theta))**2)), (-6*m*cos(theta)*sin(theta)*thetadot)/(4*(M+m)-3*m*(cos(theta))**2), (18*m*cos(theta)*sin(theta)*(2*(g*(M+m)*sin(theta)+(u-b*xdot)*cos(theta))+l*m*(thetadot**2)*cos(theta)*sin(theta))-3*(4*(M+m)-3*m*((cos(theta))**2))*(-l*m*(thetadot**2)*((sin(theta))**2)+2*(g*(M+m)*cos(theta)-(u-b*xdot)*sin(theta))+(l*m*(thetadot**2)*((cos(theta))**2))))/(l*(4*(M+m)-3*m*((cos(theta))**2))**2)],
              	[ 0, 0, 1, 0 ]] )

	return A

def getB(theta) :
	B = np.array( [[0, 4/(4*(M+m)-3*m*(cos(theta))**2), (-6*cos(theta))/(4*(M+m)-3*m*(cos(theta))**2), 1 ]] )
	B.shape = (4,1)

	return B

# we need to tune Q and R

def getQ() :
	Q =  np.array([[ 0.1, 0, 0, 0 ],
       	       	[ 0, 1, 0, 0 ],
	          	 [ 0, 0, 1, 0 ],
              	 [ 0, 0, 0, 0.5 ]] )
	return Q    

def getR() :
	R = np.array([[0.55]]) 
	return R


