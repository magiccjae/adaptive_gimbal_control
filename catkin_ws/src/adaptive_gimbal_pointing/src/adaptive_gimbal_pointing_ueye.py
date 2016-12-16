#!/usr/bin/env python
# Module to compare the two numbers and identify and error between sending via float and ASCII
import serial as sr
import numpy as np
import math as m
import rospy
from geometry_msgs.msg import Vector3
import time
import struct
import sys

f = 2900
pixel_to_meter = 0.0000040678
eps_x = 0
eps_y = 0
theta_hat = 1
Ts = 0.01
gamma_theta = 1000
alpha = 10

def callback(data):
	global eps_x
	eps_x = data.x
	global eps_y
	eps_y = data.y
	if eps_x==-800:
		eps_x=0
	if eps_y==-600:
		eps_y=0

rospy.init_node('gimbal_pointing', anonymous=True)
rospy.Subscriber("pixel_location", Vector3, callback)


def sat(u):
	out = 0
	if u > 2:
		out = 2
	elif u < -2:
		out = -2
	else:
		out = u
	return out


R_g_c = np.matrix([[0,1,0],[0,0,1],[1,0,0]])

def Rot_b_to_g(az,el):
	Rot_b_to_g1 = np.matrix([[m.cos(az),m.sin(az),0],[-m.sin(az),m.cos(az),0],[0,0,1]])
	Rot_g1_to_g = np.matrix([[m.cos(el),0,-m.sin(el)],[0,1,0],[m.sin(el), 0, m.cos(el)]])
	return Rot_g1_to_g*Rot_b_to_g1

def calculate_commanded_speed(az, el, eps_x, eps_y):
	u_uav = 0
	v_uav = 0
	w_uav = 0
	v = R_g_c*Rot_b_to_g(az,el)*np.matrix([[u_uav],[v_uav],[w_uav]])
	vx = v.item(0)
	vy = v.item(1)
	vz = v.item(2)
	global f
	global pixel_to_meter
	lambda1 = f*pixel_to_meter
	u = eps_x*pixel_to_meter
	w = eps_y*pixel_to_meter
	s = np.matrix([[u],[w]])
	s_ref = np.matrix([[0],[0]])
	e = s-s_ref
	phi = np.matrix([[-lambda1*vx+u*vz],[-lambda1*vy+w*vz]])
	global theta_hat
	global gamma_theta
	theta_hat = theta_hat+Ts*(gamma_theta*e.transpose()*phi)
	if theta_hat < 0.0005:
		theta_hat = 0.0005
	elif theta_hat > 1:
		theta_hat = 1
	z = 1/theta_hat
	denominator = m.pow(lambda1,2)+m.pow(u,2)+m.pow(w,2)
	L_w_inv = np.matrix([[0, 0],[0, 0]])
	if (denominator+denominator*w*m.tan(el)/lambda1)!=0:
		L_w_inv = 1/(denominator+denominator*w*m.tan(el)/lambda1)*\
		np.matrix([[-u*w/lambda1+u*m.tan(el),(m.pow(lambda1,2)+m.pow(u,2))/lambda1+w*m.tan(el)],\
		[-(m.pow(lambda1,2)+m.pow(w,2))/lambda1,u*w/lambda1]])
	global alpha
	u_unsat = L_w_inv*(-theta_hat.item(0)*phi-alpha*s)
	w_el = u_unsat.item(0)
	w_cel = u_unsat.item(1)
	w_el = sat(w_el)
	w_cel = sat(w_cel)
	w_az_unsat = w_cel/m.cos(el)
	w_az = sat(w_az_unsat)
	return (w_az, w_el)

print "gimbal_pointing_ueye started"
arduino = sr.Serial('/dev/ttyUSB0')		# dummy serial port to get rid of garbage values left over
with arduino:
	arduino.setDTR(False)
	time.sleep(1)
	arduino.flushInput()
	arduino.setDTR(True)

ser = sr.Serial('/dev/ttyUSB0', 57600)

while True:
	try:
		pitch_IMU = float(ser.readline())
		yaw_IMU = float(ser.readline())		
		pitch_encoder = float(ser.readline())
		yaw_encoder = float(ser.readline())
		print "IMU el, az", pitch_IMU, yaw_IMU		
		print "encoder el, az", pitch_encoder, yaw_encoder
		print "pixel_location", eps_x, eps_y
		(w_az,w_el) = calculate_commanded_speed(yaw_encoder*m.pi/180,pitch_encoder*m.pi/180,eps_x,eps_y)
		dividing_factor = 5 	# motors move too fast if angular velocity command is not divided by some constant
		desired_el_dot = round(w_el*180/m.pi, 2)/dividing_factor
		desired_az_dot = round(w_az*180/m.pi, 2)/dividing_factor
		print "desired el_dot, az_dot", desired_el_dot, desired_az_dot
		bin = struct.pack('f', desired_az_dot)
		for b in bin:		# float 4 bytes
			ser.write(b)
		bin2 = struct.pack('f', desired_el_dot)
		for b in bin2:		# float 4 bytes
			ser.write(b)
		temp_az = float(ser.readline())
		temp_el = float(ser.readline())
		print "commanded el_dot, az_dot", temp_el, temp_az
		time.sleep(0.1)	
	except KeyboardInterrupt:
		print "exiting...."
		ser.flushInput()
		ser.flushOutput()
		ser.close()
		print "serial port closed"
