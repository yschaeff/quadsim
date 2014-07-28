from time import sleep, time
import pygame
from pygame.locals import *
import sys, math
from numpy import *


def drag_area(radius, height, momentum, normal):
	#~ area = pi*radius**2
	return radius*2 * 0.015 
	#~ .314 m2
	#~ .6 x 0.018
	#find the angle between direction and orientation
	#~ angle = arctan2(normal[0], normal[1]) - arctan2(momentum[0], momentum[1])
	#~ return area


def check_sign(v1, v2):
	c_same = 0; c_diff = 0
	s1 = sign(v1); s2 = sign(v2)
	for i in range(3):
		if s1[i] != s2[i]:
			c_diff += 1
		elif v1[i] != -0 and v2[i] != 0:
			c_same += 1
	if c_same and c_diff:
		return 0 #undecided, lets pretent there is no drag
	elif c_diff:
		return 1
	else:
		return -1

def rotmatx(a):
	return array([[1.0, 0, 0], [0, cos(a), -sin(a)], [0, sin(a), cos(a)]])
def rotmaty(a):
	return array([[cos(a), 0, sin(a)], [0, 1.0, 0], [-sin(a), 0, cos(a)]])
def rotmatz(a):
	return array([[cos(a), -sin(a), 0], [sin(a), cos(a), 0], [0, 0, 1.0]])
def rotmat(angle_rad):
	"""Rotation matrix for combined x,y,z rotation"""
	x = rotmatx(angle_rad[0])
	y = rotmaty(angle_rad[1])
	z = rotmatz(angle_rad[2])
	return dot(x,dot(y,z))


class QuadSimulator:
	def __init__(self, t_start, steps, air_density, G):
		self.t_start = t_start
		self.steps = steps
		self.air_density = air_density
		self.G = G

	def get_anglegrad(self, quad):
		return (self.get_angle(quad)*180)/pi

	def get_angle(self, quad):
		return arctan2(quad.normal[0], quad.normal[1])

	def simulate(self, t_now, quad, input):
		dt = 1.0/self.steps
		quad.f1_target = input.thrust1
		quad.f2_target = input.thrust2
		while self.t_start < t_now:
			#dt has passed

			## Calculate current thrust
			quad.f1_current = (99*quad.f1_current + quad.f1_target)/100.0
			quad.f2_current = (99*quad.f2_current + quad.f2_target)/100.0

			## Update angular momentum and normal
			torque1 = cross(quad.motor_pos()[0], quad.f1_current)
			torque2 = cross(quad.motor_pos()[1], quad.f2_current)
			torque = torque1 + torque2
			quad.a_moment = quad.a_moment + (torque/quad.angular_inertia())
			rot_matrix = rotmat(quad.a_moment * dt)
			quad.normal = dot(rot_matrix, quad.normal)


			## linear drag
			earth_pull = array([0, -self.G*quad.mass, 0])	# Gravitational Force (kg*m*s^-1)
			drag = 0.5 * self.air_density * quad.momentum**2 * 1 * drag_area(quad.radius, None, None, None)
			## Drag lost sign due square, recover it.
			sign = check_sign(quad.momentum, drag)
			drag = drag * sign
			## update position and linear momentum
			ratio = linalg.norm(quad.f1_current + quad.f2_current) / linalg.norm(quad.normal)
			netforce = (quad.normal * ratio) + earth_pull + drag
			quad.momentum = quad.momentum + (netforce/quad.mass)*dt
			
			quad.position = quad.position + quad.momentum * dt
			
			self.t_start += dt
