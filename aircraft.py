from numpy import *

class Aircraft:
	def __init__(self, radius, mass):
		self.radius = radius
		self.mass = mass

		self.a_moment = array([0, 0, 0])
		self.momentum = array([2, 0, 0])
		self.f1_target = array([0, 0, 0])
		self.f2_target = array([0, 0, 0])
		self.f1_current = array([0, 0, 0])
		self.f2_current = array([0, 0, 0])
		self.normal = array([0, 1, 0])
		self.position = array([0, 0, 0])

	def angular_inertia(self):
		return self.radius**2 * self.mass

	def motor_pos(self):
		return array([self.radius, 0, 0]), array([-self.radius, 0, 0])
