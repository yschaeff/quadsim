from numpy import *

class Aircraft:
	def __init__(self, radius, mass, max_thrust):
		self.radius = radius
		self.mass = mass
		self.max_thrust = max_thrust #per engine

		self.beamwith = 0.015 # (m)

		self.a_moment = array([0, 0, 0])
		self.momentum = array([0, 0, 0])
		self.f1_target = array([0, 0, 0])
		self.f2_target = array([0, 0, 0])
		self.f1_current = array([0, 0, 0])
		self.f2_current = array([0, 0, 0])
		self.normal = array([0, 1, 0])
		self.position = array([0, 0, 0])

	def moment_of_inertia(self):
		return self.radius**2 * self.mass

	def get_angle(self):
		return arctan2(self.normal[0], self.normal[1])

	def motor_pos(self):
		return array([-self.radius, 0, 0]), \
			   array([+self.radius, 0, 0])

	def area(self):
		return self.radius*4 * self.beamwith

	def drag_coefficient(self):
		return 1
