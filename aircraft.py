from numpy import *
from random import gauss

class Aircraft:
	def __init__(self, radius, mass, max_thrust, adjust_rate, gyro_sigma):
		self.radius = radius
		self.mass = mass
		self.max_thrust = max_thrust #per engine
		self.adjust_rate = adjust_rate

		self.beamwidth = 0.015 # (m)
		self.motors = 2
		self.reset()
		self.position = array([0, 0, 0])

		self.gyro = Gyro(gyro_sigma, self)

	def reset(self):
		self.a_moment = array([0, 0, 0])
		self.momentum = array([0, 0, 0])
		self.target_force  = [array([0, 0, 0]), array([0, 0, 0])]
		self.current_force = [array([0, 0, 0]), array([0, 0, 0])]
		self.normal = array([0, 1, 0])
		self.motor_pos = array([-self.radius, 0, 0]), \
						 array([+self.radius, 0, 0])

	def area(self):
		return self.radius*4 * self.beamwidth

	def drag_coefficient(self):
		return 1

	def velocity(self):
		return linalg.norm(self.momentum)

class Sensor:
	def __init__(self, sigma, aircraft):
		self.sigma = sigma
		self.aircraft = aircraft

	def randomize_scalar(self, value):
		if self.sigma == 0:
			return value;
		return guass(value, self.sigma)

	def randomize_vector(self, vector):
		if self.sigma == 0:
			return vector;
		return vector + random.normal(0, self.sigma, len(vector))

	def read(self): ## implement this in inherited obj.
		pass

class Gyro(Sensor):
	def read(self):
		return self.randomize_vector(self.aircraft.normal)
