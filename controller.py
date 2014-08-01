from numpy import *
from simulator import z_angle

class PIDController():
	def __init__(self, quad, world, cycle_time, acro_mode):
		self.quad = quad
		self.world = world
		self.cycle_time = cycle_time
		self.acro_mode = acro_mode
		self.reset()

	def reset(self):
		self.error = 0
		self.integral = 0
		self.thrustscalar = 1
		self.target_angle = 0
		self.lastrun = 0
		self.thrust = [0, 0]
		self.input = [0,0,0,0] #4 channels, thrust, yaw, pitch, roll

	def handle_input(self):
		self.thrustscalar = 1*(self.input[0]+1)
		if self.acro_mode:
			self.target_angle += self.input[3] * self.cycle_time * 10
			while self.target_angle > pi:
				self.target_angle -= 2*pi
			while self.target_angle < -pi:
				self.target_angle += 2*pi
		else:
			self.target_angle = self.input[3] * pi/4

	def pid(self, dt): #return tuple motor force
		#~ return (self.target_angle - self.quad.get_angle())/10000
		kp = 0.040
		ki = 0.2 *0
		kd = kp *0.2

		error = self.target_angle - z_angle(self.quad.gyro.read())
		self.integral += error*dt
		derivative = (error - self.error)/dt
		self.error = error
		return kp*error + ki*self.integral + kd*derivative

	def force(self, dt):
		self.handle_input()
		angle = self.pid(dt) #-pi - pi
		if angle > pi or angle < -pi:
			angle = -angle
		f1 = (pi+angle)/(2*pi)
		f2 = (pi-angle)/(2*pi)
		# currect thrust with angle to maintain altitude
		# this can lead to instability, unable to recover during a spin
		# to fix this we need to normalize
		#~ f1 /= cos(self.quad.get_angle())
		#~ f2 /= cos(self.quad.get_angle())
		thrust1 = f1*self.world.G*self.quad.mass*self.thrustscalar #output of motor 1, wrtbody (N)
		thrust2 = f2*self.world.G*self.quad.mass*self.thrustscalar
		self.thrust = [thrust1, thrust2]
