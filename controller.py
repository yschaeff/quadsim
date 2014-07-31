from numpy import *

class PIDController():
	def __init__(self, quad, world, cycle_time):
		self.quad = quad
		self.world = world
		self.cycle_time = cycle_time
		self.reset()

	def reset(self):
		self.error = 0
		self.integral = 0
		self.thrustscalar = 1
		self.target_angle = 0
		self.lastrun = 0
		self.thrust = [0, 0]

	def pid(self, dt): #return tuple motor force
		#~ return (self.target_angle - self.quad.get_angle())/10000
		kp = 0.040
		ki = 0.2 *0
		kd = kp *0.2

		error = self.target_angle - self.quad.get_angle()
		self.integral += error*dt
		derivative = (error - self.error)/dt
		self.error = error
		return kp*error + ki*self.integral + kd*derivative

	def force(self, dt):
		angle = self.pid(dt) #-pi - pi
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
