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
		self.thrustscalar = 1 ## 0..1
		self.target_angle = 0 ## rad
		self.lastrun = 0
		self.thrust = [0, 0] ##0..1
		self.input = [0,0,0,0] #4 channels, thrust, yaw, pitch, roll

	def handle_input(self):
		## Throttle
		if self.input[0] == 0:
			## special case, hover at mid-stick
			self.thrustscalar = (self.world.G*self.quad.mass)/float(2*self.quad.max_thrust)
		else:
			self.thrustscalar = (self.input[0]+1)/2.0
		
		if self.acro_mode:
			## Acro mode, angle is increased by stick position
			self.target_angle += self.input[3] * self.cycle_time * 10
			## Wrap angle around, sort of modules operation.
			while self.target_angle > pi:
				self.target_angle -= 2*pi
			while self.target_angle < -pi:
				self.target_angle += 2*pi
		else:
			## stable mode, angle proportional to stick position
			self.target_angle = self.input[3] * pi/4

	def pid(self, measured_angle, dt): #return tuple motor force
		#~ return (self.target_angle - self.quad.get_angle())/10000
		kp = 0.020 * 1
		ki = 0.040 * 1
		kd = 0.003 * 1

		error = self.target_angle - z_angle(measured_angle)
		self.integral += error*dt
		derivative = (error - self.error)/dt
		self.error = error
		return kp*error + ki*self.integral + kd*derivative

	def force(self, dt):
		self.handle_input()
		measured_angle = self.quad.gyro.read()
		angle = self.pid(measured_angle, dt) #-pi - pi
		if angle > pi or angle < -pi:
			angle = -angle

		## divide thrust on motors depending on angle
		f1 = (pi+angle)/(2*pi)
		f2 = (pi-angle)/(2*pi)

		# currect thrust with angle to maintain altitude
		f1 /= cos(z_angle(measured_angle))
		f2 /= cos(z_angle(measured_angle))

		## Normalize, This will make sure the aircraft is steerable
		## even at full thrust. Prevent div by 0
		fm = max(f1, f2)
		if f1: f1 /= fm
		if f2: f2 /= fm

		## Quantity of thrust
		f2 *=  self.thrustscalar
		f1 *=  self.thrustscalar

		## Note: Scale down either f1 or f2 with a constant to simulate
		## an off-center center of gravity. (To for example test the
		## I-term in the PID controller)
		self.thrust = [f1, f2]
