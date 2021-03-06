from numpy import *

def vector2angles(vector):
	return array([
		arctan2(vector[2], vector[1]),
		arctan2(vector[2], vector[0]),
		arctan2(vector[1], vector[0])])

def z_angle(vector):
	return arctan2(vector[0], vector[1])

def rotmatx(a):
	"""Rotation Matrix to rotate vector around x-axis"""
	return array([[1.0, 0, 0], [0, cos(a), -sin(a)], [0, sin(a), cos(a)]])

def rotmaty(a):
	"""Rotation Matrix to rotate vector around y-axis"""
	return array([[cos(a), 0, sin(a)], [0, 1.0, 0], [-sin(a), 0, cos(a)]])

def rotmatz(a):
	"""Rotation Matrix to rotate vector around z-axis"""
	return array([[cos(a), -sin(a), 0], [sin(a), cos(a), 0], [0, 0, 1.0]])

def rotmat(angle_rad):
	"""Rotation matrix for combined x,y,z rotation"""
	x = rotmatx(angle_rad[0])
	y = rotmaty(angle_rad[1])
	z = rotmatz(angle_rad[2])
	return dot(x,dot(y,z))

def correct_drag(drag, direction):
	"""Due to the square of the volicity in the drag equation, drag
		loses its direction. This sets points the drag vector opposite
		to the flow direction"""
	if linalg.norm(direction) != 0:
		return (-direction)/linalg.norm(direction)*linalg.norm(drag)
	return drag


class QuadSimulator:
	def __init__(self, steps):
		"""steps: number of simulation steps per second (Hz)"""
		self.steps = steps

	def simulate(self, t_now, t_future, quad, controller, world):
		""" t_now: Time where last simulation ended
			t_future: Time up onto simulation should run
			returns t_now
			"""
		dt = 1.0/self.steps

		while t_now < t_future:
			if controller.lastrun+controller.cycle_time < t_now:
				controller.force(controller.cycle_time)
				controller.lastrun = t_now
			
			## Influence of all the motors on forces
			torque = array([0,0,0])
			for i in range(quad.motors):
				## cap requested thrust
				t = min(max(controller.thrust[i], 0), 1) * quad.max_thrust
				## point it in the upwards direction
				quad.target_force[i] = array([0, t, 0])
				## Calculate current thrust
				quad.current_force[i] = ((1-quad.adjust_rate)*quad.current_force[i] + quad.adjust_rate*quad.target_force[i])
				torque = torque + cross(quad.motor_pos[i], quad.current_force[i])

			## Update angular momentum and normal
			drag = 0.5 * world.fluid_density * (quad.a_moment/quad.mass)**2 * quad.drag_coefficient() * quad.area*2*pi
			drag = correct_drag(drag, quad.a_moment)
			quad.a_moment = quad.a_moment + torque + drag*dt
			rot_matrix = rotmat((quad.a_moment/quad.mass)*dt)
			quad.normal = dot(rot_matrix, quad.normal)

			## linear drag
			drag = 0.5 * world.fluid_density * (quad.momentum/quad.mass)**2 * quad.drag_coefficient() * quad.area
			drag = correct_drag(drag, quad.momentum) #(N)
			## update position and linear momentum
			ratio = linalg.norm(quad.current_force[0] + quad.current_force[1]) / linalg.norm(quad.normal)
			earth_pull = array([0, -world.G*quad.mass, 0])	# Gravitational Force (kg*m*s^-2 = N)
			netforce = (quad.normal * ratio) + earth_pull + drag
			quad.momentum = quad.momentum + netforce*dt # (kg*m*s^-1)
			quad.position = quad.position + (quad.momentum/quad.mass) * dt

			## generate some factual data for our Accelerometer.
			quad.acceleration = dot(rotmat(vector2angles(array([0,1,0]))-vector2angles(quad.normal)), netforce/quad.mass)

			t_now += dt
		return t_now

class World:
	def __init__(self, G, fluid_density):
		self.G = G
		self.fluid_density = fluid_density
