from numpy import *

def drag_area(quad):
	return quad.area()
	#TODO also depends on momentum and normal

def drag_area_rotation(quad):
	"""factor 2 because 2 arms cause drag in pitch or roll"""
	return 2*pi*quad.radius**2*quad.beamwith


#~ def check_sign(v1, v2):
	#~ c_same = 0; c_diff = 0
	#~ s1 = sign(v1); s2 = sign(v2)
	#~ for i in range(3):
		#~ if s1[i] != s2[i]:
			#~ c_diff += 1
		#~ elif v1[i] != -0 and v2[i] != 0:
			#~ c_same += 1
	#~ if c_same and c_diff:
		#~ return 0 #undecided, lets pretent there is no drag
	#~ elif c_diff:
		#~ return 1
	#~ else:
		#~ return -1

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
	#~ def __init__(self, t_start, steps):
	def __init__(self, steps):
		#~ self.t_start = t_start
		self.steps = steps

	def simulate(self, t_start, t_now, quad, controller, world):
		dt = 1.0/self.steps
		#~ print self, self.t_start, dt

		while t_start < t_now:
			controller.force(dt)
			t1 = min(max(quad.thrust1, 0), quad.max_thrust)
			t2 = min(max(quad.thrust2, 0), quad.max_thrust)
			
			quad.f1_target = array([0, t1, 0])
			quad.f2_target = array([0, t2, 0])

			## Calculate current thrust
			quad.f1_current = (99*quad.f1_current + quad.f1_target)/100.0
			quad.f2_current = (99*quad.f2_current + quad.f2_target)/100.0

			## Update angular momentum and normal
			torque1 = cross(quad.motor_pos()[0], quad.f1_current)
			torque2 = cross(quad.motor_pos()[1], quad.f2_current)
			torque = torque1 + torque2
			drag = 0.5 * world.fluid_density * quad.a_moment**2 * quad.drag_coefficient() * drag_area_rotation(quad)
			## Drag lost sign due square, recover it.
			if linalg.norm(quad.a_moment) != 0:
				drag = (-quad.a_moment)/linalg.norm(quad.a_moment)*linalg.norm(drag)
			quad.a_moment = quad.a_moment + (torque/quad.moment_of_inertia()+drag)*dt
			rot_matrix = rotmat(quad.a_moment)
			quad.normal = dot(rot_matrix, quad.normal)

			## linear drag
			earth_pull = array([0, -world.G*quad.mass, 0])	# Gravitational Force (kg*m*s^-1)
			drag = 0.5 * world.fluid_density * quad.momentum**2 * quad.drag_coefficient() * drag_area(quad)
			## Drag lost sign due square, recover it.
			if linalg.norm(quad.momentum) != 0:
				drag = (-quad.momentum)/linalg.norm(quad.momentum)*linalg.norm(drag)
			## update position and linear momentum
			ratio = linalg.norm(quad.f1_current + quad.f2_current) / linalg.norm(quad.normal)
			netforce = (quad.normal * ratio) + earth_pull + drag
			quad.momentum = quad.momentum + (netforce/quad.mass)*dt
			#~ print quad.momentum, drag
			quad.position = quad.position + quad.momentum * dt
			
			t_start += dt
		return t_start

class World:
	def __init__(self, G, fluid_density):
		self.G = G
		self.fluid_density = fluid_density
