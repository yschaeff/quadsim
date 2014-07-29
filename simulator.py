from numpy import *

def drag_volume(quad):
	""" Volume of air displaced traveling 1m*s^-1
		This is only accurate while travling in direction  of
		normal. """
	#TODO also depends on momentum and normal
	return quad.area()

def drag_volume_rotation(quad):
	""" Volume of air displaced for 1 rotation along pitch or roll
		assumes +-copter"""
	return 2*pi*quad.radius**2*quad.beamwidth

def drag_volume_rotation_yaw(quad):
	""" Volume of air displaced for 1 rotation along yaw"""
	return 4*pi*quad.radius**2*quad.beamwidth

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

	def simulate(self, t_start, t_now, quad, controller, world):
		""" t_start: Time where last simulation ended
			t_now: Time up onto simulation should run
			returns t_start
			"""
		dt = 1.0/self.steps

		while t_start < t_now:
			thrust = controller.force(dt)

			## Influence of all the motors on forces
			torque = array([0,0,0])
			for i in range(quad.motors):
				t = min(max(thrust[i], 0), quad.max_thrust)
				quad.target_force[i] = array([0, t, 0])
				## Calculate current thrust
				quad.current_force[i] = (99*quad.current_force[i] + quad.target_force[i])/100.0
				torque = torque + cross(quad.motor_pos()[i], quad.current_force[i])

			## Update angular momentum and normal
			drag = 0.5 * world.fluid_density * quad.a_moment**2 * quad.drag_coefficient() * drag_volume_rotation(quad)
			drag = correct_drag(drag, quad.a_moment)
			quad.a_moment = quad.a_moment + (torque/quad.moment_of_inertia()+drag)*dt
			rot_matrix = rotmat(quad.a_moment)
			quad.normal = dot(rot_matrix, quad.normal)

			## linear drag
			drag = 0.5 * world.fluid_density * quad.momentum**2 * quad.drag_coefficient() * drag_volume(quad)
			drag = correct_drag(drag, quad.momentum)
			## update position and linear momentum
			ratio = linalg.norm(quad.current_force[0] + quad.current_force[1]) / linalg.norm(quad.normal)
			earth_pull = array([0, -world.G*quad.mass, 0])	# Gravitational Force (kg*m*s^-1)
			netforce = (quad.normal * ratio) + earth_pull + drag
			quad.momentum = quad.momentum + (netforce/quad.mass)*dt
			quad.position = quad.position + quad.momentum * dt
			
			t_start += dt
		return t_start

class World:
	def __init__(self, G, fluid_density):
		self.G = G
		self.fluid_density = fluid_density
