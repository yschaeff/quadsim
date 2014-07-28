from time import sleep, time
import pygame
from pygame.locals import *
import sys, math
from numpy import *

#world
G = 9.81 			# Earth pull (m*s^-2)
AIR_DENSITY = 1.2922			# kg*m^-3

#aircraft parameters
RADIUS = 0.30 		# distance between center of craft and motor shaft (m)
MASS = 1.2 			# mass of craft (kg)
PULL = array([0, -G*MASS, 0])	# Gravitational Force (kg*m*s^-1)
A_INERTIA = RADIUS**2 * MASS 	# Angular inertia
MOTOR1 = array([ RADIUS, 0, 0]) # Motor position, wrt body (m)
MOTOR2 = array([-RADIUS, 0, 0]) # motor position, wrt body (m)

#display
FPS = 30				# (Hz)
FORCE_DRAW_SCALE = 10 	# (pixels/N)
PIXELS_PER_METER = 100	# (pixels/m)
WIN_WIDTH = 640			# (pixels)
WIN_HEIGHT = 640		# (pixels)

#colors
white = (255,255,255)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
black = (0,0,0)
checkers1 = (255,255,255)
checkers2 = (200,200,200)

def drag_area(radius, height, momentum, normal):
	#~ area = pi*radius**2
	return radius*2 * 0.015 
	#~ .314 m2
	#~ .6 x 0.018
	#find the angle between direction and orientation
	#~ angle = arctan2(normal[0], normal[1]) - arctan2(momentum[0], momentum[1])
	#~ return area

def checkers(winsize, checksize, xoffset, yoffset):
	SIZE = winsize+4*checksize
	#~ SIZE = int(winsize*1.4)
	#~ SIZE = int(winsize*2)
	surf = pygame.Surface((SIZE, SIZE))
	surf.fill(checkers1)
	for y in range(0, SIZE, 2*checksize):
		for x in range(0, SIZE, 2*checksize):
			pygame.draw.rect(surf, checkers2, (x+(xoffset%(2*checksize)), y+(yoffset%(2*checksize)), checksize, checksize))
			pygame.draw.rect(surf, checkers2, (x+checksize+(xoffset%(2*checksize)), y+checksize+(yoffset%(2*checksize)), checksize, checksize))
	#gravitational pull
	pygame.draw.line(surf, red, (SIZE/2, SIZE/2), (SIZE/2, SIZE/2+(MASS*G)*FORCE_DRAW_SCALE), 3)
	return surf

def craft():
	surf = pygame.Surface((2*RADIUS*PIXELS_PER_METER, 2*RADIUS*PIXELS_PER_METER))
	surf.fill(green)
	pygame.draw.rect(surf, blue, (0, RADIUS*PIXELS_PER_METER, 2*RADIUS*PIXELS_PER_METER, 10))
	surf.set_colorkey(green)
	return surf

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

class Simulator:
	@classmethod
	def rotmatx(sim, a):
		return array([[1.0, 0, 0], [0, cos(a), -sin(a)], [0, sin(a), cos(a)]])
	@classmethod
	def rotmaty(sim, a):
		return array([[cos(a), 0, sin(a)], [0, 1.0, 0], [-sin(a), 0, cos(a)]])
	@classmethod
	def rotmatz(sim, a):
		return array([[cos(a), -sin(a), 0], [sin(a), cos(a), 0], [0, 0, 1.0]])
	@classmethod
	def rotmat(sim, angle_rad):
		"""Rotation matrix for combined x,y,z rotation"""
		x = sim.rotmatx(angle_rad[0])
		y = sim.rotmaty(angle_rad[1])
		z = sim.rotmatz(angle_rad[2])
		return dot(x,dot(y,z))
	
	def __init__(self, t_start, steps):
		self.t_start = t_start
		self.steps = steps
		self.a_moment = array([0, 0, 0])
		self.momentum = array([2, 0, 0])
		self.f1_target = array([0, 0, 0])
		self.f2_target = array([0, 0, 0])
		self.f1_current = array([0, 0, 0])
		self.f2_current = array([0, 0, 0])
		#output
		self.normal = array([0, 1, 0])
		self.position = array([0, 0, 0])

	def get_anglegrad(self):
		#~ print self.get_angle()
		return (self.get_angle()*180)/pi
	def get_angle(self):
		return arctan2(self.normal[0], self.normal[1])

	def simulate(self, t_now, input):
		dt = 1.0/self.steps
		self.f1_target = input.thrust1
		self.f2_target = input.thrust2
		while self.t_start < t_now:
			#dt has passed

			## Calculate current thrust
			self.f1_current = (99*self.f1_current + self.f1_target)/100.0
			self.f2_current = (99*self.f2_current + self.f2_target)/100.0

			## Update angular momentum and normal
			torque1 = cross(MOTOR1, self.f1_current)
			torque2 = cross(MOTOR2, self.f2_current)
			torque = torque1 + torque2
			self.a_moment = self.a_moment + (torque/A_INERTIA)
			rot_matrix = Simulator.rotmat(self.a_moment * dt)
			self.normal = dot(rot_matrix, self.normal)


			## linear drag
			drag = 0.5 * AIR_DENSITY * self.momentum**2 * 1 * drag_area(RADIUS, None, None, None)
			## Drag lost sign due square, recover it.
			sign = check_sign(self.momentum, drag)
			drag = drag * sign
			#~ print self.position
			## update position and linear momentum
			ratio = linalg.norm(self.f1_current + self.f2_current) / linalg.norm(self.normal)
			netforce = (self.normal * ratio) + PULL + drag
			self.momentum = self.momentum + (netforce/MASS)*dt
			
			self.position = self.position + self.momentum * dt
			
			self.t_start += dt

class Input():
	def __init__(self):
		self.thrust1 = array([0.0, G*MASS/2+1.1, 0.0])
		self.thrust2 = array([0.0, G*MASS/2+1.1, 0.0])
		#~ self.thrust1 = array([0.0, G*MASS/2, 0.0])
		#~ self.thrust2 = array([0.0, G*MASS/2, 0.0])
		#~ self.thrust1 = array([0.0, 0, 0.0])
		#~ self.thrust2 = array([0.0, 0, 0.0])

t = time()
dt = 1.0/FPS
input  = Input()
sim = Simulator(t_start=t, steps=500)
pygame.init()
screen = pygame.display.set_mode((WIN_WIDTH,WIN_HEIGHT))
craft = craft()

while True:
	#collect user input
	for event in pygame.event.get():
		if event.type == QUIT:
			pygame.quit()
			sys.exit()

	#simulate until time t (future) -> time_sim
	state = sim.simulate(t+dt, input)
	#sleep until time_sim
	now = time()
	if (now < t+dt):
		sleep(t+dt-now)
	
	#draw
	background = checkers(max(WIN_WIDTH, WIN_HEIGHT), PIXELS_PER_METER, sim.position[0]*PIXELS_PER_METER, sim.position[1]*PIXELS_PER_METER)
	s2 = pygame.transform.rotate(background, -sim.get_anglegrad())
	r = s2.get_rect()
	r.center = WIN_WIDTH/2, WIN_HEIGHT/2
	screen.blit(s2, r)
	## Desired thrust
	pygame.draw.line(screen, red, (320+RADIUS*PIXELS_PER_METER, 320), (320+RADIUS*PIXELS_PER_METER, 320-sim.f1_target[1]*FORCE_DRAW_SCALE), 5)
	pygame.draw.line(screen, red, (320-RADIUS*PIXELS_PER_METER, 320), (320-RADIUS*PIXELS_PER_METER, 320-sim.f2_target[1]*FORCE_DRAW_SCALE), 5)
	## actual thrust
	pygame.draw.line(screen, green, (320+RADIUS*PIXELS_PER_METER, 320), (320+RADIUS*PIXELS_PER_METER, 320-sim.f1_current[1]*FORCE_DRAW_SCALE), 3)
	pygame.draw.line(screen, green, (320-RADIUS*PIXELS_PER_METER, 320), (320-RADIUS*PIXELS_PER_METER, 320-sim.f2_current[1]*FORCE_DRAW_SCALE), 3)

	s2 = craft
	r = s2.get_rect()
	r.center = WIN_WIDTH/2, WIN_HEIGHT/2
	screen.blit(s2, r)
	pygame.display.update()

	t += dt
