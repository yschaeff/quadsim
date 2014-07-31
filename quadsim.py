#!/usr/bin/python

from time import sleep, time
import pygame
from pygame.locals import *
import sys
from numpy import *
from simulator import QuadSimulator, World
from aircraft import Aircraft
from controller import PIDController as Controller

## World parameters. Current values modelled to be like earth
G = 9.81 				# Earth gravitational pull (m*s^-2)
## Note: air@15C=1.225, water@15C=999
AIR_DENSITY = 1.225		# Density of fluid (kg*m^-3)

SIMULATIONS_PER_SECOND = 5000 # (Hz) Higher values give a more accurate simulation
CONTROLLER_CYCLE_TIME = 0.003 # (s) Seconds it will take the PID controller
							  # to run an entire cycle.

## Aircraft parameters
RADIUS = 0.30 		# distance between center of craft and motor shaft (m)
MASS = 1.2 			# mass of craft (kg)
MAX_THRUST = G*MASS*2 # How much force a motor can give (N)
ADJUST_RATE = 0.01  # Change rate of motors (percentage/simulation_step)
					# 1.0: infinitely fast
ACRO_MODE = 0		# 1 acrobatic mode, 0 stable (will center on stick release)

## Display parameters
FPS = 60				# Frames per second (Hz)
FORCE_DRAW_SCALE = 10 	# (pixels/N)
PIXELS_PER_METER = 100	# (pixels/m)
WIN_WIDTH = 640			# (pixels)
WIN_HEIGHT = 480		# (pixels)
SPEED = 1				# ratio. SPEED<1 => slowdown (dimensionless) 

## Colors, used by gui
white     = (255,255,255)
red       = (255,0,0)
green     = (0,255,0)
blue      = (0,0,255)
black     = (0,0,0)
gray      = (200,200,200)
transparant = (1,1,1)
checkers1 = white
checkers2 = gray

def draw_backdrop(size, ppm):
	color = [checkers1, checkers2]
	surface = pygame.Surface((size[0]+2*ppm, size[1]+2*ppm))
	ysign = 0
	for y in range(0, size[1]+2*ppm, ppm):
		xsign = ysign
		for x in range(0, size[0]+2*ppm, ppm):
			pygame.draw.rect(surface, color[xsign], (x, y, ppm, ppm))
			xsign ^= 1
		ysign ^= 1
	return surface

def draw_world(screen, quad, controller, size, backdrop, ppm, fscale, world):
	global font_obj
	# first draw background
	offset = (quad.position*ppm)%(2*ppm)
	screen.blit(backdrop, (0,0), (ppm*2-offset[0],
		ppm*2-offset[1], size[0], size[1]))

	## Then draw all forces in world frame
	## Gravitational pull
	pygame.draw.line(screen, green, (size[0]/2, size[1]/2),
		(size[0]/2, size[1]/2+(quad.mass*world.G)*fscale), 3)
	pygame.draw.line(screen, red, (size[0]/2, size[1]/2), (size[0]/2-ppm*sin(controller.target_angle), size[1]/2-ppm*cos(controller.target_angle)), 5)

	## text
	label = font_obj.render("% 3.1fm/s"%quad.velocity(), 1, black)
	screen.blit(label, (0, size[1]-25))
	label = font_obj.render("% 3ddeg"%round(quad.get_angle()*180/pi, 1), 1, black)
	screen.blit(label, (150, size[1]-25))
	label = font_obj.render("% 3drad/s"%round(sign(quad.a_moment[2])*linalg.norm(quad.a_moment/quad.mass), 1), 1, black)
	screen.blit(label, (300, size[1]-25))
	
	#, fgcolor=None, bgcolor=None, style=STYLE_DEFAULT, rotation=0, size=0)

def draw_body(screen, quad, controller, size, ppm, fscale):
	mx = size[0]/2
	my = size[1]/2
	## Draw aircraft
	pygame.draw.rect(screen, blue, (mx-quad.radius*ppm, my, 2*quad.radius*ppm, quad.beamwidth*ppm+1))
	## draw forces
	for i in range(quad.motors):
		pygame.draw.line(screen, red, (mx-quad.motor_pos[i][0]*ppm, my),
			(mx-quad.motor_pos[i][0]*ppm,my-quad.target_force[i][1]*fscale), 5)
		pygame.draw.line(screen, green, (mx-quad.motor_pos[i][0]*ppm, my),
			(mx-quad.motor_pos[i][0]*ppm, my-quad.current_force[i][1]*fscale), 3)
		pygame.draw.line(screen, green, (mx, my), (mx, my-ppm), 3)

def get_anglegrad(quad):
	return (quad.get_angle()*180)/pi

## Simulation initialization
t = time()*SPEED
dt = 1.0/FPS
t_sim = t
world = World(G, AIR_DENSITY)
sim = QuadSimulator(SIMULATIONS_PER_SECOND)
quad = Aircraft(RADIUS, MASS, MAX_THRUST, ADJUST_RATE)
controller  = Controller(quad, world, CONTROLLER_CYCLE_TIME, ACRO_MODE)

pygame.init()
font_obj = pygame.font.SysFont("monospace", 25)
screen = pygame.display.set_mode((WIN_WIDTH, WIN_HEIGHT))
backdrop = draw_backdrop((WIN_WIDTH, WIN_HEIGHT), PIXELS_PER_METER)

while True:
	#collect user input
	for event in pygame.event.get():
		if event.type == QUIT:
			pygame.quit()
			sys.exit()

	keys = pygame.key.get_pressed()
	if keys[ord('q')]:
		pygame.quit()
		sys.exit()
	if keys[ord(' ')]:
		quad.reset()
		controller.reset()
	## map key presses to stick positions [-1..0..1]
	if keys[pygame.K_UP]:
		controller.input[0] = 1
	elif keys[pygame.K_DOWN]:
		controller.input[0] = -1
	else:
		controller.input[0] = 0
	if keys[pygame.K_LEFT]:
		controller.input[3] = 1
	elif keys[pygame.K_RIGHT]:
		controller.input[3] = -1
	else:
		controller.input[3] = 0

	#simulate until time t (future) -> time_sim
	t_sim = sim.simulate(t_sim, t+dt*SPEED, quad, controller, world)

	#draw
	draw_world(screen, quad, controller, (WIN_WIDTH, WIN_HEIGHT), backdrop, PIXELS_PER_METER, FORCE_DRAW_SCALE, world)

	surface = pygame.Surface((WIN_WIDTH, WIN_HEIGHT))
	surface.fill(transparant)
	surface.set_colorkey(transparant)
	draw_body(surface, quad, controller, (WIN_WIDTH, WIN_HEIGHT), PIXELS_PER_METER, FORCE_DRAW_SCALE)

	surface = pygame.transform.rotate(surface, get_anglegrad(quad))
	r = surface.get_rect()
	r.center = WIN_WIDTH/2, WIN_HEIGHT/2
	screen.blit(surface, r)
	pygame.display.update()

	#sleep until time_sim
	now = time()*SPEED
	if (now < t+dt*SPEED):
		sleep(t+dt*SPEED-now)
		
	t += dt*SPEED
