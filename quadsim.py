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

## Aircraft parameters
RADIUS = 0.30 		# distance between center of craft and motor shaft (m)
MASS = 1.2 			# mass of craft (kg)
MAX_THRUST = G*MASS*2

## Display parameters
FPS = 60				# Frames per second (Hz)
FORCE_DRAW_SCALE = 10 	# (pixels/N)
PIXELS_PER_METER = 100	# (pixels/m)
WIN_WIDTH = 640			# (pixels)
WIN_HEIGHT = 640		# (pixels)
SPEED = 1				# ratio. SPEED<1 => slowdown (dimensionless) 

## Colors, used by gui
white     = (255,255,255)
red       = (255,0,0)
green     = (0,255,0)
blue      = (0,0,255)
black     = (0,0,0)
gray      = (200,200,200)
checkers1 = white
checkers2 = gray

def draw_backdrop(winsize, checksize, xoffset, yoffset):
	""""""
	SIZE = winsize+4*checksize
	surf = pygame.Surface((SIZE, SIZE))
	surf.fill(checkers1)
	for y in range(0, SIZE, 2*checksize):
		for x in range(0, SIZE, 2*checksize):
			pygame.draw.rect(surf, checkers2, (x+(xoffset%(2*checksize)), y+(yoffset%(2*checksize)), checksize, checksize))
			pygame.draw.rect(surf, checkers2, (x+checksize+(xoffset%(2*checksize)), y+checksize+(yoffset%(2*checksize)), checksize, checksize))
	#TODO: don't draw that here. gravitational pull
	pygame.draw.line(surf, red, (SIZE/2, SIZE/2), (SIZE/2, SIZE/2+(MASS*G)*FORCE_DRAW_SCALE), 3)
	return surf

def craft(quad):
	surf = pygame.Surface((2*quad.radius*PIXELS_PER_METER, 2*quad.radius*PIXELS_PER_METER))
	surf.fill(green)
	pygame.draw.rect(surf, blue, (0, quad.radius*PIXELS_PER_METER, 2*quad.radius*PIXELS_PER_METER, 10))
	surf.set_colorkey(green)
	return surf

def get_anglegrad(quad):
	return (quad.get_angle()*180)/pi

## Simulation initialization
t = time()*SPEED
dt = 1.0/FPS
t_sim = t
world = World(G, AIR_DENSITY)
sim = QuadSimulator(5000)
quad = Aircraft(RADIUS, MASS, MAX_THRUST)
controller  = Controller(quad, world)

pygame.init()
screen = pygame.display.set_mode((WIN_WIDTH, WIN_HEIGHT))

while True:
	#collect user input
	for event in pygame.event.get():
		if event.type == QUIT:
			pygame.quit()
			sys.exit()
	keys = pygame.key.get_pressed()
	if keys[pygame.K_UP]:
		controller.thrustscalar = 2
	elif keys[pygame.K_DOWN]:
		controller.thrustscalar = 0
	else:
		controller.thrustscalar = 1
	if keys[pygame.K_LEFT]:
		controller.target_angle = pi/4
	elif keys[pygame.K_RIGHT]:
		controller.target_angle = -pi/4
	else:
		controller.target_angle = 0

	#simulate until time t (future) -> time_sim
	t_sim = sim.simulate(t_sim, t+dt*SPEED, quad, controller, world)

	#draw
	background = draw_backdrop(max(WIN_WIDTH, WIN_HEIGHT), PIXELS_PER_METER, quad.position[0]*PIXELS_PER_METER, quad.position[1]*PIXELS_PER_METER)
	#~ s2 = pygame.transform.rotate(background, -get_anglegrad(quad))
	s2 = background
	r = s2.get_rect()
	r.center = WIN_WIDTH/2, WIN_HEIGHT/2
	screen.blit(s2, r)

	## Desired thrust
	pygame.draw.line(screen, red, (320+quad.radius*PIXELS_PER_METER, 320), (320+quad.radius*PIXELS_PER_METER, 320-quad.f1_target[1]*FORCE_DRAW_SCALE), 5)
	pygame.draw.line(screen, red, (320-quad.radius*PIXELS_PER_METER, 320), (320-quad.radius*PIXELS_PER_METER, 320-quad.f2_target[1]*FORCE_DRAW_SCALE), 5)
	## actual thrust
	pygame.draw.line(screen, green, (320+quad.radius*PIXELS_PER_METER, 320), (320+quad.radius*PIXELS_PER_METER, 320-quad.f1_current[1]*FORCE_DRAW_SCALE), 3)
	pygame.draw.line(screen, green, (320-quad.radius*PIXELS_PER_METER, 320), (320-quad.radius*PIXELS_PER_METER, 320-quad.f2_current[1]*FORCE_DRAW_SCALE), 3)

	s2 = craft(quad)
	s2 = pygame.transform.rotate(s2, get_anglegrad(quad))
	r = s2.get_rect()
	r.center = WIN_WIDTH/2, WIN_HEIGHT/2
	screen.blit(s2, r)
	pygame.display.update()

	#sleep until time_sim
	now = time()*SPEED
	if (now < t+dt*SPEED):
		sleep(t+dt*SPEED-now)
		
	t += dt*SPEED
