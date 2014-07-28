#!/bin/python

from time import sleep, time
import pygame
from pygame.locals import *
import sys, math
from numpy import *
from simulator import QuadSimulator
from aircraft import Aircraft

#world
G = 9.81 			# Earth pull (m*s^-2)
AIR_DENSITY = 1.2922			# kg*m^-3

#aircraft parameters
RADIUS = 0.30 		# distance between center of craft and motor shaft (m)
MASS = 1.2 			# mass of craft (kg)

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

def checkers(winsize, checksize, xoffset, yoffset):
	SIZE = winsize+4*checksize
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
sim = QuadSimulator(t, 500, AIR_DENSITY, G)
quad = Aircraft(RADIUS, MASS)
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
	state = sim.simulate(t+dt, quad, input)
	#sleep until time_sim
	now = time()
	if (now < t+dt):
		sleep(t+dt-now)
	
	#draw
	background = checkers(max(WIN_WIDTH, WIN_HEIGHT), PIXELS_PER_METER, quad.position[0]*PIXELS_PER_METER, quad.position[1]*PIXELS_PER_METER)
	s2 = pygame.transform.rotate(background, -sim.get_anglegrad(quad))
	r = s2.get_rect()
	r.center = WIN_WIDTH/2, WIN_HEIGHT/2
	screen.blit(s2, r)
	## Desired thrust
	pygame.draw.line(screen, red, (320+quad.radius*PIXELS_PER_METER, 320), (320+quad.radius*PIXELS_PER_METER, 320-quad.f1_target[1]*FORCE_DRAW_SCALE), 5)
	pygame.draw.line(screen, red, (320-quad.radius*PIXELS_PER_METER, 320), (320-quad.radius*PIXELS_PER_METER, 320-quad.f2_target[1]*FORCE_DRAW_SCALE), 5)
	## actual thrust
	pygame.draw.line(screen, green, (320+quad.radius*PIXELS_PER_METER, 320), (320+quad.radius*PIXELS_PER_METER, 320-quad.f1_current[1]*FORCE_DRAW_SCALE), 3)
	pygame.draw.line(screen, green, (320-quad.radius*PIXELS_PER_METER, 320), (320-quad.radius*PIXELS_PER_METER, 320-quad.f2_current[1]*FORCE_DRAW_SCALE), 3)

	s2 = craft
	r = s2.get_rect()
	r.center = WIN_WIDTH/2, WIN_HEIGHT/2
	screen.blit(s2, r)
	pygame.display.update()

	t += dt
