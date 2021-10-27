# physics simulation for quadruped
import sys
import pygame
import pymunk
import pymunk.pygame_util

import random
random.seed(1)
from math import radians,degrees,cos,sin

# NOTE: to make display with pygame simpler unit here used is scaled
# distance: cm, g: cm/s^2, force: N/100

from common import *
from Quadruped import *
from Controller import *

class Sim:
    def __init__(self):
        self.width = 800
        self.height = 600
        self.dt = 0.01

        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Quadruped Simulation")
        self.clock = pygame.time.Clock()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)

        self.space = pymunk.Space()
        self.space.damping = 0.1
        self.g = 9.8*100
        self.space.gravity = (0.0, -self.g)
        self.setCollisionHandler()
        #self.space.gravity = (0.0, 0)
        self.addGround()

        # DEBUG
        self.ball = self.addBall()
        self.quadruped = Quadruped(self.space, (100,170))
        self.quadruped.controller = Controller(self.quadruped)
        self.quadruped.controller.screen = self.screen

    def setCollisionHandler(self):
        def collide(x,y,z):
            return True
        def no_collide(x,y,z):
            return False
        self.space.add_collision_handler(collision_types['ground'], collision_types['body']).begin = collide
        self.space.add_collision_handler(collision_types['body'], collision_types['body']).begin = no_collide
        self.space.add_collision_handler(collision_types['body'], collision_types['obstacle']).begin = collide
        return

    def addBall(self):
        """Add a ball to the given space at a random position"""
        mass = 3
        radius = 25
        inertia = pymunk.moment_for_circle(mass, 0, radius, (0,0))
        body = pymunk.Body(mass, inertia)
        body.position = 150, self.ground_height + 200
        shape = pymunk.Circle(body, radius, (0,0))
        shape.friction = 1
        shape.collision_type = collision_types['obstacle']
        self.space.add(body, shape)
        return body

    def addGround(self):
        radius = 5
        # (x,y) origin at upper left, x+ right y+down
        self.ground_height = ground_height = 100
        segment = pymunk.Segment(self.space.static_body, (0, ground_height), (self.width,ground_height), radius)
        segment.friction = 0.9
        #segment.collision_type = None
        self.space.add(segment)

    def updateControl(self):
        quadruped = self.quadruped
        controller = quadruped.controller
        u = controller.stand()
        quadruped.applyGroundReaction(u)
        return

    def addDummyForce(self):
        quadruped = self.quadruped
        #link = quadruped.front_lower_link
        #force = link.mass*self.g/2
        #quadruped.front_lower_link.body.apply_force_at_local_point((0,force), quadruped.front_lower_link.shape.b)
        #quadruped.front_lower_link.body.apply_force_at_world_point((0,force), quadruped.front_lower_link.get_end_position())
        #quadruped.front_lower_link.body.torque = 20000


        ball = self.ball
        force = ball.mass * self.g
        ball.apply_force_at_world_point((0,force), ball.position)


        M = quadruped.mass
        g = self.g
        l = quadruped.front_lower_link.length
        cos60 = cos(radians(60))
        torque = 0.95/2*M*g*l*cos60 + 0.025*M*g*l*cos60
        torque = torque * 1.1
        self.quadruped.front_lower_motor(-torque)
        self.quadruped.rear_lower_motor(-torque)
        return

    def checkExit(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                sys.exit(0)

    def loop(self):
        self.checkExit()

        screen = self.screen
        screen.fill((255,255,255))
        self.space.debug_draw(self.draw_options)

        # update control
        self.updateControl()
        #self.addDummyForce()


        screen.blit(pygame.transform.flip(screen,False,True), (0,0))
        pygame.display.flip()

        self.space.step(1/100.0)
        # limit framerate to 100
        self.clock.tick(100)


if __name__=="__main__":
    main = Sim()
    while True:
        main.loop()

