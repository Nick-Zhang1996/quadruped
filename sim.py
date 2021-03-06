# physics simulation for quadruped
import sys
import pygame
import pymunk
import pymunk.pygame_util
from Joystick import *
from timeUtil import *

import random
random.seed(1)
from math import radians,degrees,cos,sin
from time import time,sleep

# NOTE: to make display with pygame simpler unit here used is scaled
# distance: cm, g: cm/s^2, force: N/100

from common import *
from Quadruped import *
from Event import *

class Sim(PrintObject):
    def __init__(self):
        self.t = execution_timer(True)
        self.width = 800
        self.height = 600
        self.sim_dt = 1.0/1000
        self.display_freq = 50
        self.real_to_sim_speedup = 1
        self.last_display_update = time()
        self.event = Event()
        self.event.sim = self

        self.display_steps = 0
        self.controller_steps = 0

        self.sim_steps = 0

        pygame.init()
        self.clock = pygame.time.Clock()
        self.joystick = Joystick(self.event)

        # screen setting
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.screen.fill((255,255,255))
        pygame.display.set_caption("Quadruped Simulation")
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)

        # space setting
        self.space = pymunk.Space()
        self.space.damping = 0.1
        self.g = 9.8*100
        self.space.gravity = (0.0, -self.g)

        self.setCollisionHandler()
        self.addGround()
        self.mytext = ""


        #self.font = pygame.freetype.Font("./font/data-unifon.ttf",24)
        self.font = pygame.freetype.SysFont(None,24)
        #text_surface, rect = font.render("Hello World 123", (0,0,0))
        #screen.blit(text_surface, (100,100))

        # DEBUG
        #self.ball = self.addBall()
        self.quadruped = Quadruped(self.event, (100,170))
        #self.quadruped = Quadruped(self.event, (100,270))

    def exit(self):
        self.print_info()
        self.t.summary()
        self.quadruped.exit()

    def setCollisionHandler(self):
        def collide(x,y,z):
            return True
        def no_collide(x,y,z):
            return False
        self.space.add_collision_handler(collision_types['ground'], collision_types['body']).begin = collide
        self.space.add_collision_handler(collision_types['body'], collision_types['body']).begin = no_collide
        self.space.add_collision_handler(collision_types['body'], collision_types['obstacle']).begin = collide

        # get contact forces for debugging
        #update_contact_forces = lambda a,s,d: self.updateContactForces(a, s, d)
        self.space.add_collision_handler(collision_types['ground'], collision_types['body']).post_solve = self.update_contact_forces
        return

    def update_contact_forces(self, arbiter, space, data):
        if (self.quadruped.front_lower_link.shape in arbiter.shapes or self.quadruped.front_lower_link.circle in arbiter.shapes):
            self.quadruped.front_ground_reaction = np.array(arbiter.total_impulse) / self.sim_dt
        elif (self.quadruped.rear_lower_link.shape in arbiter.shapes or self.quadruped.rear_lower_link.circle in arbiter.shapes):
            self.quadruped.rear_ground_reaction = np.array(arbiter.total_impulse) / self.sim_dt
        else:
            pass
            #self.print_warning("unexpected contact")

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
        self.ground_friction = 0.9
        segment.friction = self.ground_friction
        #segment.collision_type = None
        self.space.add(segment)

    def updateControl(self):
        self.quadruped.step()
        return

    def addDummyForce(self):
        quadruped = self.quadruped
        #link = quadruped.front_lower_link
        #force = link.mass*self.g/2
        #quadruped.front_lower_link.body.apply_force_at_local_point((0,force), quadruped.front_lower_link.shape.b)
        #quadruped.front_lower_link.body.apply_force_at_world_point((0,force), quadruped.front_lower_link.get_end_position())
        #quadruped.front_lower_link.body.torque = 20000


        '''
        ball = self.ball
        force = ball.mass * self.g
        ball.apply_force_at_world_point((0,force), ball.position)
        '''


        M = quadruped.mass
        g = self.g
        l = quadruped.front_lower_link.length
        cos60 = cos(radians(60))
        torque = 0.95/2*M*g*l*cos60 + 0.025*M*g*l*cos60
        torque = torque * 1.1
        self.quadruped.front_knee_motor(-torque)
        self.quadruped.rear_knee_motor(-torque)
        return

    def checkPygameEvent(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return True
            self.joystick.checkJoystickEvent(event)

    def updateDisplay(self):
        screen = self.screen
        self.space.debug_draw(self.draw_options)
        screen.blit(pygame.transform.flip(screen,False,True), (0,0))
        self.font.render_to(self.screen, (10,10), self.mytext, (0,0,0))
        pygame.display.flip()
        # do this for next frame, so that drawing between calls to this fun won't be overwritten
        screen.fill((255,255,255))

    def showText(self, text):
        self.mytext = text

    def loop(self):
        while (not self.checkPygameEvent()):
            t0 = time()
            self.step()
            sim_lead = (self.sim_dt) - (time()-t0)
            if(sim_lead > 0):
                sleep(sim_lead)
        self.exit()

    def step(self):
        t = self.t
        t.s()

        # update control (include drawing)
        t.s('updateControl')
        self.updateControl()
        t.e('updateControl')
        #self.addDummyForce()

        # update simulation
        t.s('sim step')
        self.quadruped.preStep()
        self.space.step(self.sim_dt)
        t.e('sim step')
        self.sim_steps += 1
        t.s('display update')
        if (time() - self.last_display_update > 1.0/self.display_freq):
            self.last_display_update = time()

            self.showText("sim time = %.3f"%(self.sim_steps*self.sim_dt))
            #self.showText("sim: %d, control: %d, display %d" %(self.sim_steps, self.controller_steps, self.display_steps))
            self.updateDisplay()
            self.display_steps += 1
        t.e('display update')
        t.e()

if __name__=="__main__":
    main = Sim()
    main.loop()


