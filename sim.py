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

collision_types = {'ground':0, 'body':1, 'obstacle':2}

class Link:
    # create a link, attached to parent_body at it's a or b point (start/finish), also add a joint at connection. Start(a) of this link is the connecting point
    def __init__(self, length=None, angle=None, mass=None, moment=None, parent_link=None, joint_at_start=True):
        self.length = length
        self.shape = None
        self.body = None
        self.thickness = 4
        self.joint = None
        if (parent_link is None):
            # return empty class
            return
        else:
            if (joint_at_start):
                start_pos = parent_link.get_start_position() 
            else:
                start_pos = parent_link.get_end_position()


        if (moment is None):
            moment = pymunk.moment_for_box(mass, (self.thickness, self.length))

        self.body = pymunk.Body(mass,moment)
        x = start_pos[0] + length/2.0*cos(angle)
        y = start_pos[1] + length/2.0*sin(angle)
        self.body.position = (x,y)
        self.body.angle = angle
        self.shape = pymunk.Segment(self.body, ( -length/2,0), (length/2,0), self.thickness/2)
        self.shape.collision_type = collision_types['body']
        self.shape.friction = 1.0

        # this is the joint connecting this link to the parent link
        if (joint_at_start):
            self.joint = pymunk.PivotJoint(parent_link.body, self.body, parent_link.shape.a, self.shape.a)
        else:
            self.joint = pymunk.PivotJoint(parent_link.body, self.body, parent_link.shape.b, self.shape.a)
        #self.pivot_limit = pymunk.RotaryLimitJoint(parent_link.body, self.body, -radians(60), radians(60))

    def add_to_space(self,space):
        space.add(self.body, self.shape)
        if (not self.joint is None):
            space.add(self.joint)


    def get_start_position(self):
        pos = self.body.position
        angle = self.body.angle
        x = pos[0] - self.length/2.0*cos(angle)
        y = pos[1] - self.length/2.0*sin(angle)
        return (x,y)

    def get_end_position(self):
        pos = self.body.position
        angle = self.body.angle
        x = pos[0] + self.length/2.0*cos(angle)
        y = pos[1] + self.length/2.0*sin(angle)
        return (x,y)



class Physics:
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
        self.space.damping = 0.9
        self.space.gravity = (0.0, -9.8*100)
        self.setCollisionHandler()
        #self.space.gravity = (0.0, 0)
        self.addGround()
        self.links = []

        # DEBUG
        #self.addBall()
        self.addQuadruped()

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
        return shape

    def addGround(self):
        radius = 5
        # (x,y) origin at upper left, x+ right y+down
        self.ground_height = ground_height = 100
        segment = pymunk.Segment(self.space.static_body, (0, ground_height), (self.width,ground_height), radius)
        segment.friction = 0.9
        #segment.collision_type = None
        self.space.add(segment)

    def addQuadruped(self):
        total_mass = 20
        base_link = self.createBaseLink()

        front_upper_link = Link(40,-radians(120), total_mass*0.1/4, None, base_link, False)
        front_lower_link = Link(40,-radians(60), total_mass*0.1/4, None, front_upper_link, False)
        rear_upper_link = Link(40,-radians(120), total_mass*0.1/4, None, base_link, True)
        rear_lower_link = Link(40,-radians(60), total_mass*0.1/4, None, rear_upper_link, False)

        self.links.append(front_upper_link)
        self.links.append(front_lower_link)
        self.links.append(rear_upper_link)
        self.links.append(rear_lower_link)

        for link in self.links:
            link.add_to_space(self.space)

    def createBaseLink(self):
        total_mass = 20
        base_length = 100
        base_mass = 0.9*total_mass
        #location, start, end

        # create base
        base_link_length = 100
        base_link_thickness = 10
        base_I = pymunk.moment_for_box(base_mass, (base_link_thickness, base_length))
        base_body = pymunk.Body(base_mass, base_I)
        #base_body = pymunk.Body(base_mass, base_I, body_type=pymunk.Body.STATIC)
        base_body.position = (200,200)
        base_body.angle = radians(0)
        shape = pymunk.Segment(base_body, (-base_link_length/2,0), (base_link_length/2,0), base_link_thickness/2)
        shape.collision_type = collision_types['body']
        base_link = Link()
        base_link.length = base_link_length
        base_link.shape = shape
        base_link.body = base_body
        self.links.append(base_link)
        return base_link


    def loop(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                sys.exit(0)

        screen = self.screen
        screen.fill((255,255,255))
        self.space.debug_draw(self.draw_options)
        #screen.blit(pygame.transform.rotate(screen,180), (0,0))
        screen.blit(pygame.transform.flip(screen,False,True), (0,0))

        self.space.step(1/100.0)

        pygame.display.flip()
        # limit framerate to 100
        self.clock.tick(100)

if __name__=="__main__":
    main = Physics()
    while True:
        main.loop()

