# physics simulation for quadruped
import sys
import pygame
import pymunk
import pymunk.pygame_util

import random
random.seed(1)

# NOTE: to make display with pygame simpler unit here used is scaled
# distance: cm, g: cm/s^2, force: N/100

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
        self.space.gravity = (0.0, -9.8*100)
        self.addGround()

        # DEBUG
        self.addBall()
    def addBall(self):
        """Add a ball to the given space at a random position"""
        mass = 3
        radius = 25
        inertia = pymunk.moment_for_circle(mass, 0, radius, (0,0))
        body = pymunk.Body(mass, inertia)
        x = random.randint(120,300)
        body.position = x, self.ground_height + 100
        shape = pymunk.Circle(body, radius, (0,0))
        shape.friction = 1
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

        self.space.step(1/50.0)

        pygame.display.flip()
        # limit framerate to 50
        self.clock.tick(50)

if __name__=="__main__":
    main = Physics()
    while True:
        main.loop()

