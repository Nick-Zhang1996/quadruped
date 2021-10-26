# physics simulation for quadruped
import sys
import pygame
import pymunk
import pymunk.pygame_util

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
        self.space.gravity = (0.0, 9.8)
        self.addGround()

    def addGround(self):
        radius = 5
        # (x,y) origin at upper left, x+ right y+down
        ground_height = 100
        segment = pymunk.Segment(self.space.static_body, (100, ground_height), (self.width/2,ground_height), radius)
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

