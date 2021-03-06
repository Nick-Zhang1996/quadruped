import sys
import numpy as np
from math import radians,degrees
import pygame
import pymunk
import pymunk.pygame_util


pygame.init()
screen = pygame.display.set_mode((600,600))
pygame.display.set_caption("Joints. Just wait and the L will tip over")

clock = pygame.time.Clock()

space = pymunk.Space()
space.gravity = (0.0, 900.0)

draw_options = pymunk.pygame_util.DrawOptions(screen)

static_body = pymunk.Body(body_type=pymunk.Body.STATIC)
static_body.position = (300,300)

angle = radians(10)
length = 500
start = length/2 * np.array([-np.cos(angle), -np.sin(angle)])
end = length/2 * np.array([np.cos(angle), np.sin(angle)])

l1 = pymunk.Segment(static_body, tuple(start), tuple(end), 10)
space.add(static_body,l1)

body = pymunk.Body(10, 100)
body.position = (300,250)
length = 50
start = length/2 * np.array([-np.cos(angle), -np.sin(angle)])
end = length/2 * np.array([np.cos(angle), np.sin(angle)])
l2 = pymunk.Segment(body, tuple(start), tuple(end), 5)
space.add(body, l2)

space.gravity = (0.0, 980)

mu = np.tan(radians(11))
l1.friction = 1
l2.friction = mu



while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit(0)
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            sys.exit(0)
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
            pygame.image.save(screen, "slide_and_pinjoint.png")

    space.step(1 / 50.0)

    screen.fill((255, 255, 255))
    space.debug_draw(draw_options)

    pygame.display.flip()
    clock.tick(50)
