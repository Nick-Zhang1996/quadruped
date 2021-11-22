from common import *
import sys
import pymunk

from math import radians,degrees,cos,sin
import numpy as np

class Link(PrintObject):
    # create a link, attached to parent_body at it's a or b point (start/finish), also add a joint at connection. Start(a) of this link is the connecting point
    def __init__(self, length=None, angle=None, mass=None, moment=None, parent_link=None, joint_at_start=True, add_circle_at_end = False):
        self.length = length
        self.shape = None
        self.body = None
        self.thickness = 4
        self.mass = None
        self.joint = None
        self.circle = None
        self.front_ground_reaction = (0,0)
        self.rear_ground_reaction = (0,0)
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

        self.mass = mass
        self.body = pymunk.Body(mass,moment)
        x = start_pos[0] + length/2.0*cos(angle)
        y = start_pos[1] + length/2.0*sin(angle)
        self.body.position = (x,y)
        self.body.angle = angle
        self.shape = pymunk.Segment(self.body, ( -length/2,0), (length/2,0), self.thickness/2)
        self.shape.collision_type = collision_types['body']
        self.shape.friction = 1.0
        if (add_circle_at_end):
            self.circle = pymunk.Circle(self.body, 3.0, (length/2,0))
            self.circle.friction = 1.0
            self.circle.collision_type = collision_types['body']

        # this is the joint connecting this link to the parent link
        if (joint_at_start):
            self.joint = pymunk.PivotJoint(parent_link.body, self.body, parent_link.shape.a, self.shape.a)
        else:
            self.joint = pymunk.PivotJoint(parent_link.body, self.body, parent_link.shape.b, self.shape.a)
        # orientation of body 
        #self.pivot_limit = pymunk.RotaryLimitJoint(parent_link.body, self.body, -radians(60), radians(60))
        # simple motor keeps a constant velocity
        #self.angular_velocity = pymunk.SimpleMotor(parent_link.body, self.body, 0)

    def add_to_space(self,space):
        space.add(self.body, self.shape)
        if (not self.joint is None):
            space.add(self.joint)
        if (not self.circle is None):
            space.add(self.circle)


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

