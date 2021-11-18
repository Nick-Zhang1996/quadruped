from common import *
import sys
import pygame
import pymunk
import pymunk.pygame_util

import random
random.seed(1)
from math import radians,degrees,cos,sin
import numpy as np
from Controller import *

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


class Quadruped(PrintObject):
    def __init__(self, sim,  position):
        self.base_position = position
        self.mass = 20
        self.sim = sim
        self.space = sim.space
        self.links = []
        self.addLinks()

        # controller related
        self.controller_freq = 250
        #self.joint_torque = np.zeros((4,3))
        self.joint_torque = np.zeros(4)
        self.ground_reaction_force = np.zeros(4)
        # unit: sim time
        self.last_controller_update = 0
        self.controller = Controller(self)
        self.controller.buildModel()


    def addLinks(self):
        total_mass = self.mass
        self.base_link = self.createBaseLink()

        # length, initial angle, mass
        each_limb_mass = (1-self.base_mass_ratio)/4*self.mass
        self.front_upper_link = Link(40,-radians(120),each_limb_mass,  None, self.base_link, False)
        self.front_lower_link = Link(40,-radians(60), each_limb_mass, None, self.front_upper_link, False,True)
        self.rear_upper_link = Link(40,-radians(120), each_limb_mass, None, self.base_link, True)
        self.rear_lower_link = Link(40,-radians(60),  each_limb_mass,None, self.rear_upper_link, False, True)

        self.links.append(self.front_upper_link)
        self.links.append(self.front_lower_link)
        self.links.append(self.rear_upper_link)
        self.links.append(self.rear_lower_link)

        for link in self.links:
            link.add_to_space(self.space)

    def createBaseLink(self):
        total_mass = self.mass
        base_length = 100
        self.base_mass_ratio = 0.95
        base_mass = self.base_mass_ratio*total_mass
        #location, start, end

        # create base
        base_link_length = 100
        base_link_thickness = 10
        base_I = pymunk.moment_for_box(base_mass, (base_link_thickness, base_length))
        base_body = pymunk.Body(base_mass, base_I)
        #base_body = pymunk.Body(base_mass, base_I, body_type=pymunk.Body.STATIC)
        base_body.position = self.base_position
        base_body.angle = radians(0)
        shape = pymunk.Segment(base_body, (-base_link_length/2,0), (base_link_length/2,0), base_link_thickness/2)
        shape.collision_type = collision_types['body']
        base_link = Link()
        base_link.length = base_link_length
        base_link.shape = shape
        base_link.body = base_body
        self.links.append(base_link)
        return base_link

    def front_shoulder_motor(self, torque):
        self.front_upper_link.body.torque += torque
        self.base_link.body.torque += -torque

    def front_knee_motor(self, torque):
        self.front_lower_link.body.torque += torque
        self.front_upper_link.body.torque += -torque

    def rear_shoulder_motor(self, torque):
        self.rear_upper_link.body.torque += torque
        self.base_link.body.torque += -torque

    def rear_knee_motor(self, torque):
        self.rear_lower_link.body.torque += torque
        self.rear_upper_link.body.torque += -torque

    def front_shoulder_pos_np(self):
        return np.array(self.front_upper_link.get_start_position())
    def front_knee_pos_np(self):
        return np.array(self.front_upper_link.get_end_position())
    def front_foot_pos_np(self):
        return np.array(self.front_lower_link.get_end_position())
    def rear_shoulder_pos_np(self):
        return np.array(self.rear_upper_link.get_start_position())
    def rear_knee_pos_np(self):
        return np.array(self.rear_upper_link.get_end_position())
    def rear_foot_pos_np(self):
        return np.array(self.rear_lower_link.get_end_position())

    def front_shoulder_pos(self):
        return self.front_upper_link.get_start_position()
    def front_knee_pos(self):
        return self.front_upper_link.get_end_position()
    def front_foot_pos(self):
        return self.front_lower_link.get_end_position()
    def rear_shoulder_pos(self):
        return self.rear_upper_link.get_start_position()
    def rear_knee_pos(self):
        return self.rear_upper_link.get_end_position()
    def rear_foot_pos(self):
        return self.rear_lower_link.get_end_position()


    # cheating function, apply ground reaction force to base_link directly
    def applyGroundReactionCheat(self, u):
        # u = [fx1 fy1 fx2 fy2] ground->robot
        # apply force at actual foot location
        #self.base_link.body.apply_force_at_world_point( (u[0],u[1]), self.front_foot_pos())
        #self.base_link.body.apply_force_at_world_point( (u[2],u[3]), self.rear_foot_pos())
        # apply force at fake foot position
        # front
        self.base_link.body.apply_force_at_world_point( (u[0],u[1]), (150,100))
        # rear
        self.base_link.body.apply_force_at_world_point( (u[2],u[3]), (50,100))

    def applyGroundReaction(self, u):
        joint_torque = self.controller.calcJointTorque(u)
        self.front_knee_motor( joint_torque[0][2])
        self.front_shoulder_motor( joint_torque[1][2])
        self.rear_knee_motor( joint_torque[2][2])
        self.rear_shoulder_motor( joint_torque[3][2])

    def applyJointTorque(self, joint_torque):
        self.front_knee_motor( joint_torque[0])
        self.front_shoulder_motor( joint_torque[1])
        self.rear_knee_motor( joint_torque[2])
        self.rear_shoulder_motor( joint_torque[3])
        '''
        self.front_knee_motor( joint_torque[0][2])
        self.front_shoulder_motor( joint_torque[1][2])
        self.rear_knee_motor( joint_torque[2][2])
        self.rear_shoulder_motor( joint_torque[3][2])
        '''

    def controllerStep(self):
        if (self.sim.sim_steps * self.sim.sim_dt - self.last_controller_update > 1.0/self.controller_freq):
            js_vals = self.sim.joystick.vals
            pitch = js_vals['RV']*radians(30)
            dp = np.array([js_vals['LH'], -js_vals['LV']])*10
            #self.print_info("target pitch: %.2f(deg), x:%.1f, y:%.1f"%(degrees(pitch), dp[0], dp[1]))
            ground_reaction_force = self.controller.step(dp,pitch)

            self.joint_torque = self.controller.calcJointTorque(ground_reaction_force)
            self.last_controller_update = self.sim.sim_steps * self.sim.sim_dt
            self.sim.controller_steps += 1
            self.ground_reaction_force = ground_reaction_force
        self.applyJointTorque(self.joint_torque)
        #self.applyGroundReactionCheat(self.ground_reaction_force)

