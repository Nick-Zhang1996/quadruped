from common import *
import sys
import pygame
import pymunk
import pymunk.pygame_util

import random
random.seed(1)
from math import radians,degrees,cos,sin
import numpy as np
from StepPlanner import *
from Link import *
from Event import *


class Quadruped(PrintObject):
    def __init__(self, event,  position):
        self.base_position = position
        self.mass = 20
        self.event = event
        self.event.quadruped = self
        self.sim = event.sim
        self.space = event.sim.space
        self.links = []
        self.addLinks()

        # controller related
        self.controller_freq = 250
        # front knee, front shoulder, rear knee, rear, shoulder
        self.joint_torque = np.zeros(4)
        # front_x, front_y, rear_x, rear_y
        self.ground_reaction_force = np.zeros(4)
        # unit: sim time
        self.last_controller_update = 0

        self.step_planner = StepPlanner(event)
        self.step_planner.getTime = self.getTime
        self.step_planner.setPlan('stand')

        self.front_foot_contact = False
        self.rear_foot_contact = False
        dt = event.step_planner.dt
        P = 5e3*0
        I = 1e3*0
        D = 1e3*0
        self.front_shoulder_joint_pid = PidController(P,I,D,dt,10000,1000)
        self.front_knee_joint_pid = PidController(P,I,D,dt,10000,1000)
        self.rear_shoulder_joint_pid = PidController(P,I,D,dt,10000,1000)
        self.rear_knee_joint_pid = PidController(P,I,D,dt,10000,1000)

    def exit(self):
        self.step_planner.exit()

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

    def getFrontKneeAngle(self):
        return self.front_lower_link.body.angle - self.front_upper_link.body.angle
    def getFrontShoulderAngle(self):
        return self.front_upper_link.body.angle - self.base_link.body.angle
    def getRearKneeAngle(self):
        return self.rear_lower_link.body.angle - self.rear_upper_link.body.angle
    def getRearShoulderAngle(self):
        return self.rear_upper_link.body.angle - self.base_link.body.angle


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

    # call step planner/controller if necessary
    # actuate joints accordingly
    def step(self):
        if (self.sim.sim_steps * self.sim.sim_dt - self.last_controller_update > 1.0/self.controller_freq):
            self.step_planner.step()
            self.joint_torque = self.step_planner.getJointTorque()
            self.ground_reaction_force = self.step_planner.getGroundReactionForce()

            self.last_controller_update = self.sim.sim_steps * self.sim.sim_dt
            self.sim.controller_steps += 1
        self.applyJointTorque(self.joint_torque)
        #self.applyGroundReactionCheat(self.ground_reaction_force)

    def getTime(self):
        return self.sim.sim_steps*self.sim.sim_dt
    def getState(self):
        body = self.base_link.body
        return np.array([body.angle, body.position[0], body.position[1], body.angular_velocity, body.velocity[0], body.velocity[1]])
