# NOTE: possible performance improvement by setting p_ref and x0 as constraind variables, and leave dynamics constraints untouched.
from common import *
import pygame
import gurobipy as gp
from gurobipy import GRB
import numpy as np
from collections import namedtuple
from recordclass import recordclass
from timeUtil import *
from math import degrees,radians
from PidController import *

class Controller(PrintObject):
    def __init__(self,event, dt, horizon):
        self.quadruped = event.quadruped
        self.sim = event.sim
        self.event = event
        self.g = 9.8*100
        # discretization step for MPC 0.05
        self.dt = dt
        self.N = horizon

        self.t = execution_timer(True)
        np.set_printoptions(precision=4)
        self.gurobi_constraints = []
        return

    def exit(self):
        self.print_info()
        self.t.summary()

    # build gurobi model
    # centroidal dynamics, mpc
    # dp: position offset from ref (100,170)
    # pitch: target pitch in rad
    def buildModel(self):
        quadruped = self.quadruped
        # state dimension
        n = 6
        # control dimension, legs*dim
        m = 4
        N = self.N # horizon

        # leg contact position, world frame
        # leg no: 0:front 1:rear
        # constant when standing
        dim = 2
        leg_count = 2

        # x = [pitch, x, y, omega, vx, vy ]
        # p = [x,y]
        # u = [fx1 fy1 fx2 fy2] ground->robot
        # dxdt = Ax + Bu + f
        # x+ = (I + A dt) x + (B*dt) u + f*dt
        #        G             H         F
        A = np.zeros((n,n))
        A[0,3] = 1 
        A[1,4] = 1
        A[2,5] = 1

        d = np.array([0,0,0,0,0,-self.g])

        G = np.eye(n) + A*self.dt

        model = gp.Model("stand")

        # supress output
        model.setParam(GRB.Param.OutputFlag, 0)
        # [x1,x2,...xN]
        x = model.addMVar(shape=n*N, lb=-10000, ub=10000, name='x')
        # [u0, .... u(N-1)]
        # simple constraint
        # TODO incorporate fy>Fmin here
        max_force = 80e3
        u = model.addMVar(shape=m*N, lb=-max_force, ub=max_force, name='u')

        # constraint on ground reaction (friction limit)
        mu = self.sim.ground_friction*0.9
        for i in range(N):
            # |fx| < mu*fy
            model.addConstr( u[m*i + 0] <= mu * u[m*i + 1])
            model.addConstr( u[m*i + 0] >= -mu * u[m*i + 1])
            # fy > 0
            model.addConstr( u[m*i + 1] >= 1000 )
            # |fx| < mu*fy
            model.addConstr( u[m*i + 2] <= mu * u[m*i + 3])
            model.addConstr( u[m*i + 2] >= -mu * u[m*i + 3])
            # fy > 0
            model.addConstr( u[m*i + 3] >= 1000 )

        self.model = model
        self.gurobi_x = x
        self.gurobi_u = u
        self.d = d
        self.G = G
        self.A = A
        self.normalGain()
        return

    def normalGain(self):
        n = 6
        # control dimension, legs*dim
        m = 4
        N = self.N # horizon
        Q = np.eye(n)
        # x = [pitch, x, y, omega, vx, vy ]
        Q[0,0] *= 57*10
        Q[1,1] *= 1
        Q[2,2] *= 1
        Q[3,3] *= 57*0.01
        Q[4,4] *= 0.01
        Q[5,5] *= 0.01

        '''
        Q[0,0] *= 1
        Q[1,1] *= 10
        Q[2,2] *= 10
        Q[3,3] *= 1
        Q[4,4] *= 0.0001
        Q[5,5] *= 0.0001
        '''
        R = np.eye(m)*1e-8
        #R = np.zeros((m,m))

        bigQ = np.kron(np.eye(N),Q)
        bigR = np.kron(np.eye(N),R)
        self.Q = Q
        self.bigQ = bigQ
        self.R = R
        self.bigR = bigR
        self.dt = 0.02

    def velocityGain(self):
        n = 6
        # control dimension, legs*dim
        m = 4
        N = self.N # horizon
        Q = np.eye(n)
        # x = [pitch, x, y, omega, vx, vy ]
        #Q[0,0] *= 57*10
        Q[0,0] *= 57*100
        Q[1,1] *= 0
        Q[2,2] *= 0
        #Q[3,3] *= 57*0.01
        Q[3,3] *= 57
        Q[4,4] *= 0.4
        Q[5,5] *= 0.8
        R = np.eye(m)*1e-8

        cost_over_horizon = np.diag(np.linspace(1,0.1,N))

        bigQ = np.kron(cost_over_horizon,Q)
        bigR = np.kron(cost_over_horizon,R)
        self.Q = Q
        self.bigQ = bigQ
        self.R = R
        self.bigR = bigR
        self.dt = 0.005

    # step controller, select contact leg or swing leg
    # contact schedule: N*2 (bool)
    # target_state: N*n (float)
    # target_angle: 4, only valid if a leg is scheduled to be swinging
    def step(self,contact_schedule, target_state, target_angle):
        self.ground_reaction_force = self.contactLegControl(contact_schedule, target_state)
        self.joint_torque = self.calcJointTorque(self.ground_reaction_force, contact_schedule[0], target_angle)


    def getJointTorque(self):
        return self.joint_torque
    def getGroundReactionForce(self):
        return self.ground_reaction_force

    # TODO handle single leg contact
    def contactLegControl(self, contact_schedule, target_state):
        if (not any(contact_schedule[0])):
            return np.array([0,0,0,0])
        # broadcast x_ref to fill horizon
        # x1,x2,x3.. x1,x2,x3
        xr = target_state.flatten()

        model = self.model
        x = self.gurobi_x
        u = self.gurobi_u
        bigQ = self.bigQ
        bigR = self.bigR
        d = self.d
        G = self.G
        A = self.A

        t = self.t
        t.s()
        t.s('Controller Prep')
        quadruped = self.quadruped
        # reference position of robot, world frame
        # state dimension
        n = 6
        # control dimension, legs*dim
        m = 4
        N = self.N # horizon

        # leg contact position, world frame
        # leg no: 0:front 1:rear
        # constant when standing
        dim = 2
        leg_count = 2
        r_world = np.zeros((leg_count, dim))
        r_world[0] = quadruped.front_foot_pos()
        r_world[1] = quadruped.rear_foot_pos()

        # check alignment
        p_world = np.array(self.quadruped.base_link.body.position)
        r_local = r_world - p_world

        # x = [pitch, x, y, omega, vx, vy ]
        # p = [x,y]
        # u = [fx1 fy1 fx2 fy2] ground->robot
        # dxdt = Ax + Bu + f
        # x+ = (I + A dt) x + (B*dt) u + f*dt
        #        G             H         F
        B = np.zeros((n,m))
        r = r_local
        r_help = [-r[0,1], r[0,0], -r[1,1], r[1,0]]
        moment = quadruped.base_link.body.moment
        mass = quadruped.mass

        B[3,:] = 1/moment * np.array(r_help)
        B[4,:] = 1/mass * np.array([1,0,1,0])
        B[5,:] = 1/mass * np.array([0,1,0,1])

        H = B*self.dt
        F = d*self.dt


        # formulate optimization problem
        # J = sum( (x_ref - x).T Q (x_ref-x) + u.T R u )
        # x = [pitch, x, y, omega, vx, vy ]
        t.e('Controller Prep')

        body = quadruped.base_link.body
        x0 = [body.angle, body.position[0], body.position[1], body.angular_velocity, body.velocity[0], body.velocity[1]]
        x0 = np.array(x0)

        t.s("constrain dynamics")
        [model.remove(constr) for constr in self.gurobi_constraints]
        self.gurobi_constraints = []

        constr = model.addConstr( x[:n] == G @ x0 + H @ u[:m] + F )
        self.gurobi_constraints.append(constr)
        for i in range(N-1):
            constr = model.addConstr( x[n*(i+1):n*(i+2)] == G @ x[n*i:n*(i+1)] + H @ u[m*(i+1):m*(i+2)] + F )
            self.gurobi_constraints.append(constr)
        t.e("constrain dynamics")


        #obj = xr @ Q @ xr + x @ Q @ x - 2* xr @ Q @ x + u @ R @ u
        obj = xr @ bigQ @ xr + x @ bigQ @ x - 2* xr @ bigQ @ x + u @ bigR @ u
        t.s("obj")
        model.setObjective(obj, GRB.MINIMIZE)
        t.e("obj")

        t.s('Gurobi optimize')
        model.optimize()
        t.e('Gurobi optimize')

        t.s('Post Processing')
        # find total torque 
        control = u.x[:4]
        # x acceleration
        x_acc = 1/mass * ( control[0] + control[2] )
        y_acc = 1/mass * ( control[1] + control[3] )  - self.g
        w = 1/moment * (x_acc*mass * (-r[0,1]) + control[1] * r[0,0] + control[3] * r[1,0])
        #self.print_info("ax: %.2f, ay: %.2f, a: %.2f"%(x_acc, y_acc, w))
        #self.print_info("control: ", u.x[:4])
        ctrl = u.x[:4]
        #self.print_info("requested",(u.x[0] + u.x[2], u.x[1] + u.x[3]))
        #self.print_info("actual",self.quadruped.front_ground_reaction + self.quadruped.rear_ground_reaction)

        # return ground reaction
        #self.print_info("Fground",u.x[:4])
        t.e('Post Processing')
        t.e()

        # draw anticipated trajectory
        x_ref_world = xr.reshape((N,n))

        if (self.sim.joystick.button['S']):
            Q = self.Q
            R = self.R
            x_predict = [x0]
            J = 0
            for i in range(N):
                ctrl = u.x[m*i:m*(i+1)]
                x_new = G @ x_predict[-1] + H @ ctrl + F
                J += (x_new - x_ref_world[i]).T @ Q @ (x_new - x_ref_world[i]) + ctrl.T @ R @ ctrl
                x_predict.append(x_new)
                #pygame.draw.circle(self.screen, (0,0,0), x_new[1:3], 2)
            x_predict = np.array(x_predict)
            self.print_info("expected trajectory \n", x_predict)
            self.print_info("expected x_f", x_predict[-1])
            #self.print_info("expected J = %.2f"%(J))
            self.print_info("current angle: %.2f expected EOH angle: %.2f"%(degrees(x0[0]), degrees(x_predict[-1,0])))
            breakpoint()

        x_expected = x.x.reshape((N,n))

        '''
        dv0 = (x_ref_world[0] - x0)[4:]
        dvf = (x_ref_world[0] - x_expected[-1])[4:]
        ratio = np.linalg.norm(dvf) / np.linalg.norm(dv0)
        self.print_info("dv0 =", dv0, "dvf =", dvf, "ratio = %.2f"%ratio, u.x[:4], "%.2f"%body.angular_velocity)
        if (self.sim.joystick.button['S']):
            ctrl = u.x.reshape((N,m))
            print(ctrl)
            print(x_expected)
            breakpoint()
        '''

        self.print_info(u.x[:4])
        return u.x[:4]

    # given desired ground reaction force at foot (Fx,Fy)
    # calculate and joint torques
    # Fground: force at foot, ground -> foot
    def calcJointTorque(self, Fground, contact_schedule, target_angle):
        quadruped = self.quadruped
        Joint = recordclass('Joint', 'pos torque angle')
        Foot = namedtuple('Foot', 'pos Fground')

        front_foot = Foot(quadruped.front_foot_pos_np(), np.array([Fground[0],Fground[1],0]))
        front_knee_joint = Joint(quadruped.front_knee_pos_np(), 0, 0)
        front_shoulder_joint = Joint(quadruped.front_shoulder_pos_np(), 0, 0)

        rear_foot = Foot(quadruped.rear_foot_pos_np(), np.array([Fground[2], Fground[3],0]))
        rear_knee_joint = Joint(quadruped.rear_knee_pos_np(), 0, 0)
        rear_shoulder_joint = Joint(quadruped.rear_shoulder_pos_np(), 0, 0)
        base_angle = quadruped.base_link.body.angle
        base_omega = quadruped.base_link.body.angular_velocity
        # front
        if (contact_schedule[0]):
            # feed forward
            # torque = - (pos_joint - pos_foot) X Fground
            front_knee_joint.torque = -np.cross(front_foot.pos - front_knee_joint.pos, front_foot.Fground)[2]
            front_shoulder_joint.torque = -np.cross(front_foot.pos - front_shoulder_joint.pos, front_foot.Fground)[2]
        else:
            if (quadruped.front_foot_contact != contact_schedule[0]):
                quadruped.front_knee_joint_pid.reset()
                quadruped.front_shoulder_joint_pid.reset()
            # swing leg control
            angle = quadruped.front_lower_link.body.angle - quadruped.front_upper_link.body.angle - target_angle[0]
            angle = self.wrap(angle) + target_angle[0]
            omega = quadruped.front_lower_link.body.angular_velocity - quadruped.front_lower_link.body.angular_velocity
            front_knee_joint.torque = quadruped.front_knee_joint_pid.control(target_angle[0], angle, omega)

            angle = quadruped.front_upper_link.body.angle - base_angle - target_angle[1]
            angle = self.wrap(angle) + target_angle[1]
            omega = quadruped.front_upper_link.body.angular_velocity - base_omega
            front_shoulder_joint.torque = quadruped.front_shoulder_joint_pid.control(target_angle[1], angle, omega)
        quadruped.front_foot_contact = contact_schedule[0]

        if (contact_schedule[1]):
            rear_knee_joint.torque = -np.cross(rear_foot.pos - rear_knee_joint.pos, rear_foot.Fground)[2]
            rear_shoulder_joint.torque = -np.cross(rear_foot.pos - rear_shoulder_joint.pos, rear_foot.Fground)[2]
        else:
            if (quadruped.rear_foot_contact != contact_schedule[0]):
                quadruped.rear_knee_joint_pid.reset()
                quadruped.rear_shoulder_joint_pid.reset()
            # swing leg control
            angle = quadruped.rear_lower_link.body.angle - quadruped.rear_upper_link.body.angle - target_angle[2]
            angle = self.wrap(angle) + target_angle[2]
            omega = quadruped.rear_lower_link.body.angular_velocity - quadruped.rear_lower_link.body.angular_velocity
            rear_knee_joint.torque = quadruped.rear_knee_joint_pid.control(target_angle[2], angle, omega)

            angle = quadruped.rear_upper_link.body.angle - base_angle - target_angle[3]
            angle = self.wrap(angle) + target_angle[3]
            omega = quadruped.rear_upper_link.body.angular_velocity - base_omega
            rear_shoulder_joint.torque = quadruped.rear_shoulder_joint_pid.control(target_angle[3], angle, omega)
        quadruped.rear_foot_contact = contact_schedule[1]

        joints = [front_knee_joint, front_shoulder_joint, rear_knee_joint, rear_shoulder_joint]
        joint_torques =  [joint.torque for joint in joints]
        '''
        if (self.sim.joystick.button['S']):
            breakpoint()
        '''
        return joint_torques

    def wrap(self, angle):
        return (angle + 3*np.pi) % (2*np.pi) - np.pi

