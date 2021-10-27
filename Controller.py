import pygame
import gurobipy as gp
from gurobipy import GRB
import numpy as np

class Controller:
    def __init__(self,quadruped):
        self.quadruped = quadruped
        self.g = 9.81*100
        self.dt = 0.01
        # horizon
        self.N = 20
        return

    # centroidal dynamics, mpc
    def stand(self):
        quadruped = self.quadruped
        # reference position of robot, world frame
        # P_ref_world(t) = (x,y)
        # constant when standing
        p_ref_world = np.array([100,170])
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
        r_local = r_world - p_ref_world

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

        B = np.zeros((n,m))
        r = r_local
        r_help = [-r[0,1], r[0,0], -r[1,1], r[1,0]]
        moment = quadruped.base_link.body.moment
        mass = quadruped.mass

        B[3,:] = 1/moment * np.array(r_help)
        B[4,:] = 1/mass * np.array([1,0,1,0])
        B[5,:] = 1/mass * np.array([0,1,0,1])

        f = np.array([0,0,0,0,0,-self.g])

        G = np.eye(n) + A*self.dt
        H = B*self.dt
        F = f*self.dt

        x_ref_world = np.hstack([[0],p_ref_world,[0,0,0]])

        # formulate optimization problem
        # J = sum( (x_ref - x).T Q (x_ref-x) + u.T R u )
        Q = np.eye(n)
        # x = [pitch, x, y, omega, vx, vy ]
        Q[0,0] *= 50
        R = np.eye(m)*1e-8

        bigQ = np.kron(np.eye(N),Q)
        bigR = np.kron(np.eye(N),R)

        model = gp.Model("stand")
        # supress output
        #model.setParam(GRB.Param.OutputFlag, 0)
        # [x1,x2,...xN]
        x = model.addMVar(shape=n*N, lb=-10000, ub=10000, name='x')
        # [u0, .... u(N-1)]
        # simple constraint
        u = model.addMVar(shape=m*N, lb=-20e3, ub=20e3, name='u')

        body = quadruped.base_link.body
        x0 = [body.angle, body.position[0], body.position[1], body.angular_velocity, body.velocity[0], body.velocity[1]]
        x0 = np.array(x0)
        model.addConstr( x[:n] == G @ x0 + H @ u[:m] + F )
        for i in range(N-1):
            model.addConstr( x[n*(i+1):n*(i+2)] == G @ x[n*i:n*(i+1)] + H @ u[m*(i+1):m*(i+2)] + F )

        # broadcast x_ref to fill horizon
        xr = np.repeat(x_ref_world.reshape(1,-1),N,axis=0).flatten()
        #obj = xr @ Q @ xr + x @ Q @ x - 2* xr @ Q @ x + u @ R @ u
        obj = xr @ bigQ @ xr + x @ bigQ @ x - 2* xr @ bigQ @ x + u @ bigR @ u
        model.setObjective(obj, GRB.MINIMIZE)
        model.optimize()

        # verify constraints and model
        '''
        x_post = x.x
        u_post = u.x
        print("verify")
        print("step 0 ")
        print(x_post[:n] - ( G @ x0 + H @ u_post[:m] + F))
        for i in range(N-1):
            print("step %d"%i)
            error = x_post[n*(i+1):n*(i+2)] - (G @ x_post[n*i:n*(i+1)] + H @ u_post[m*(i+1):m*(i+2)] + F)
            print(np.linalg.norm(error))
            print(error)
        '''

        # draw anticipated trajectory
        x_predict = [x0]
        J = 0
        for i in range(N):
            ctrl = u.x[m*i:m*(i+1)]
            x_new = G @ x_predict[-1] + H @ ctrl + F
            J += (x_new - x_ref_world).T @ Q @ (x_new - x_ref_world) + ctrl.T @ R @ ctrl
            x_predict.append(x_new)
            pygame.draw.circle(self.screen, (0,0,0), x_new[1:3], 2)
        x_predict = np.array(x_predict)
        print("x0 pos error")
        print(p_ref_world - x0[1:3])
        print("x0 norm error")
        print(np.linalg.norm(x_ref_world - x0))
        print("xf pos error")
        print(p_ref_world - x_predict[-1,1:3])
        print("xf norm error")
        print(np.linalg.norm(x_ref_world - x_predict[-1]))
        print("J")
        print(J)
        print("u control")
        print(u.x)

        # return ground reaction
        return u.x 




