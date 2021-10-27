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
        r_help = [-r[0,1], r[0,1], -r[1,1], r[1,1]]
        moment = quadruped.base_link.body.moment
        mass = quadruped.mass
        B[3,:] = 1/moment * np.array(r_help)
        B[4,:] = 1/mass * np.array([1,1,0,0])
        B[5,:] = 1/mass * np.array([0,0,1,1])

        f = np.array([0,0,0,0,0,-self.g*mass])

        G = np.eye(n) + A*self.dt
        H = B*self.dt
        F = f*self.dt

        x_ref_world = np.hstack([[0],p_ref_world,[0,0,0]])

        N = self.N # horizon
        # formulate optimization problem
        # J = sum( (x_ref - x).T Q (x_ref-x) + u.T R u )
        Q = np.eye(n*N)
        R = np.eye(m*N)*0.01

        model = gp.Model("stand")
        # [x1,x2,...xN]
        x = model.addMVar(shape=n*N, name='x')
        # [u0, .... u(N-1)]
        u = model.addMVar(shape=m*N, name='u')

        body = quadruped.base_link.body
        x0 = [body.angle, body.position[0], body.position[1], body.angular_velocity, body.velocity[0], body.velocity[1]]
        x0 = np.array(x0)
        model.addConstr( x[:n] == G @ x0 + H @ u[:m] + F )
        for i in range(N-1):
            model.addConstr( x[n*(i+1):n*(i+2)] == G @ x[n*i:n*(i+1)] + H @ u[m*(i+1):m*(i+2)] + F )

        # broadcast x_ref to fill horizon
        xr = np.repeat(x_ref_world, N)
        obj = xr @ Q @ xr + x @ Q @ x - 2* xr @ Q @ x + u @ R @ u
        model.setObjective(obj, GRB.MINIMIZE)
        model.optimize()
        # return ground reaction
        #return u.x
        return (0,0,0,0)


