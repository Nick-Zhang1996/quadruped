from common import *
from math import degrees,radians
from Controller import *
class StepPlanner(PrintObject):
    def __init__(self, sim, quadruped):
        self.sim = sim
        self.quadruped = quadruped
        self.horizon = 10
        self.dt = 0.05
        self.current_plan = None
        self.current_plan_t0 = 0
        self.name_to_plan = {}
        self.getTime = self.dummyGetTime
        self.joint_torque = [0,0,0,0]
        self.ground_reaction_force = [0,0,0,0]
        # WBC
        self.controller = Controller(self.quadruped, self.dt, self.horizon)
        self.controller.buildModel()

        self.newPlan('stand',self.getPlanStand)
        # jump = squat + jump + in_air + landing
        self.newPlan('jump',self.getPlanSquat)
    def exit(self):
        self.controller.exit()

    def dummyGetTime(self):
        print_error("set this to a function that returns current time")

    def newPlan(self, name, handle):
        self.name_to_plan[name] = handle

    # start executing a plan
    # name: stand, squat, jump, land
    def setPlan(self,name):
        self.current_plan_t0 = time()
        self.current_plan = self.name_to_plan[name]
        self.getPlan(True)

    # contact schedule: N*2 (bool)
    # target_state: N*n (float)
    # time_left: float
    def step(self):
        contact_schedule, target_state, time_left = self.getPlan()
        Fground = self.controller.step(contact_schedule, target_state)
        self.ground_reaction_force = Fground
        self.joint_torque = self.controller.calcJointTorque(self.ground_reaction_force)
        pass

    def getJointTorque(self):
        return self.joint_torque


    def getGroundReactionForce(self):
        return self.ground_reaction_force

    # horizon: plan horizon steps
    # dt: plan step size
    # return: 
    # contact schedule
    # target pitch,x,y,omega, vx,vy
    # remaining time in plan, <0 means to switch
    def getPlan(self, first_call = False):
        return self.current_plan(first_call)

    # getPlan for squatting down
    # TODO
    def getPlanSquat(self, first_call):
        return contact_schedule, target_state,1

    # getPlan for jump
    # TODO
    def getPlanJump(self, first_call):
        return contact_schedule, target_state,1

    # getPlan for standing
    def getPlanStand(self, first_call):
        elapsed_time = self.getTime() - self.current_plan_t0
        if (first_call):
            self.stand_target_x = self.quadruped.base_link.body.position[0]
            self.stand_original_y = self.quadruped.base_link.body.position[1]
            self.stand_transition_duration = 1
            self.stand_target_y = 160
            self.stand_y_vel = (self.stand_target_y - self.stand_original_y)/self.stand_transition_duration
        contact_schedule = [[True,True] for i in range(self.horizon)]
        if (elapsed_time > self.stand_transition_duration):
            ref_state = np.array([0, self.stand_target_x, self.stand_target_y, 0, 0, 0])
            target_state = np.repeat(ref_state.reshape(1,-1),self.horizon,axis=0)
        else:
            target_state = []
            for i in range(self.horizon):
                current_step_time = elapsed_time + self.dt * i
                if (current_step_time < self.stand_transition_duration):
                    y = map(current_step_time, elapsed_time, elapsed_time+self.stand_transition_duration, self.stand_original_y, self.stand_target_y)
                    ref_state = np.array([0, self.stand_target_x, y, 0, 0, self.stand_y_vel])
                else:
                    ref_state = np.array([0, self.stand_target_x, self.stand_target_y, 0, 0, 0])
                target_state.append(ref_state)
            target_state = np.array(target_state)

        return contact_schedule, target_state,1

    
        


