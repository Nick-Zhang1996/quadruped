from common import *
from math import degrees,radians
from Controller import *
import matplotlib.pyplot as plt
class StepPlanner(PrintObject):
    def __init__(self, event):
        self.sim = event.sim
        event.step_planner = self
        self.quadruped = event.quadruped
        self.event = event
        self.horizon = 10
        self.dt = 0.02
        self.current_plan = None
        self.next_plan_name = None
        self.current_plan_t0 = 0
        self.name_to_plan = {}
        self.getTime = self.dummyGetTime
        self.joint_torque = [0,0,0,0]
        self.ground_reaction_force = [0,0,0,0]
        # WBC
        self.controller = Controller(event, self.dt, self.horizon)
        self.controller.buildModel()

        self.newPlan('stand',self.getPlanStand)
        # jump = squat + jump + in_air + landing
        self.newPlan('jumpSeq',self.getPlanSquat)
        self.newPlan('jump',self.getPlanJump)
        self.newPlan('in_air',self.getPlanInAir)
    def exit(self):
        self.controller.exit()

    def dummyGetTime(self):
        print_error("set this to a function that returns current time")

    def newPlan(self, name, handle):
        self.name_to_plan[name] = handle

    # start executing a plan
    # name: stand, squat, jump, land
    def setPlan(self,name):
        self.current_plan_t0 = self.getTime()
        self.current_plan = self.name_to_plan[name]
        self.getPlan(True)

    def processJoystick(self):
        if self.event.joystick.button['E'] == 1:
            if (self.current_plan == self.getPlanStand):
                self.print_info("switching to jump")
                self.setPlan('jumpSeq')

    # contact schedule: N*2 (bool)
    # target_state: N*n (float)
    # time_left: float
    def step(self):
        self.processJoystick()
        contact_schedule, target_state, target_angle, time_left = self.getPlan()
        # joint order  [front_knee_joint, front_shoulder_joint, rear_knee_joint, rear_shoulder_joint]
        # 0 means all legs pointing downward
        #target_angle = [radians(30),-radians(120),0,-radians(120)]
        self.controller.step(contact_schedule, target_state, target_angle)
        self.joint_torque = self.controller.getJointTorque()
        self.ground_reaction_force = self.controller.getGroundReactionForce()

        if (not time_left):
            self.updatePlan()

    def updatePlan(self):
        if (self.next_plan_name is None):
            self.print_error("next plan is None")
        else:
            self.setPlan(self.next_plan_name)
            self.next_plan_name = None

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
        target_angle = np.zeros(4)
        elapsed_time = self.getTime() - self.current_plan_t0
        if (first_call):
            self.jump_target_x = self.quadruped.base_link.body.position[0] + 5
            self.jump_original_y = self.quadruped.base_link.body.position[1]
            #self.jump_transition_duration = 1
            self.jump_transition_duration = 0.1
            self.jump_target_y = 160 - 30
            self.jump_y_vel = (self.jump_target_y - self.jump_original_y)/self.jump_transition_duration
        contact_schedule = [[True,True] for i in range(self.horizon)]

        if (elapsed_time > self.jump_transition_duration):
            ref_state = np.array([0, self.jump_target_x, self.jump_target_y, 0, 0, 0])
            target_state = np.repeat(ref_state.reshape(1,-1),self.horizon,axis=0)
        else:
            target_state = []
            for i in range(self.horizon):
                current_step_time = elapsed_time + self.dt * i
                if (current_step_time < self.jump_transition_duration):
                    y = map(current_step_time, 0, self.jump_transition_duration, self.jump_original_y, self.jump_target_y)
                    ref_state = np.array([0, self.jump_target_x, y, 0, 0, self.jump_y_vel])
                else:
                    ref_state = np.array([0, self.jump_target_x, self.jump_target_y, 0, 0, 0])
                target_state.append(ref_state)
            target_state = np.array(target_state)

        state_error = self.quadruped.getState()-target_state[0]
        state_error_norm = np.linalg.norm(state_error)
        #self.print_info("t: %.2f, state_error: %.3f, dy %.2f, dvy %.2f "%(elapsed_time, state_error_norm, state_error[2], state_error[5]))
        '''
        if (self.event.joystick.button['S']):
            plt.plot(self.quadruped.getState()[2],'o')
            plt.plot(target_state[:,2])
            plt.show()
            #breakpoint()
        '''

        if (state_error_norm < 0.5):
            self.print_info("squat finished")
            self.next_plan_name = 'jump'
            return contact_schedule, target_state,target_angle, 0
        else:
            return contact_schedule, target_state,target_angle,1

    # getPlan for jump
    # TODO handle single foot contact with WBC
    def getPlanJump(self, first_call):
        target_angle = np.zeros(4)
        h = 0.5*100
        g = self.event.sim.g
        v_takeoff_y = (2*h*g)**0.5
        elapsed_time = self.getTime() - self.current_plan_t0
        self.controller.velocityGain()
        #ref_state = np.array([0, 0, 0, 0, 200, 300])
        ref_state = np.array([0, 0, 0, 0, 300, 350])
        target_state = ref_state.reshape((1,6)).repeat(self.horizon,axis=0)
        contact_schedule = [[True,True] for i in range(self.horizon)]
        #if (np.abs(self.quadruped.getFrontKneeAngle()) < radians(20) or np.abs(self.quadruped.getRearKneeAngle()) < radians(20)):
        if (np.linalg.norm(self.quadruped.front_ground_reaction) < 1000 or np.linalg.norm(self.quadruped.rear_ground_reaction) < 1000):
            self.print_info("jump finished")
            self.print_info("target velocity: x: %.1f y: %.1f"%(ref_state[4], ref_state[5]))
            v = self.quadruped.base_link.body.velocity
            self.print_info("achieved velocity: x: %.1f y: %.1f"%(v[0],v[1]))
            self.next_plan_name = 'in_air'
            return contact_schedule, target_state,target_angle,0
        else:
            return contact_schedule, target_state,target_angle,1

    def getPlanInAir(self, first_call):
        if (first_call):
            #self.in_air_target_angle = np.array([radians(70), radians(-120),radians(70), radians(-120)])
            self.in_air_target_angle = np.array([radians(110), radians(-140),radians(110), radians(-140)])
            self.in_air_contact_count = 0
        target_angle = self.in_air_target_angle
        contact_schedule = [[False,False] for i in range(self.horizon)]
        target_state = None

        # if foot contact, switch to landing controller
        if (np.linalg.norm(self.quadruped.front_ground_reaction) > 100 and np.linalg.norm(self.quadruped.rear_ground_reaction) > 100):
            self.in_air_contact_count += 1

        if (self.in_air_contact_count > 3):
            self.print_info("contact detected, switching to standing")
            self.next_plan_name = 'stand'
            self.controller.normalGain()
            return contact_schedule, target_state, target_angle,0
        else:
            return contact_schedule, target_state, target_angle,1

    # getPlan for standing
    def getPlanStand(self, first_call):
        target_angle = np.zeros(4)
        elapsed_time = self.getTime() - self.current_plan_t0
        if (first_call):
            #self.stand_target_x = self.quadruped.base_link.body.position[0]
            # set target CG location to be center of foot contact
            xf = self.quadruped.front_foot_pos()[0]
            xr = self.quadruped.rear_foot_pos()[0]
            self.stand_target_x = 0.5*(xf+xr)

            self.stand_original_y = self.quadruped.base_link.body.position[1]
            #self.stand_transition_duration = 1
            self.stand_transition_duration = 0.1
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

        return contact_schedule, target_state,target_angle,1

    
        


