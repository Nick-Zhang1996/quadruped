from common import *
import pygame

class Joystick(PrintObject):
    def __init__(self):
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        self.joystick = joysticks[0]
        self.axes = self.joystick.get_numaxes()
        # LT:left trigger
        # LH: left horizontal
        self.vals = {'LT':-1,'RT':-1,'LH':0,'LV':0,'RH':0,'RV':0}
        self.button_id_to_name = ['S','E','W','N','LB','RB','BACK','START']
        self.button = {'S':0, 'E':0, 'W':0, 'N':0, 'LB':0, 'RB':0, 'BACK':0, 'START':0}

    def checkJoystickEvent(self,event):
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN
        # JOYBUTTONUP JOYHATMOTION
        '''
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
        '''
        # reset button every time
        if event.type == pygame.JOYAXISMOTION:
            joystick = self.joystick
            self.vals['LH'] = joystick.get_axis(0)
            self.vals['LV'] = joystick.get_axis(1)
            self.vals['LT'] = joystick.get_axis(2)
            self.vals['RH'] = joystick.get_axis(3)
            self.vals['RV'] = joystick.get_axis(4)
            self.vals['RT'] = joystick.get_axis(5)
            #self.print_info("joystick updated",self.vals)
        elif event.type == pygame.JOYBUTTONDOWN:
            self.button[self.button_id_to_name[event.button]] = 1
        elif event.type == pygame.JOYBUTTONUP:
            self.button[self.button_id_to_name[event.button]] = 0


if __name__=="__main__":
    pygame.init()
    pygame.display.set_caption("Quadruped Simulation")
    main = Joystick()
    while True:
        for event in pygame.event.get():
            main.checkJoystickEvent(event)
            #print(main.vals)
            print(main.button)

