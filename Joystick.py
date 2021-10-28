import pygame

class Joystick:
    def __init__(self):
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        self.joystick = joysticks[0]
        self.axes = self.joystick.get_numaxes()
        # LT:left trigger
        # LH: left horizontal
        self.vals = {'LT':-1,'RT':-1,'LH':0,'LV':0,'RH':0,'RV':0}

    def checkJoystickEvent(self,event):
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN
        # JOYBUTTONUP JOYHATMOTION
        '''
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
        '''
        if event.type == pygame.JOYAXISMOTION:
            joystick = self.joystick
            self.vals['LH'] = joystick.get_axis(0)
            self.vals['LV'] = joystick.get_axis(1)
            self.vals['LT'] = joystick.get_axis(2)
            self.vals['RH'] = joystick.get_axis(3)
            self.vals['RV'] = joystick.get_axis(4)
            self.vals['RT'] = joystick.get_axis(5)


if __name__=="__main__":
    pygame.init()
    pygame.display.set_caption("Quadruped Simulation")
    main = Joystick()
    while True:
        for event in pygame.event.get():
            main.checkJoystickEvent(event)
            print(main.vals)

