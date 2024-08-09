import wpilib
import cmath

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.controller = wpilib.Joystick()
        self.velocity = complex(0, 0)

    def teleopPeriodic(self):
        self.velocity = complex(self.controller.getRawAxis(0), self.controller.getRawAxis(1))