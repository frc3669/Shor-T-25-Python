import wpilib
from subsystems.swerveDrive import SwerveDrive
from subsystems.swerveModule import SwerveModule

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.controller = wpilib.Joystick(0)
        self.swerve = SwerveDrive()
        self.swerve.add_module(SwerveModule(1, 1, 1))
        self.swerve.add_module(SwerveModule(1, -1, 1))
        self.swerve.add_module(SwerveModule(1, -1, -1))
        self.swerve.add_module(SwerveModule(1, 1, -1))
    
    def teleopPeriodic(self):
        self.swerve.set_velocity(self.controller.getRawAxis(0), self.controller.getRawAxis(1), self.controller.getRawAxis(2))