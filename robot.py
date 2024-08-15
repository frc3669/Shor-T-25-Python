import wpilib
from subsystems.swerveDrive import SwerveDrive
from subsystems.swerveModule import SwerveModule

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.controller = wpilib.Joystick(0)
        self.swerve = SwerveDrive()
        self.swerve.add_module(SwerveModule(1, 1, 1))
        self.swerve.add_module(SwerveModule(2, -1, 1))
        self.swerve.add_module(SwerveModule(3, -1, -1))
        self.swerve.add_module(SwerveModule(4, 1, -1))
    
    def teleopPeriodic(self):
        # automatically choose the controller
        if self.controller.getName() == "Controller (Xbox One For Windows)":
            self.swerve.set_velocity(-self.controller.getRawAxis(1), -self.controller.getRawAxis(0), -self.controller.getRawAxis(4))
        if self.controller.getName() == "Radiomaster Boxer Joystick":
            self.swerve.set_velocity(self.controller.getRawAxis(0), self.controller.getRawAxis(1), self.controller.getRawAxis(2))

        if self.controller.getRawButton(4):
            self.swerve.gyro.set_yaw(0)