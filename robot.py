import wpilib
from subsystems.swerveDrive import SwerveDrive
from subsystems.swerveModule import SwerveModule
from trajectory import Trajectory

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.swerve = SwerveDrive()
        self.swerve.add_module(SwerveModule(1, 1, 1))
        self.swerve.add_module(SwerveModule(2, -1, 1))
        self.swerve.add_module(SwerveModule(3, -1, -1))
        self.swerve.add_module(SwerveModule(4, 1, -1))
        self.trajectory = Trajectory(wpilib.getDeployDirectory() + "/test.traj")
        self.controller = wpilib.Joystick(0)
    
    def teleopPeriodic(self):
        self.swerve.driveTeleop(self.controller)
    
    def autonomousInit(self) -> None:
        self.swerve.setTrajectory(self.trajectory)
        self.swerve.resetPosition()

    def autonomousPeriodic(self):
        self.swerve.followTrajectory()