import wpilib, commands2
from subsystems.swerve import Swerve, SwerveModule, Trajectory

class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        Swerve.add_module(SwerveModule(1, 1, 1))
        Swerve.add_module(SwerveModule(2, -1, 1))
        Swerve.add_module(SwerveModule(3, -1, -1))
        Swerve.add_module(SwerveModule(4, 1, -1))
        self.trajectory1 = Trajectory(wpilib.getDeployDirectory() + "/test.traj")
        self.controller = wpilib.Joystick(0)
    
    def teleopPeriodic(self):
        Swerve.driveTeleop(self.controller)
    
    def autonomousInit(self) -> None:
        Swerve.followTrajectory(self.trajectory1).schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous"""

if __name__ == "__main__":
    wpilib.run(Robot)