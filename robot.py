import wpilib, commands2
import rev
from wpilib import DataLogManager
from subsystems.swerve import Swerve, SwerveModule, Trajectory

class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        Swerve.add_module(SwerveModule(1, 1, 1))
        Swerve.add_module(SwerveModule(2, -1, 1))
        Swerve.add_module(SwerveModule(3, -1, -1))
        Swerve.add_module(SwerveModule(4, 1, -1))
        self.controller = wpilib.Joystick(0)
        self.motor = rev.CANSparkMax(51,rev.CANSparkLowLevel.MotorType.kBrushless)
        self.pidController = self.motor.getPIDController()
        self.pidController.setP(0.1)


    
    def teleopInit(self):
        DataLogManager.start()

    def teleopPeriodic(self):
        # Swerve.driveTeleop(self.controller)
        position = -self.controller.getRawAxis(5)
        self.pidController.setReference(position+1,rev.CANSparkLowLevel.ControlType.kPosition)


    
    def teleopExit(self):
        DataLogManager.stop()
    
    def autonomousInit(self) -> None:
        commands2.cmd.sequence(
            Swerve.followTrajectory(Trajectory("Grab First Note.traj")),
            Swerve.followTrajectory(Trajectory("Shoot First Note.traj")),
            Swerve.followTrajectory(Trajectory("Grab Second Note.traj")),
            Swerve.followTrajectory(Trajectory("Grab Third Note.traj"))).schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous"""

if __name__ == "__main__":
    wpilib.run(Robot)