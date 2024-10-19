import cmath, commands2
from wpilib import Timer, SmartDashboard, Joystick
from phoenix6 import hardware
from utils.trajectory import Trajectory
from subsystems.swerveModule import SwerveModule, constants, mathFunctions
from typing import final

@final
class Swerve(commands2.Subsystem):
    gyro = hardware.Pigeon2(1, "CTREdevices")
    auto_timer = Timer()
    modules = []
    slew_velocity = complex()
    slew_angular_velocity = 0
    position = constants.startingPosition
    heading = 0
    trajectory = None
    sample_index = 0
    
    @staticmethod
    def simulationPeriodic():
        # This method will be called once per scheduler run during simulation
        pass

    @staticmethod
    def driveTeleop(controller: Joystick):
        velocity = 0
        angular_velocity = 0
        if controller.getName() == "Controller (Xbox One For Windows)":
            velocity = complex(-controller.getRawAxis(1), -controller.getRawAxis(0))
            angular_velocity = -controller.getRawAxis(4)
        elif controller.getName() == "Radiomaster Boxer Joystick":
            velocity = complex(controller.getRawAxis(0), controller.getRawAxis(1))
            angular_velocity = controller.getRawAxis(2)
        elif controller.getName() == "Keyboard 0":
            velocity = complex(controller.getRawAxis(0), -controller.getRawAxis(1))
            angular_velocity = -controller.getRawAxis(2)

        if controller.getRawButton(4):
            Swerve.gyro.set_yaw(0)
        
        SmartDashboard.putNumber('velocity_x', velocity.real)
        SmartDashboard.putNumber('velocity_y', velocity.imag)
        # apply smooth deadband
        dB = 0.03
        if abs(velocity) > dB:
            velocity *= (1 - dB/abs(velocity))/(1 - dB)
        else:
            velocity = complex(0, 0)
        if abs(angular_velocity) > dB:
            angular_velocity *= (1 - dB/abs(angular_velocity))/(1 - dB)
        else:
            angular_velocity = 0
        # scale the velocities to meters per second
        velocity *= constants.max_m_per_sec
        angular_velocity *= constants.max_m_per_sec
        # find the robot oriented velocity
        Swerve.heading = Swerve.gyro.get_yaw().value_as_double*cmath.tau/360
        SmartDashboard.putNumber('heading', Swerve.heading)
        robot_velocity = velocity * cmath.rect(1, -Swerve.heading)
        # find the fastest module speed
        highest = constants.max_m_per_sec
        for module in Swerve.modules:
            module_speed = abs(module.find_module_vector(robot_velocity, angular_velocity))
            if module_speed > highest:
                highest = module_speed
        # scale the velocities
        velocity *= constants.max_m_per_sec/highest
        angular_velocity *= constants.max_m_per_sec/highest
        robot_velocity *= constants.max_m_per_sec/highest
        # find the error between the command and the current velocities
        velocity_error = velocity - Swerve.slew_velocity
        angular_velocity_error = angular_velocity - Swerve.slew_angular_velocity
        SmartDashboard.putNumber('vel_err_x', velocity_error.real)
        SmartDashboard.putNumber('vel_err_y', velocity_error.imag)
        # find the robot oriented velocity error
        robot_velocity_error = velocity_error * cmath.rect(1, -Swerve.heading)
        robot_slew_velocity = Swerve.slew_velocity * cmath.rect(1, -Swerve.heading)
        # find the max acceleration overshoot
        highest = 1
        for module in Swerve.modules:
            module_overshoot = module.get_accel_overshoot(robot_slew_velocity, Swerve.slew_angular_velocity, robot_velocity_error, angular_velocity_error)
            if module_overshoot > highest:
                highest = module_overshoot
        # find velocity increments
        velocity_increment = velocity_error/highest
        angular_velocity_increment = angular_velocity_error/highest
        SmartDashboard.putNumber('vel_inc_x', velocity_increment.real)
        SmartDashboard.putNumber('vel_inc_y', velocity_increment.imag)
        # increment velocity
        if abs(velocity_error) > constants.max_m_per_sec_per_cycle:
            Swerve.slew_velocity += velocity_increment
        else:
            Swerve.slew_velocity = velocity
        if abs(angular_velocity_error) > constants.max_m_per_sec_per_cycle:
            Swerve.slew_angular_velocity += angular_velocity_increment
        else:
            Swerve.slew_angular_velocity = angular_velocity
        # update the robot oriented slew velocity
        robot_slew_velocity = Swerve.slew_velocity * cmath.rect(1, -Swerve.heading)
        # find acceleration feedforward
        robot_accel = robot_velocity_error*2
        angular_accel = angular_velocity_error*2
        # drive the modules
        SmartDashboard.putNumber('Swerve.slew_velocity_x', robot_slew_velocity.real)
        SmartDashboard.putNumber('Swerve.slew_velocity_y', robot_slew_velocity.imag)
        for module in Swerve.modules:
            module.set_velocity(robot_slew_velocity, Swerve.slew_angular_velocity, robot_accel, angular_accel)
        SmartDashboard.putNumber('timestamp', Timer.getFPGATimestamp())
        Swerve.calculateOdometry()
    
    @staticmethod
    def setTrajectory(trajectory: Trajectory):
        Swerve.trajectory = trajectory
        Swerve.sample_index = 0
        Swerve.auto_timer.restart()

    @staticmethod  
    def moveToNextSample():
        Swerve.heading = Swerve.gyro.get_yaw().value_as_double*cmath.tau/360
        # find the latest sample index
        while Swerve.auto_timer.hasElapsed(Swerve.trajectory.get_sample(Swerve.sample_index).timestamp) and Swerve.sample_index < Swerve.trajectory.get_sample_count() - 1:
            Swerve.sample_index += 1
        if Swerve.sample_index < Swerve.trajectory.get_sample_count():
            Swerve.calculateOdometry()
            current_sample = Swerve.trajectory.get_sample(Swerve.sample_index)
            # calculate the proportional response
            position_error = current_sample.position - Swerve.position
            heading_error = current_sample.heading - Swerve.heading
            heading_error = mathFunctions.get_wrapped(heading_error)
            velocity = current_sample.velocity + constants.swerve_position_P * position_error
            angular_velocity = current_sample.angular_velocity + constants.swerve_heading_P * heading_error
            velocity *= cmath.rect(1, -Swerve.heading)
            for module in Swerve.modules:
                module.set_velocity(velocity, angular_velocity, current_sample.acceleration, current_sample.angular_acceleration)
        else:
            for module in Swerve.modules:
                module.set_velocity()

    @staticmethod
    def followTrajectory(trajectory: Trajectory) -> commands2.Command:
        return commands2.FunctionalCommand (
            lambda: Swerve.setTrajectory(trajectory), 
            lambda: Swerve.moveToNextSample(),
            lambda x : Swerve.doNothing(),
            lambda: Swerve.sample_index == Swerve.trajectory.get_sample_count() - 1,
            Swerve)
    
    @staticmethod
    def getTrajectoryRemainingTime():
        return Swerve.trajectory.getEndTime() - Swerve.auto_timer.get()

    @staticmethod
    def calculateOdometry():
        position_change = complex(0, 0)
        for module in Swerve.modules:
            position_change += module.getPositionChange()
        Swerve.position += position_change * cmath.rect(0.25, Swerve.heading)

    @staticmethod
    def add_module(module: SwerveModule):
        Swerve.modules.append(module)

    @staticmethod
    def resetPosition(new_position: complex = complex()):
        for module in Swerve.modules:
            module.reset_encoders()
        Swerve.position = new_position
    
    @staticmethod
    def getRotationRate():
        angularRate = 0
        for module in Swerve.modules:
            angularRate += module.getRotationContribution()
        SmartDashboard.putNumber('rotation_rate', angularRate/4)
        return angularRate / 4
    
    def doNothing():
        pass