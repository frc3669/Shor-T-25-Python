from subsystems.swerveModule import SwerveModule
import constants
import cmath
from wpilib import Timer
from phoenix6 import hardware

class SwerveDrive:
    def __init__(self):
        self.gyro = hardware.Pigeon2(1, "CTREdevices")
        self.auto_timer = Timer()
        self.modules = []
        self.sample_index = 0
        self.slew_velocity = complex()
        self.slew_angular_velocity = 0
        self.position = complex()
        self.heading = 0

    def set_velocity(self, x_velocity: float, y_velocity: float, angular_velocity: float):
        velocity = complex(x_velocity, y_velocity)
        # apply smooth deadband
        dB = 0.03
        velocity = 0
        if abs(velocity) > dB:
            velocity *= (1 - dB/abs(velocity))/(1 - dB)
        angular_velocity = 0
        if abs(angular_velocity) > dB:
            angular_velocity *= (1 - dB/abs(angular_velocity))/(1 - dB)
        # scale the velocities to meters per second
        velocity *= constants.max_m_per_sec
        angular_velocity *= constants.max_m_per_sec
        # find the robot oriented velocity
        self.heading = self.gyro.get_yaw().value_as_double*cmath.tau/360
        robot_velocity = velocity * cmath.rect(1, -self.heading)
        # find the fastest module speed
        highest = constants.max_m_per_sec
        for module in self.modules:
            module_speed = abs(module.find_module_vector(robot_velocity, angular_velocity))
            if module_speed > highest:
                highest = module_speed
        # scale the velocities
        velocity *= constants.max_m_per_sec/highest
        angular_velocity *= constants.max_m_per_sec/highest
        robot_velocity *= constants.max_m_per_sec/highest
        # find the error between the command and the current velocities
        velocity_error = velocity - self.slew_velocity
        angular_velocity_error = angular_velocity = self.slew_angular_velocity
        # find the robot oriented velocity error
        robot_velocity_error = velocity_error * cmath.rect(1, -self.heading)
        robot_slew_velocity = self.slew_velocity * cmath.rect(1, -self.heading)
        # find the max acceleration overshoot
        highest = 1
        for module in self.modules:
            module_overshoot = module.get_accel_overshoot(robot_slew_velocity, self.slew_angular_velocity, robot_velocity_error, angular_velocity_error)
            if module_overshoot > highest:
                highest = module_overshoot
        # find velocity increments
        velocity_increment = velocity_error/highest
        angular_velocity_increment = angular_velocity_error/highest
        # increment velocity
        if abs(velocity_error) > constants.max_m_per_sec_per_cycle:
            self.slew_velocity += velocity_increment
        else:
            self.slew_velocity = velocity
        if abs(angular_velocity_error) > constants.max_m_per_sec_per_cycle:
            self.slew_angular_velocity += angular_velocity_increment
        else:
            self.slew_angular_velocity = angular_velocity
        # update the robot oriented slew velocity
        robot_slew_velocity = self.slew_velocity * cmath.rect(1, -self.heading)
        # find acceleration feedforward
        robot_accel = robot_velocity_error*2
        angular_accel = angular_velocity_error*2
        # drive the modules
        for module in self.modules:
            module.set_velocity(robot_slew_velocity, self.slew_angular_velocity, robot_accel, angular_accel)
    
    def add_module(self, module: SwerveModule):
        self.modules.append(module)