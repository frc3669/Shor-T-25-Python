import cmath, commands2
from phoenix6 import hardware, controls, configs, StatusCode
import constants
from utils import mathFunctions as mf
from wpilib import SmartDashboard

class SwerveModule(commands2.Subsystem):
    # create a swerve module at the given position relative to the center of the robot
    def __init__(self, moduleID: int, module_position_x: float, module_position_y: float):
        super().__init__()
        self.drive_motor = hardware.TalonFX(10 + moduleID, "CTREdevices")
        self.steering_motor = hardware.TalonFX(20 + moduleID, "CTREdevices")
        self.angle_encoder = hardware.CANcoder(30 + moduleID, "CTREdevices")
        self.velocity_ctrl = controls.VelocityTorqueCurrentFOC(0)
        self.torque_ctrl = controls.TorqueCurrentFOC(0)
        cfg = configs.TalonFXConfiguration()
        cfg.slot0.k_p = 5
        cfg.slot0.k_s = 3
        cfg.torque_current.peak_forward_torque_current = constants.max_current
        cfg.torque_current.peak_reverse_torque_current = -constants.max_current
        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.drive_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        # calculate the turn vector
        self.turn_vector = complex(module_position_x, module_position_y) * complex(0, 1)
        if abs(self.turn_vector) != 0:
            self.turn_vector /= abs(self.turn_vector)
        self.motor_position_old = 0
        self.position_change = 0
        self.module_velocity = 0
        self.moduleID = moduleID

    def odometryCalc(self):
        self.angle = self.angle_encoder.get_absolute_position().value_as_double * cmath.tau
        SmartDashboard.putNumber("wheel_ang_" + str(self.moduleID), self.angle / cmath.tau)
        motor_position = self.drive_motor.get_position().value_as_double
        motor_position_change = motor_position - self.motor_position_old
        self.motor_position_old = motor_position
        self.position_change = cmath.rect(motor_position_change / constants.motor_turns_per_m, self.angle)
        self.module_velocity = cmath.rect(self.drive_motor.get_velocity().value_as_double / constants.motor_turns_per_m, self.angle)
    
    def set_velocity(self, robot_velocity: complex = complex(), angular_velocity: float = 0, robot_accel: complex = complex(), angular_accel: float = 0):
        velocity = self.find_module_vector(robot_velocity, angular_velocity)
        accel_current = self.find_module_vector(robot_accel, angular_accel)*constants.current_to_accel_ratio
        wheel_speed = abs(velocity)
        self.angle = self.angle_encoder.get_absolute_position().value_as_double*cmath.tau
        error = mf.get_wrapped(cmath.phase(velocity) - self.angle)
        if wheel_speed < 0.008:
            error = 0
        if abs(error) > cmath.pi/2:
            error = mf.get_wrapped(error + cmath.pi)
            wheel_speed *= -1
        self.steering_motor.set_control(controls.DutyCycleOut(error/cmath.pi))
        # use torque/velocity to set the drive motor velocity
        wheel_accel_current = mf.get_projection_size(accel_current, cmath.rect(1, self.angle))
        self.drive_motor.set_control(self.velocity_ctrl.with_velocity(wheel_speed*constants.motor_turns_per_m).with_feed_forward(wheel_accel_current))
        SmartDashboard.putNumber("wheel_vel_" + str(self.moduleID), self.drive_motor.get_velocity().value_as_double)
        self.odometryCalc()

    def accelTest(self, torque_current: float = 0):
        self.angle = self.angle_encoder.get_absolute_position().value_as_double*cmath.tau
        error = mf.get_wrapped(cmath.pi/2 - self.angle)
        self.steering_motor.set_control(controls.DutyCycleOut(error/cmath.pi))
        # use torque/velocity to set the drive motor velocity
        self.drive_motor.set_control(controls.TorqueCurrentFOC(torque_current))
        SmartDashboard.putNumber("wheel_vel_" + str(self.moduleID), self.drive_motor.get_velocity().value_as_double)
        
    def find_module_vector(self, robot_vector, angular_rate):
        return robot_vector + self.turn_vector*angular_rate
    
    def get_accel_overshoot(self, robot_vel, angular_vel, robot_vel_increment, angular_vel_increment):
        velocity = self.find_module_vector(robot_vel, angular_vel)
        vel_increment = self.find_module_vector(robot_vel_increment, angular_vel_increment)
        accel_overshoot = 1
        if abs(vel_increment) > constants.max_m_per_sec_per_cycle:
            accel_overshoot = abs(vel_increment) / constants.max_m_per_sec_per_cycle
        wheel_current = mf.get_projection_size(vel_increment/constants.code_cycle_time*constants.current_to_accel_ratio, velocity) + constants.feedforward_current
        wheel_accel_overshoot = abs(wheel_current) / (constants.max_current-constants.current_headroom)
        if wheel_accel_overshoot > accel_overshoot:
            accel_overshoot = wheel_accel_overshoot
        return accel_overshoot
    
    def getPositionChange(self):
        return self.position_change
    
    def getRotationContribution(self):
        rot_contr = mf.get_projection_size(self.module_velocity, self.turn_vector)
        SmartDashboard.putNumber("rot_contr_" + str(self.moduleID), rot_contr)
        return rot_contr

    def reset_encoders(self):
        self.motor_position_old = 0
        self.drive_motor.set_position(0)