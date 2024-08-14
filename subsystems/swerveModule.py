import constants
import mathFunctions
import cmath
from phoenix6 import hardware, controls, configs, StatusCode

class SwerveModule:
    # create a swerve module at the given position relative to the center of the robot
    def __init__(self, moduleID: int, module_position_x: float, module_position_y: float):
        self.drive_motor = hardware.TalonFX(10 + moduleID, "CTREdevices")
        self.steering_motor = hardware.TalonFX(20 + moduleID, "CTREdevices")
        self.angle_encoder = hardware.CANcoder(30 + moduleID, "CTREdevices")
        self.velocity_ctrl = controls.VelocityTorqueCurrentFOC(0)
        cfg = configs.TalonFXConfiguration()
        cfg.slot0.k_p = 5
        cfg.slot0.k_s = 3
        cfg.torque_current.peak_forward_torque_current = 28
        cfg.torque_current.peak_reverse_torque_current = -28
        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.drive_motor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        # calculate the turn vector
        self.turn_vector = complex(module_position_x, module_position_y) * 1j
    
    def set_velocity(self, robot_velocity, angular_velocity, robot_accel, angular_accel):
        velocity = self.find_module_vector(robot_velocity, angular_velocity)
        accel_current = self.find_module_vector(robot_accel, angular_accel)*constants.current_to_accel_ratio
        wheel_speed = abs(velocity)
        self.angle = self.angle_encoder.get_absolute_position().value_as_double*cmath.tau
        error = mathFunctions.get_wrapped(cmath.phase(velocity) - self.angle)
        if wheel_speed < 0.008:
            error = 0
        if abs(error) > cmath.pi/2:
            error = mathFunctions.get_wrapped(error + cmath.pi)
            wheel_speed *= -1
        self.steering_motor.set_control(controls.DutyCycleOut(error/cmath.pi))
        # use torque/velocity to set the drive motor velocity
        wheel_accel_current = mathFunctions.get_projection_magnitude(accel_current, cmath.rect(1, self.angle))
        self.drive_motor.set_control(self.velocity_ctrl.with_velocity(wheel_speed*constants.motor_turns_per_m).with_feed_forward(wheel_accel_current))

        
    def find_module_vector(self, robot_vector, angular_rate):
        return robot_vector + self.turn_vector * angular_rate
    
    def get_accel_overshoot(self, robot_vel, angular_vel, robot_vel_increment, angular_vel_increment):
        velocity = self.find_module_vector(robot_vel, angular_vel)
        vel_increment = self.find_module_vector(robot_vel_increment, angular_vel_increment)
        accel_overshoot = 1
        if abs(vel_increment) > constants.max_m_per_sec_per_cycle:
            accel_overshoot = abs(vel_increment) / constants.max_m_per_sec_per_cycle
        wheel_current = mathFunctions.get_projection_magnitude(vel_increment/constants.code_cycle_time*constants.current_to_accel_ratio, velocity) + constants.feedforward_current
        wheel_accel_overshoot = abs(wheel_current) / (constants.max_current-constants.current_headroom)
        if wheel_accel_overshoot > accel_overshoot:
            accel_overshoot = wheel_accel_overshoot
        return accel_overshoot