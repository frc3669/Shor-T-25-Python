import wpilib.simulation as sim
from wpilib import RobotController, DriverStation

from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, meters, radians
from wpimath.geometry import Pose2d

from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged

from subsystems.swerve import Swerve
import constants

import typing

if typing.TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller
        self.physics_controller.field.setRobotPose(Pose2d(constants.startingPosition.real, constants.startingPosition.imag, 0))
        self.motor_sims = [sim.DCMotorSim(DCMotor.krakenX60FOC(1), 1, 0.0001) for x in range(8)]
    
    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # If the driver station is enabled, then feed enable for phoenix devices
        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)
        i = 0
        for module in Swerve.modules:
            # simulate drive motor
            module.drive_motor.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
            self.motor_sims[i].setInputVoltage(module.drive_motor.sim_state.motor_voltage)
            self.motor_sims[i].update(tm_diff)
            module.drive_motor.sim_state.set_raw_rotor_position(radiansToRotations(self.motor_sims[i].getAngularPosition()))
            module.drive_motor.sim_state.set_rotor_velocity(radiansToRotations(self.motor_sims[i].getAngularVelocity()))
            # simulate steering motor
            module.steering_motor.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
            self.motor_sims[i+1].setInputVoltage(module.steering_motor.sim_state.motor_voltage)
            self.motor_sims[i+1].update(tm_diff)
            module.steering_motor.sim_state.set_raw_rotor_position(radiansToRotations(self.motor_sims[i+1].getAngularPosition()))
            module.steering_motor.sim_state.set_rotor_velocity(radiansToRotations(self.motor_sims[i+1].getAngularVelocity()))
            # simulate cancoder
            module.angle_encoder.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
            module.angle_encoder.sim_state.set_raw_position(radiansToRotations(self.motor_sims[i+1].getAngularPosition()/6.12))
            module.angle_encoder.sim_state.set_velocity(radiansToRotations(self.motor_sims[i+1].getAngularVelocity()/6.12))
            i += 2
        Swerve.gyro.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
        Swerve.gyro.sim_state.add_yaw(Swerve.getRotationRate()*135 * tm_diff)
        posititonx = Swerve.position.real
        posititony = Swerve.position.imag
        heading = Swerve.heading
        self.physics_controller.field.setRobotPose(Pose2d(posititonx, posititony, heading))