import cmath
from wpimath.units import seconds

code_cycle_time = 0.02
max_current = 25
feedforward_current = 4
current_headroom = 4
max_accel = 9
max_m_per_sec_per_cycle = max_accel * code_cycle_time
current_to_accel_ratio = 10
motor_turns_per_wheel_turn = 6.12
wheel_diameter_m = 0.09906
motor_turns_per_m = motor_turns_per_wheel_turn / (wheel_diameter_m*cmath.pi)
max_m_per_sec = 5

swerve_position_P = 0.04
swerve_heading_P = 2.5