import cmath

def get_wrapped(x):
    while x > cmath.pi:
        x -= cmath.tau
    while x < -cmath.pi:
        x += cmath.tau
    return x

def get_projection_size(a, b):
    """ get the size of the projection of a onto b """
    # make b a unit vector
    if abs(b) != 0:
        b /= abs(b)
    return a.real*b.real + a.imag*b.imag

def get_projection(a, b):
    """get the projection of a onto b"""
    if abs(b) == 0: return 0
    b_normalized = b / abs(b)
    return (a * b_normalized.conjugate()).real * b_normalized

def get_perpendicular(a, b):
    """get the perpendicular portion of a with respect to b"""
    if abs(b) == 0: return 0
    b_normalized = b / abs(b)
    return (a * b_normalized.conjugate()).imag * (b_normalized * 1j)

def get_linear_accel(torque_current: float):
    return torque_current * 2.35 - 15.75

def get_linear_torque_current(acceleration: float):
    return (acceleration + 15.75) / 2.35

def get_angular_accel(torque_current: float):
    return torque_current * 3.433 - 23.87

def get_angular_torque_current(acceleration: float):
    return (acceleration + 23.87) / 3.433