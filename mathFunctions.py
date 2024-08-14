import cmath

def get_wrapped(x):
    while x > cmath.pi:
        x -= cmath.tau
    while x < -cmath.pi:
        x += cmath.tau
    return x

# get the projection of a onto b
def get_projection_magnitude(a, b):
    # make b a unit vector
    if abs(b) != 0:
        b /= abs(b)
    return a.real*b.real + a.imag*b.imag