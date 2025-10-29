import numpy as np

def generate_random_coordinates_flat(n_points=128, seed=1):

    rng = np.random.default_rng(seed)
    coords = rng.random((n_points, 2), dtype=np.float32)
    # for first accurate vector
    coords[0][0] = 1.0
    coords[0][1] = 0.0
    return coords.ravel()

samples128 = generate_random_coordinates_flat()
# print(samples128.shape)
# print(samples128.dtype)
# print(samples128)
# print(average(samples128))

samples256 = generate_random_coordinates_flat(n_points=256, seed=5)

samples512 = generate_random_coordinates_flat(n_points=512, seed=2)

samples1024 = generate_random_coordinates_flat(n_points=1024, seed=3)

samples2048 = generate_random_coordinates_flat(n_points=2048, seed=3)

samples4096 = generate_random_coordinates_flat(n_points=4096, seed=3)

"""
import math
def random_point_on_sphere(cx, cy, cz, r, u, v):
    phi = 2*math.pi*u
    z   = 2*v - 1
    rho = math.sqrt(max(0.0, 1 - z*z))  # guard tiny negatives from FP error
    x = rho*math.cos(phi)
    y = rho*math.sin(phi)
    return (cx + r*x, cy + r*y, cz + r*z)

"""

