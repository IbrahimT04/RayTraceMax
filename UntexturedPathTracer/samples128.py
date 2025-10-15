import numpy as np
from numpy.ma.extras import average


def generate_random_coordinates_flat(n_points=128, seed=1):
    """
    Generates n_points coordinate pairs (x, y) where each coordinate is in [0, 1].
    Returns a flattened float32 array of length 2 * n_points.
    """
    rng = np.random.default_rng(seed)
    coords = rng.random((n_points, 2), dtype=np.float32)
    return coords.ravel()

# Example
samples128 = generate_random_coordinates_flat()
# print(samples128.shape)  # (256,)
# print(samples128.dtype)  # float32
# print(samples128)
# print(average(samples128))
