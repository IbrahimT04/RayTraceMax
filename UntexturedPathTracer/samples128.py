import numpy as np

def sample_points_in_unit_circle(n_points=128, seed=0):
    rng = np.random.default_rng(seed)
    # Uniformly distributed points inside the unit circle
    r = np.sqrt(rng.random(n_points, dtype=np.float32))
    theta = 2 * np.pi * rng.random(n_points, dtype=np.float32)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    # Combine and flatten to 1D array of length 256 (128 pairs)
    return np.column_stack((x, y)).flatten().astype(np.float32)

samples128 = sample_points_in_unit_circle()
"""print(samples128.shape)   # (256,)
print(samples128.dtype)   # float32
print(samples128)
"""