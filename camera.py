import numpy as np
import pyrr


class Camera:
    def __init__(self, position):
        self.forwards = None
        self.right = None
        self.up = None
        self.position = np.array(position, dtype=np.float32)
        self.theta = 0
        self.phi = 0
        self.recalculate_camera_vectors()

    def recalculate_camera_vectors(self):
        self.forwards = np.array(
            [
                np.cos(np.deg2rad(self.theta)) * np.cos(np.deg2rad(self.phi)),
                np.sin(np.deg2rad(self.theta)) * np.cos(np.deg2rad(self.phi)),
                np.sin(np.deg2rad(self.phi))
            ], dtype=np.float32
        )

        self.right = pyrr.vector.normalize(
            pyrr.vector3.cross(
                self.forwards,
                np.array([0, 0, 1], dtype=np.float32)
            )
        )

        self.up = pyrr.vector.normalize(
            pyrr.vector3.cross(
                self.right,
                self.forwards
            )
        )