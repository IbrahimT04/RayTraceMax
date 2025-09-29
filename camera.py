import numpy as np
import pyrr

class Camera:
    def __init__(self, position):
        self.forwards = None
        self.right = None
        self.up = None
        self.position = np.array(position, dtype=np.float32)

        self.theta = 0.0
        self.phi = 0.0

        self.move_speed = 6.0
        self.sprint_multiplier = 2.0
        self.mouse_sensitivity = 0.15

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
            pyrr.vector3.cross(self.forwards, np.array([0, 0, 1], dtype=np.float32))
        )
        self.up = pyrr.vector.normalize(pyrr.vector3.cross(self.right, self.forwards))

    def move(self, forward=0.0, right=0.0, up=0.0):
        self.position = self.position + self.forwards * np.float32(forward)
        self.position = self.position + self.right * np.float32(right)
        self.position = self.position + self.up * np.float32(up)

    def rotate(self, delta_yaw, delta_pitch):
        self.theta += float(delta_yaw)
        self.phi += float(delta_pitch)
        self.phi = float(np.clip(self.phi, -89.9, 89.9))
        self.recalculate_camera_vectors()

    def apply_movement(self, forward_dir, right_dir, up_dir, dt, sprint=False):
        speed = self.move_speed * (self.sprint_multiplier if sprint else 1.0)
        self.move(forward=forward_dir * speed * dt,
                  right=right_dir * speed * dt,
                  up=up_dir * speed * dt)

    def apply_rotation_from_mouse(self, dx, dy):
        yaw = dx * -1.0 * self.mouse_sensitivity
        pitch = dy * -1.0 * self.mouse_sensitivity
        self.rotate(yaw, pitch)

    def get_pose(self):
        return {
            "position": self.position.copy(),
            "forwards": self.forwards.copy(),
            "right": self.right.copy(),
            "up": self.up.copy(),
            "theta": self.theta,
            "phi": self.phi
        }
