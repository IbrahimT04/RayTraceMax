import numpy as np

from utilities import get_shaders


class GameObject:
    def __init__(self):
        self.vao = None
        self.ebo = None
        self.vbos = []
        self.shader_program = get_shaders('default')


class Quad(GameObject):
    vertices = np.array([-0.5, -0.5, 0.0, 1.0, 0.0, 0.0,
                         0.5, -0.5, 0.0, 0.0, 1.0, 0.0,
                         -0.5, 0.5, 0.0, 0.0, 0.0, 1.0,
                         0.5, 0.5, 0.0, 1.0, 1.0, 1.0], dtype=np.float32)

    def __init__(self):
        GameObject.__init__(self)
