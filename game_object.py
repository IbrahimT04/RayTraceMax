from OpenGL.GL import *
import numpy as np

from utilities import calc_shaders


class GameObject:
    def __init__(self, shader_program_name='default', num_buffers=1):
        # VAO
        self.vao = glGenVertexArrays(1)
        glBindVertexArray(self.vao)

        # VBO
        self.vbos = glGenBuffers(num_buffers)

        # EBO
        self.ebo = glGenBuffers(1)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)

        # Shader
        self.shader_program = calc_shaders(shader_program_name)

    def destroy(self):
        glBindVertexArray(0)
        glBindBuffer(0)
        self.vao = None
        self.vbos = None
        self.ebo = None


class Quad(GameObject):
    vertices = np.array([-0.5, -0.5,  0.0,   1.0, 0.0, 0.0,
                          0.5, -0.5,  0.0,    0.0, 1.0, 0.0,
                         -0.5,  0.5,  0.0,    0.0, 0.0, 1.0,
                          0.5,  0.5,  0.0,    1.0, 1.0, 1.0],
                        dtype=np.float32)

    indices = np.array([0, 1, 2,
                        1, 2, 3,
                        2, 3, 4],
                       np.uint32)

    def __init__(self):
        GameObject.__init__(self)
        glBindVertexArray(self.vao)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbos[0])
        glBufferData(GL_ARRAY_BUFFER, Quad.vertices.nbytes, Quad.vertices, GL_STATIC_DRAW)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, Quad.indices.nbytes, Quad.indices, GL_STATIC_DRAW)

        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, ctypes.c_void_p(0))

        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 24, ctypes.c_void_p(12))

        glBindBuffer(0)
        glBindVertexArray(0)
