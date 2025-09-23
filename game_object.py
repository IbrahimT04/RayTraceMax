from OpenGL.GL import *
import numpy as np

from utilities import calc_shaders, get_textures


class GameObject:
    info = {}
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
        self.vao = None
        self.vbos = None
        self.ebo = None


class Quad(GameObject):
    vertices = np.array([-1.0, -1.0,  0.0,   1.0, 0.0, 0.0,
                          1.0, -1.0,  0.0,    0.0, 1.0, 0.0,
                         -1.0,  1.0,  0.0,    0.0, 0.0, 1.0,
                          1.0,  1.0,  0.0,    1.0, 1.0, 1.0],
                        dtype=np.float32)

    indices = np.array([0, 1, 2,
                        1, 2, 3],
                       np.uint32)

    def __init__(self):
        GameObject.__init__(self)

        self.num_indices = len(self.indices)

        glBindVertexArray(self.vao)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbos)
        glBufferData(GL_ARRAY_BUFFER, Quad.vertices.nbytes, Quad.vertices, GL_STATIC_DRAW)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, Quad.indices.nbytes, Quad.indices, GL_STATIC_DRAW)

        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, ctypes.c_void_p(0))

        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 24, ctypes.c_void_p(12))

        glBindVertexArray(0)

    def draw(self):
        glBindVertexArray(self.vao)
        glUseProgram(self.shader_program)
        glDrawElements(GL_TRIANGLES, self.num_indices, GL_UNSIGNED_INT, ctypes.c_void_p(0))

    def destroy(self):
        GameObject.destroy(self)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

class TexturedQuad(Quad):
    vertices = np.array([-1.0,  -1.0,  0.0,     0.0, 0.0,
                          1.0, -1.0,  0.0,    1.0, 0.0,
                         -1.0,  1.0,  0.0,    0.0, 1.0,
                          1.0,  1.0,  0.0,    1.0, 1.0],
                        dtype=np.float32)

    indices = np.array([0, 1, 2,
                        1, 2, 3],
                       np.uint32)
    def __init__(self, shader_program_name='default'):
        GameObject.__init__(self, shader_program_name=shader_program_name)

        self.num_indices = len(self.indices)

        glBindVertexArray(self.vao)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbos)
        glBufferData(GL_ARRAY_BUFFER, TexturedQuad.vertices.nbytes, TexturedQuad.vertices, GL_STATIC_DRAW)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, TexturedQuad.indices.nbytes, TexturedQuad.indices, GL_STATIC_DRAW)

        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, TexturedQuad.indices.itemsize * 5, ctypes.c_void_p(0))

        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, TexturedQuad.indices.itemsize * 5, ctypes.c_void_p(12))

        self.texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.texture)

        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)

        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)

        glBindVertexArray(0)

    def add_texture(self, texture):
        glBindVertexArray(self.vao)
        image, image_data = get_textures(texture)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glBindVertexArray(0)
