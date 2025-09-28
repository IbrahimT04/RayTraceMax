from OpenGL.GL import *
import numpy as np
from game_object import TexturedQuad

from utilities import calc_compute_shaders


class RayTracer:
    info = {}
    def __init__(self, shader_program_name='raytracer'):
        self.comp_shaders = calc_compute_shaders(shader_program_name)
        self.quad_screen = TexturedQuad(shader_program_name)
        self.screenwidth, self.screenheight = 0, 0
        self.create_color_buffer()

    def create_color_buffer(self):

        self.screenwidth, self.screenheight = RayTracer.info["window_info"][0], RayTracer.info["window_info"][1]
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, self.screenwidth, self.screenheight, 0, GL_RGBA, GL_FLOAT, None)

    def pre_bake(self):
        pass

    def ray_draw(self):
        glUseProgram(self.comp_shaders)
        glActiveTexture(GL_TEXTURE0)
        glBindImageTexture(0, self.quad_screen.texture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F)

        glDispatchCompute(self.screenwidth//8, self.screenheight//8, 1)

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT)

        glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F)

        self.quad_screen.draw()

    def destroy(self):
        glBindVertexArray(0)
        self.comp_shaders = None
        self.quad_screen.destroy()
        self.quad_screen = None

class RayObject:
    def __init__(self, color, refraction_index):
        self.color = np.array(color, dtype=np.float32)
        self.refraction_index = refraction_index
    def draw(self):
        pass

class MeshRayObject(RayObject):
    pass

class Sphere(RayObject):
    def __init__(self, refraction_index: float, center: np.ndarray, radius: float, color: np.ndarray):
        RayObject.__init__(self, color, refraction_index)
        self.center = np.array(center,dtype=np.float32)
        self.radius = radius
    def destroy(self):
        pass