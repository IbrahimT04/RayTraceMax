from OpenGL.GL import *
import numpy as np
from game_object import TexturedQuad

from utilities import calc_compute_shaders


class RayTracer:
    info = {}
    def __init__(self, shader_program_name='raytracer'):
        self.sphereDataBuffer = None
        self.sphereData = None
        self.camera = None
        self.comp_shaders = calc_compute_shaders(shader_program_name)
        self.quad_screen = TexturedQuad(shader_program_name)
        self.screenwidth, self.screenheight = 0, 0
        self.create_color_buffer()
        self.create_resource_memory()

    def create_color_buffer(self):
        self.screenwidth, self.screenheight = RayTracer.info["window_info"][0], RayTracer.info["window_info"][1]
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, self.screenwidth, self.screenheight, 0, GL_RGBA, GL_FLOAT, None)

    def create_resource_memory(self):
        self.sphereData = np.zeros(1024 * 8, dtype=np.float32)

        self.sphereDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.sphereDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.sphereData.nbytes, self.sphereData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self.sphereDataBuffer)

    def add_camera(self, cam):
        self.camera = cam

    def record_sphere(self, index, _sphere: 'Sphere'):
        self.sphereData[8 * index    ] = _sphere.center[0]
        self.sphereData[8 * index + 1] = _sphere.center[1]
        self.sphereData[8 * index + 2] = _sphere.center[2]

        self.sphereData[8 * index + 3] = _sphere.radius

        self.sphereData[8 * index + 4] = _sphere.color[0]
        self.sphereData[8 * index + 5] = _sphere.color[1]
        self.sphereData[8 * index + 6] = _sphere.color[2]

    def prepare_scene(self, ray_objects):
        glUseProgram(self.comp_shaders)
        glUniform3fv(glGetUniformLocation(self.comp_shaders, "viewer.position"),1, self.camera.position)
        glUniform3fv(glGetUniformLocation(self.comp_shaders, "viewer.forwards"),1, self.camera.forwards)
        glUniform3fv(glGetUniformLocation(self.comp_shaders, "viewer.right"),1, self.camera.right)
        glUniform3fv(glGetUniformLocation(self.comp_shaders, "viewer.up"),1, self.camera.up)

        glUniform1f(glGetUniformLocation(self.comp_shaders, "sphere_count"), len(ray_objects))

        for i, _sphere in enumerate(ray_objects):
            self.record_sphere(i, _sphere)

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.sphereDataBuffer)
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 8 * 4 * len(ray_objects), self.sphereData)

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self.sphereDataBuffer)




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
    isRayObj = True
    def __init__(self, color: np.ndarray, refraction_index: float, smoothness=1):
        self.color = np.array(color, dtype=np.float32)
        self.refraction_index = refraction_index
        self.smoothness = smoothness
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


class InfPlane(RayObject):
    def __init__(self, refraction_index: float, normal: np.ndarray, center: np.ndarray, color: np.ndarray):
        super().__init__(color, refraction_index)
        self.normal = np.array(normal, dtype=np.float32)
        self.center = np.array(center, dtype=np.float32)
    def destroy(self):
        pass


class Plane(InfPlane):
    def __init__(self, refraction_index: float, normal: np.ndarray, tangent: np.ndarray, bitangent: np.ndarray,
                 center: np.ndarray, color: np.ndarray, u_min: float, u_max: float, v_min: float, v_max: float):
        super().__init__(refraction_index, normal, center, color)
        self.tangent = np.array(tangent, dtype=np.float32)
        self.bitangent = np.array(bitangent, dtype=np.float32)
        self.uMin = u_min
        self.uMax = u_max
        self.vMin = v_min
        self.vMax = v_max
    def destroy(self):
        pass