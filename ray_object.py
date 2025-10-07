from OpenGL.GL import *
import numpy as np
from game_object import TexturedQuad
from lighting import Light
from skybox import Skybox
from utilities import calc_compute_shaders, get_textures
from PIL import Image


class RayTracer:
    info = {}

    def __init__(self, shader_program_name='raytracer'):

        self.output_texture = None

        self.spheresNeedUpdate = True
        self.planesNeedUpdate = True
        self.lightsNeedUpdate = True

        self.planeDataBuffer = None
        self.planeData = None

        self.sphereDataBuffer = None
        self.sphereData = None

        self.lightDataBuffer = None
        self.lightData = None

        self.camera = None

        """# Confirm unbinding
        glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_2D, 0)"""

        self.comp_shaders = calc_compute_shaders(shader_program_name)
        self.quad_screen = TexturedQuad(shader_program_name)

        self.screenwidth, self.screenheight = 0, 0

        glUseProgram(self.comp_shaders)

        """loc = glGetUniformLocation(self.comp_shaders, "skybox")
        if loc != -1:
            glUniform1i(loc, 2)"""

        self.create_color_buffer()
        self.create_resource_memory()

        self.skybox = Skybox()
        self.skybox_texture = None

        self.sphere_objects = []
        self.plane_objects = []
        self.light_objects = []

    def add_object(self, ray_object):
        if isinstance(ray_object, Sphere):
            self.sphere_objects.append(ray_object)
            self.spheresNeedUpdate = True
        elif isinstance(ray_object, Plane):
            self.plane_objects.append(ray_object)
            self.planesNeedUpdate = True
        else:
            self.light_objects.append(ray_object)
            self.lightsNeedUpdate = True

    def create_color_buffer(self):
        self.screenwidth, self.screenheight = RayTracer.info["window_info"][0], RayTracer.info["window_info"][1]
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, self.screenwidth, self.screenheight, 0, GL_RGBA, GL_FLOAT, None)

    def create_resource_memory(self):
        # Sphere Data Allocation
        self.sphereData = np.zeros(8192 * 8, dtype=np.float32)

        self.sphereDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.sphereDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.sphereData.nbytes, self.sphereData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self.sphereDataBuffer)

        # Plane Data Allocation
        self.planeData = np.zeros(8192 * 20, dtype=np.float32)

        self.planeDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.planeDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.planeData.nbytes, self.planeData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.planeDataBuffer)

        # Light Data Allocation
        self.lightData = np.zeros(8192 * 20, dtype=np.float32)

        self.lightDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.lightDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.lightData.nbytes, self.lightData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, self.lightDataBuffer)

    def add_camera(self, cam):
        self.camera = cam

    def record_light(self, index, _light: 'Light'):
        self.lightsNeedUpdate = False
        self.lightData[8 * index    ] = _light.position[0]
        self.lightData[8 * index + 1] = _light.position[1]
        self.lightData[8 * index + 2] = _light.position[2]

        self.lightData[8 * index + 3] = _light.color[0]
        self.lightData[8 * index + 4] = _light.color[1]
        self.lightData[8 * index + 5] = _light.color[2]

        self.lightData[8 * index + 6] = _light.strength

    def record_sphere(self, index, _sphere: 'Sphere'):
        self.spheresNeedUpdate = False
        self.sphereData[8 * index] = _sphere.center[0]
        self.sphereData[8 * index + 1] = _sphere.center[1]
        self.sphereData[8 * index + 2] = _sphere.center[2]

        self.sphereData[8 * index + 3] = _sphere.radius

        self.sphereData[8 * index + 4] = _sphere.color[0]
        self.sphereData[8 * index + 5] = _sphere.color[1]
        self.sphereData[8 * index + 6] = _sphere.color[2]

    def record_plane(self, index, _plane: 'Plane'):
        self.planesNeedUpdate = False
        self.planeData[20 * index] = _plane.center[0]
        self.planeData[20 * index + 1] = _plane.center[1]
        self.planeData[20 * index + 2] = _plane.center[2]

        self.planeData[20 * index + 3] = _plane.uMin

        self.planeData[20 * index + 4] = _plane.tangent[0]
        self.planeData[20 * index + 5] = _plane.tangent[1]
        self.planeData[20 * index + 6] = _plane.tangent[2]

        self.planeData[20 * index + 7] = _plane.uMax

        self.planeData[20 * index + 8] = _plane.bitangent[0]
        self.planeData[20 * index + 9] = _plane.bitangent[1]
        self.planeData[20 * index + 10] = _plane.bitangent[2]

        self.planeData[20 * index + 11] = _plane.vMin

        self.planeData[20 * index + 12] = _plane.normal[0]
        self.planeData[20 * index + 13] = _plane.normal[1]
        self.planeData[20 * index + 14] = _plane.normal[2]

        self.planeData[20 * index + 15] = _plane.vMax

        self.planeData[20 * index + 16] = _plane.color[0]
        self.planeData[20 * index + 17] = _plane.color[1]
        self.planeData[20 * index + 18] = _plane.color[2]

    def prepare_scene(self):

        glUseProgram(self.comp_shaders)
        glUniform3fv(glGetUniformLocation(self.comp_shaders, "viewer.position"), 1, self.camera.position)
        glUniform3fv(glGetUniformLocation(self.comp_shaders, "viewer.forwards"), 1, self.camera.forwards)
        glUniform3fv(glGetUniformLocation(self.comp_shaders, "viewer.right"), 1, self.camera.right)
        glUniform3fv(glGetUniformLocation(self.comp_shaders, "viewer.up"), 1, self.camera.up)

        if self.spheresNeedUpdate:
            # Spheres Update
            spheres = self.sphere_objects
            glUniform1f(glGetUniformLocation(self.comp_shaders, "sphere_count"), len(spheres))

            for i, _sphere in enumerate(spheres):
                self.record_sphere(i, _sphere)

            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.sphereDataBuffer)
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 8 * 4 * len(spheres), self.sphereData)

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self.sphereDataBuffer)

        if self.planesNeedUpdate:
            # Planes Update
            planes = self.plane_objects
            glUniform1f(glGetUniformLocation(self.comp_shaders, "plane_count"), len(planes))

            for i, _plane in enumerate(planes):
                self.record_plane(i, _plane)

            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.planeDataBuffer)
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 19 * 4 * len(planes), self.planeData)

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.planeDataBuffer)

        if self.lightsNeedUpdate:
            # Lights Update
            lights = self.light_objects
            glUniform1f(glGetUniformLocation(self.comp_shaders, "light_count"), len(lights))

            for i, _light in enumerate(lights):
                self.record_light(i, _light)

            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.lightDataBuffer)
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 19 * 4 * len(lights), self.lightData)

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, self.lightDataBuffer)

        self.skybox.use()

    def ray_draw(self):
        glUseProgram(self.comp_shaders)
        glActiveTexture(GL_TEXTURE0)
        glBindImageTexture(0, self.quad_screen.texture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F)

        glDispatchCompute(self.screenwidth // 8, self.screenheight // 8, 1)

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

    def __init__(self, color, refraction_index: float, smoothness=1):
        self.color = np.array(color, dtype=np.float32)
        self.refraction_index = refraction_index
        self.smoothness = smoothness

    def draw(self):
        pass


class MeshRayObject(RayObject):
    pass


class Sphere(RayObject):
    def __init__(self, refraction_index: float, center, radius: float, color):
        RayObject.__init__(self, color, refraction_index)
        self.center = np.array(center, dtype=np.float32)
        self.radius = radius

    def destroy(self):
        pass


class InfPlane(RayObject):
    def __init__(self, refraction_index: float, normal, center, color):
        super().__init__(color, refraction_index)
        self.normal = np.array(normal, dtype=np.float32)
        self.center = np.array(center, dtype=np.float32)

    def destroy(self):
        pass


class Plane(InfPlane):
    def __init__(self, refraction_index: float, normal, tangent, bitangent,
                 center, color, u_min: float, u_max: float, v_min: float, v_max: float):
        super().__init__(refraction_index, normal, center, color)
        self.tangent = np.array(tangent, dtype=np.float32)
        self.bitangent = np.array(bitangent, dtype=np.float32)
        self.uMin = u_min
        self.uMax = u_max
        self.vMin = v_min
        self.vMax = v_max

    def destroy(self):
        pass
