from OpenGL.GL import *
import numpy as np

from UntexturedPathTracer.lighting import Light
from UntexturedPathTracer.game_object import DepthQuad
from UntexturedPathTracer.skybox import Skybox
from UntexturedPathTracer.utilities import calc_compute_shaders, get_textures
from UntexturedPathTracer.samples128 import samples128, samples256, samples512, samples1024

class RayTracer:
    info = {}
    def __init__(self, shader_program_name='pathtracer'):

        self.ray_texture = None
        self.output_texture = None

        self.pathSpreadNeedsUpdate = True
        self.trianglesNeedUpdate = True
        self.spheresNeedUpdate = True
        self.planesNeedUpdate = True
        self.lightsNeedUpdate = True

        self.pathSpreadBuffer = None
        self.pathSpreadData = None
        # Path Spread
        self.pathSpread = samples128

        self.triangleDataBuffer = None
        self.triangleData = None

        self.planeDataBuffer = None
        self.planeData = None

        self.sphereDataBuffer = None
        self.sphereData = None

        self.lightDataBuffer = None
        self.lightData = None

        self.camera = None

        self.path_program = calc_compute_shaders(shader_program_name)
        self.ray_program = calc_compute_shaders(shader_program_name + '_pre')
        self.quad_screen = DepthQuad(shader_program_name)

        self.screenwidth, self.screenheight = 0, 0
        self.screendepth = len(self.pathSpread)

        glUseProgram(self.path_program)

        self.create_color_buffer()
        self.create_reflection_buffer()
        self.create_resource_memory()

        self.skybox = Skybox()
        self.skybox_texture = None

        self.triangle_objects = []
        self.sphere_objects = []
        self.plane_objects = []
        self.light_objects = []

    def add_object(self, ray_object):
        if isinstance(ray_object, Sphere):
            self.sphere_objects.append(ray_object)
            self.spheresNeedUpdate = True
        elif isinstance(ray_object, Triangle):
            self.triangle_objects.append(ray_object)
            self.trianglesNeedUpdate = True
        elif isinstance(ray_object, Plane):
            self.plane_objects.append(ray_object)
            self.planesNeedUpdate = True
        elif isinstance(ray_object, Light):
            self.light_objects.append(ray_object)
            self.lightsNeedUpdate = True

    def create_color_buffer(self):
        self.screenwidth, self.screenheight = RayTracer.info["window_info"][0], RayTracer.info["window_info"][1]
        # glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, self.screenwidth, self.screenheight, 0, GL_RGBA, GL_FLOAT, None)
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA32F, self.screenwidth, self.screenheight, self.screendepth, 0, GL_RGBA, GL_FLOAT, None)

        # Compute Output
        glActiveTexture(GL_TEXTURE0)
        glBindImageTexture(0, self.quad_screen.texture, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F)

    def create_reflection_buffer(self):
        self.ray_texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.ray_texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, self.screenwidth, self.screenheight, 0, GL_RGBA, GL_FLOAT, None)

        # Pre-calculated reflections
        glActiveTexture(GL_TEXTURE7)
        glBindImageTexture(7, self.ray_texture, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA32F)

    def create_resource_memory(self):
        # Sphere Data Allocation
        self.sphereData = np.zeros(8192 * 8, dtype=np.float32)

        self.sphereDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.sphereDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.sphereData.nbytes, self.sphereData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.sphereDataBuffer)

        # Triangle Data Allocation
        self.triangleData = np.zeros(8192 * 20, dtype=np.float32)

        self.triangleDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.triangleDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.triangleData.nbytes, self.triangleData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, self.triangleDataBuffer)

        # Plane Data Allocation
        self.planeData = np.zeros(8192 * 24, dtype=np.float32)

        self.planeDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.planeDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.planeData.nbytes, self.planeData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, self.planeDataBuffer)

        # Light Data Allocation
        self.lightData = np.zeros(8192 * 24, dtype=np.float32)

        self.lightDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.lightDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.lightData.nbytes, self.lightData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, self.lightDataBuffer)

        # Path Spread
        self.pathSpreadBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.pathSpreadBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.pathSpread.nbytes, self.pathSpread, GL_STATIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, self.pathSpreadBuffer)

    def add_camera(self, cam):
        self.camera = cam

    def record_sphere(self, index, _sphere: 'Sphere'):
        self.spheresNeedUpdate = False
        self.sphereData[12 * index + 0] = _sphere.center[0]
        self.sphereData[12 * index + 1] = _sphere.center[1]
        self.sphereData[12 * index + 2] = _sphere.center[2]

        self.sphereData[12 * index + 3] = _sphere.radius

        self.sphereData[12 * index + 4] = _sphere.color[0]
        self.sphereData[12 * index + 5] = _sphere.color[1]
        self.sphereData[12 * index + 6] = _sphere.color[2]

        self.sphereData[12 * index + 7] = _sphere.metalic

        self.sphereData[12 * index + 8] = _sphere.emissive[0]
        self.sphereData[12 * index + 9] = _sphere.emissive[1]
        self.sphereData[12 * index + 10] = _sphere.emissive[2]

    def record_triangle(self, index, _triangle: 'Triangle'):
        self.trianglesNeedUpdate = False

        self.triangleData[20 * index + 0] = _triangle.vertex1[0]
        self.triangleData[20 * index + 1] = _triangle.vertex1[1]
        self.triangleData[20 * index + 2] = _triangle.vertex1[2]
        self.triangleData[20 * index + 3] = 0.0   # padding

        self.triangleData[20 * index + 4] = _triangle.vertex2[0]
        self.triangleData[20 * index + 5] = _triangle.vertex2[1]
        self.triangleData[20 * index + 6] = _triangle.vertex2[2]
        self.triangleData[20 * index + 7] = 0.0

        self.triangleData[20 * index + 8] = _triangle.vertex3[0]
        self.triangleData[20 * index + 9] = _triangle.vertex3[1]
        self.triangleData[20 * index + 10] = _triangle.vertex3[2]
        self.triangleData[20 * index + 11] = 0.0

        self.triangleData[20 * index + 12] = _triangle.color[0]
        self.triangleData[20 * index + 13] = _triangle.color[1]
        self.triangleData[20 * index + 14] = _triangle.color[2]
        self.triangleData[20 * index + 15] = _triangle.metalic

        self.triangleData[20 * index + 20] = _triangle.emissive[0]
        self.triangleData[20 * index + 17] = _triangle.emissive[1]
        self.triangleData[20 * index + 18] = _triangle.emissive[2]

    def record_plane(self, index, _plane: 'Plane'):
        self.planesNeedUpdate = False
        self.planeData[24 * index + 0] = _plane.center[0]
        self.planeData[24 * index + 1] = _plane.center[1]
        self.planeData[24 * index + 2] = _plane.center[2]
        self.planeData[24 * index + 3] = _plane.uMin

        self.planeData[24 * index + 4] = _plane.tangent[0]
        self.planeData[24 * index + 5] = _plane.tangent[1]
        self.planeData[24 * index + 6] = _plane.tangent[2]
        self.planeData[24 * index + 7] = _plane.uMax

        self.planeData[24 * index + 8] = _plane.bitangent[0]
        self.planeData[24 * index + 9] = _plane.bitangent[1]
        self.planeData[24 * index + 10] = _plane.bitangent[2]
        self.planeData[24 * index + 11] = _plane.vMin

        self.planeData[24 * index + 12] = _plane.normal[0]
        self.planeData[24 * index + 13] = _plane.normal[1]
        self.planeData[24 * index + 14] = _plane.normal[2]
        self.planeData[24 * index + 15] = _plane.vMax

        self.planeData[24 * index + 16] = _plane.color[0]
        self.planeData[24 * index + 17] = _plane.color[1]
        self.planeData[24 * index + 18] = _plane.color[2]
        self.planeData[24 * index + 19] = _plane.metalic

        self.planeData[24 * index + 20] = _plane.emissive[0]
        self.planeData[24 * index + 21] = _plane.emissive[1]
        self.planeData[24 * index + 22] = _plane.emissive[2]

    def record_light(self, index, _light: 'Light'):
        self.lightsNeedUpdate = False
        self.lightData[8 * index    ] = _light.position[0]
        self.lightData[8 * index + 1] = _light.position[1]
        self.lightData[8 * index + 2] = _light.position[2]

        self.lightData[8 * index + 3] = _light.color[0]
        self.lightData[8 * index + 4] = _light.color[1]
        self.lightData[8 * index + 5] = _light.color[2]

        self.lightData[8 * index + 6] = _light.strength


    def prepare_scene(self):

        glUseProgram(self.ray_program)
        glUniform3fv(glGetUniformLocation(self.ray_program, "viewer.position"), 1, self.camera.position)
        glUniform3fv(glGetUniformLocation(self.ray_program, "viewer.forwards"), 1, self.camera.forwards)
        glUniform3fv(glGetUniformLocation(self.ray_program, "viewer.right"), 1, self.camera.right)
        glUniform3fv(glGetUniformLocation(self.ray_program, "viewer.up"), 1, self.camera.up)

        if self.spheresNeedUpdate:
            # Spheres Update
            spheres = self.sphere_objects
            glUniform1f(glGetUniformLocation(self.ray_program, "sphere_count"), len(spheres))

        if self.trianglesNeedUpdate:
            # Triangles Update
            triangles = self.triangle_objects
            glUniform1f(glGetUniformLocation(self.ray_program, "triangle_count"), len(triangles))

        if self.planesNeedUpdate:
            # Planes Update
            planes = self.plane_objects
            glUniform1f(glGetUniformLocation(self.ray_program, "plane_count"), len(planes))

        glUseProgram(self.path_program)
        glUniform3fv(glGetUniformLocation(self.path_program, "viewer.position"), 1, self.camera.position)
        glUniform3fv(glGetUniformLocation(self.path_program, "viewer.forwards"), 1, self.camera.forwards)
        glUniform3fv(glGetUniformLocation(self.path_program, "viewer.right"), 1, self.camera.right)
        glUniform3fv(glGetUniformLocation(self.path_program, "viewer.up"), 1, self.camera.up)

        if self.spheresNeedUpdate:
            # Spheres Update
            spheres = self.sphere_objects
            glUniform1f(glGetUniformLocation(self.path_program, "sphere_count"), len(spheres))

            for i, _sphere in enumerate(spheres):
                self.record_sphere(i, _sphere)

            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.sphereDataBuffer)
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 12 * 4 * len(spheres), self.sphereData)

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.sphereDataBuffer)

        if self.trianglesNeedUpdate:
            # Triangles Update
            triangles = self.triangle_objects
            glUniform1f(glGetUniformLocation(self.path_program, "triangle_count"), len(triangles))

            for i, _triangle in enumerate(triangles):
                self.record_triangle(i, _triangle)

            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.triangleDataBuffer)
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 20 * 4 * len(triangles), self.triangleData)

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, self.triangleDataBuffer)

        if self.planesNeedUpdate:
            # Planes Update
            planes = self.plane_objects
            glUniform1f(glGetUniformLocation(self.path_program, "plane_count"), len(planes))

            for i, _plane in enumerate(planes):
                self.record_plane(i, _plane)

            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.planeDataBuffer)
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 24 * 4 * len(planes), self.planeData)

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, self.planeDataBuffer)
            self.skybox.use()

    def ray_calc_reflect(self):
        glUseProgram(self.ray_program)
        glActiveTexture(GL_TEXTURE7)
        glBindImageTexture(7, self.ray_texture, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA32F)

    def ray_draw(self):
        glUseProgram(self.ray_program)
        glDispatchCompute(self.screenwidth // 8, self.screenheight // 8, 1)

        glUseProgram(self.path_program)
        # glActiveTexture(GL_TEXTURE0)
        # glBindImageTexture(0, self.quad_screen.texture, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F)

        glDispatchCompute(self.screenwidth // 8, self.screenheight // 8, self.screendepth // 4)

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT)

        # glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F)

        self.quad_screen.draw()

    def destroy(self):
        glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F)
        glBindVertexArray(0)
        self.path_program = None
        self.quad_screen.destroy()
        self.quad_screen = None


class RayObject:
    isRayObj = True

    def __init__(self, color, emissive, refraction_index: float, metalic: float=1):
        self.color = np.array(color, dtype=np.float32)
        self.emissive = np.array(emissive, dtype=np.float32)
        self.refraction_index = refraction_index
        self.metalic = metalic

    def draw(self):
        pass

    def delete(self):
        pass


class MeshRayObject(RayObject):
    pass


class Sphere(RayObject):
    def __init__(self, refraction_index: float, center, radius: float, color, emissive, metalic: float=1):
        RayObject.__init__(self, color, emissive, refraction_index, metalic)
        self.center = np.array(center, dtype=np.float32)
        self.radius = radius


class InfPlane(RayObject):
    def __init__(self, refraction_index: float, normal, center, color, emissive, metalic: float=1):
        super().__init__(color, emissive, refraction_index, metalic)
        self.normal = np.array(normal, dtype=np.float32)
        self.center = np.array(center, dtype=np.float32)


class Plane(InfPlane):
    def __init__(self, refraction_index: float, normal, tangent, bitangent, emissive,
                 center, color, u_min: float, u_max: float, v_min: float, v_max: float, metalic: float=1):
        super().__init__(refraction_index, normal, center, color, emissive, metalic)
        self.tangent = np.array(tangent, dtype=np.float32)
        self.bitangent = np.array(bitangent, dtype=np.float32)
        self.uMin = u_min
        self.uMax = u_max
        self.vMin = v_min
        self.vMax = v_max


class Triangle(RayObject):
    def __init__(self, refraction_index: float, vert1, vert2, vert3, color, emissive, metalic: float=1):
        RayObject.__init__(self, color, emissive, refraction_index, metalic)
        self.vertex1 = np.array(vert1, dtype=np.float32)
        self.vertex2 = np.array(vert2, dtype=np.float32)
        self.vertex3 = np.array(vert3, dtype=np.float32)
