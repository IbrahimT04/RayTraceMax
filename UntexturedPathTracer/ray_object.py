from OpenGL.GL import *
import numpy as np
from game_object import TexturedQuad, DepthQuad
from skybox import Skybox
from utilities import calc_compute_shaders, get_textures
from PIL import Image
from samples128 import samples128, samples256, samples1024

class RayTracer:
    info = {}
    def __init__(self, shader_program_name='pathtracer'):

        self.output_texture = None

        self.pathSpreadNeedsUpdate = True
        self.trianglesNeedUpdate = True
        self.spheresNeedUpdate = True
        self.planesNeedUpdate = True

        self.pathSpreadBuffer = None
        self.pathSpreadData = None
        # Path Spread
        self.pathSpread = samples1024

        self.triangleDataBuffer = None
        self.triangleData = None

        self.planeDataBuffer = None
        self.planeData = None

        self.sphereDataBuffer = None
        self.sphereData = None

        self.camera = None

        self.comp_shaders = calc_compute_shaders(shader_program_name)
        self.quad_screen = DepthQuad(shader_program_name)

        self.screenwidth, self.screenheight = 0, 0
        self.screendepth = len(self.pathSpread)

        glUseProgram(self.comp_shaders)

        self.create_color_buffer()
        self.create_resource_memory()
        self.add_path_spread()

        self.skybox = Skybox()
        self.skybox_texture = None

        self.triangle_objects = []
        self.sphere_objects = []
        self.plane_objects = []

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

    def create_color_buffer(self):
        self.screenwidth, self.screenheight = RayTracer.info["window_info"][0], RayTracer.info["window_info"][1]
        # glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, self.screenwidth, self.screenheight, 0, GL_RGBA, GL_FLOAT, None)
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA32F, self.screenwidth, self.screenheight, self.screendepth, 0, GL_RGBA, GL_FLOAT, None)

    def create_resource_memory(self):
        # Sphere Data Allocation
        self.sphereData = np.zeros(8192 * 8, dtype=np.float32)

        self.sphereDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.sphereDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.sphereData.nbytes, self.sphereData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.sphereDataBuffer)

        # Triangle Data Allocation
        self.triangleData = np.zeros(8192 * 16, dtype=np.float32)

        self.triangleDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.triangleDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.triangleData.nbytes, self.triangleData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, self.triangleDataBuffer)

        # Plane Data Allocation
        self.planeData = np.zeros(8192 * 20, dtype=np.float32)

        self.planeDataBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.planeDataBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.planeData.nbytes, self.planeData, GL_DYNAMIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, self.planeDataBuffer)

        # Path Spread
        self.pathSpreadBuffer = glGenBuffers(1)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.pathSpreadBuffer)

        glBufferData(GL_SHADER_STORAGE_BUFFER, self.pathSpread.nbytes, self.pathSpread, GL_STATIC_READ)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, self.pathSpreadBuffer)

    def add_path_spread(self):
        glUniform1f(glGetUniformLocation(self.comp_shaders, "path_spread_length"), len(self.pathSpread)//2)
        # print(len(self.pathSpread))

    def add_camera(self, cam):
        self.camera = cam

    def record_sphere(self, index, _sphere: 'Sphere'):
        self.spheresNeedUpdate = False
        self.sphereData[8 * index    ] = _sphere.center[0]
        self.sphereData[8 * index + 1] = _sphere.center[1]
        self.sphereData[8 * index + 2] = _sphere.center[2]

        self.sphereData[8 * index + 3] = _sphere.radius

        self.sphereData[8 * index + 4] = _sphere.color[0]
        self.sphereData[8 * index + 5] = _sphere.color[1]
        self.sphereData[8 * index + 6] = _sphere.color[2]

        self.sphereData[8 * index + 7] = _sphere.metalic

    def record_triangle(self, index, _triangle: 'Triangle'):
        self.trianglesNeedUpdate = False

        self.triangleData[16 * index    ] = _triangle.vertex1[0]
        self.triangleData[16 * index + 1] = _triangle.vertex1[1]
        self.triangleData[16 * index + 2] = _triangle.vertex1[2]
        self.triangleData[16 * index + 3] = 0.0   # padding

        self.triangleData[16 * index + 4] = _triangle.vertex2[0]
        self.triangleData[16 * index + 5] = _triangle.vertex2[1]
        self.triangleData[16 * index + 6] = _triangle.vertex2[2]
        self.triangleData[16 * index + 7] = 0.0

        self.triangleData[16 * index + 8] = _triangle.vertex3[0]
        self.triangleData[16 * index + 9] = _triangle.vertex3[1]
        self.triangleData[16 * index + 10] = _triangle.vertex3[2]
        self.triangleData[16 * index + 11] = 0.0

        self.triangleData[16 * index + 12] = _triangle.color[0]
        self.triangleData[16 * index + 13] = _triangle.color[1]
        self.triangleData[16 * index + 14] = _triangle.color[2]
        self.triangleData[16 * index + 15] = _triangle.metalic

    def record_plane(self, index, _plane: 'Plane'):
        self.planesNeedUpdate = False
        self.planeData[20 * index    ] = _plane.center[0]
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
        self.planeData[20 * index + 19] = _plane.metalic

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

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.sphereDataBuffer)

        if self.trianglesNeedUpdate:
            # Triangles Update
            triangles = self.triangle_objects
            glUniform1f(glGetUniformLocation(self.comp_shaders, "triangle_count"), len(triangles))

            for i, _triangle in enumerate(triangles):
                self.record_triangle(i, _triangle)

            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.triangleDataBuffer)
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 16 * 4 * len(triangles), self.triangleData)

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, self.triangleDataBuffer)

        if self.planesNeedUpdate:
            # Planes Update
            planes = self.plane_objects
            glUniform1f(glGetUniformLocation(self.comp_shaders, "plane_count"), len(planes))

            for i, _plane in enumerate(planes):
                self.record_plane(i, _plane)

            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.planeDataBuffer)
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 20 * 4 * len(planes), self.planeData)

            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, self.planeDataBuffer)
            self.skybox.use()

    def ray_draw(self):
        glUseProgram(self.comp_shaders)
        glActiveTexture(GL_TEXTURE0)
        glBindImageTexture(0, self.quad_screen.texture, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F)

        glDispatchCompute(self.screenwidth // 8, self.screenheight // 8, self.screendepth // 4)

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT)

        glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F)

        self.quad_screen.draw()

    def destroy(self):
        glBindVertexArray(0)
        self.comp_shaders = None
        self.quad_screen.destroy()
        self.quad_screen = None


class RayObject:
    isRayObj = True

    def __init__(self, color, refraction_index: float, metalic: float=1):
        self.color = np.array(color, dtype=np.float32)
        self.refraction_index = refraction_index
        self.metalic = metalic

    def draw(self):
        pass

    def delete(self):
        pass


class MeshRayObject(RayObject):
    pass


class Sphere(RayObject):
    def __init__(self, refraction_index: float, center, radius: float, color, metalic: float=1):
        RayObject.__init__(self, color, refraction_index, metalic)
        self.center = np.array(center, dtype=np.float32)
        self.radius = radius


class InfPlane(RayObject):
    def __init__(self, refraction_index: float, normal, center, color, metalic: float=1):
        super().__init__(color, refraction_index, metalic)
        self.normal = np.array(normal, dtype=np.float32)
        self.center = np.array(center, dtype=np.float32)


class Plane(InfPlane):
    def __init__(self, refraction_index: float, normal, tangent, bitangent,
                 center, color, u_min: float, u_max: float, v_min: float, v_max: float, metalic: float=1):
        super().__init__(refraction_index, normal, center, color, metalic)
        self.tangent = np.array(tangent, dtype=np.float32)
        self.bitangent = np.array(bitangent, dtype=np.float32)
        self.uMin = u_min
        self.uMax = u_max
        self.vMin = v_min
        self.vMax = v_max


class Triangle(RayObject):
    def __init__(self, refraction_index: float, vert1, vert2, vert3, color, metalic: float=1):
        RayObject.__init__(self, color, refraction_index, metalic)
        self.vertex1 = np.array(vert1, dtype=np.float32)
        self.vertex2 = np.array(vert2, dtype=np.float32)
        self.vertex3 = np.array(vert3, dtype=np.float32)
