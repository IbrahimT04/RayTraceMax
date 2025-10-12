import numpy as np

import camera
import lighting
from game_object import GameObject
from ray_object import RayTracer, Sphere, Plane, RayObject, Triangle


class SceneRenderer:
    def __init__(self):
        self.camera = None
        self.rayTracer = None
        self.scene = None
        self._objects = []
        self.info = {}
        GameObject.info = self.get_info()
        RayTracer.info = self.get_info()

    def add_camera(self, cam: camera.Camera):
        self.camera = cam
        self.rayTracer.add_camera(cam)

    def add_object(self, obj):
        if isinstance(obj, RayObject) or isinstance(obj, lighting.Light):
            self.rayTracer.add_object(obj)
        else:
            self._objects.append(obj)

    def input_info(self, input_name, input_value):
        self.info[input_name] = input_value

    def render(self):
        for obj in self._objects:
            obj.draw()

    def prebake(self):
        self.rayTracer = RayTracer()

    def get_info(self):
        return self.info

    def destroy(self):
        for obj in self._objects:
            obj.destroy()

    def ray_trace_render(self):
        self.rayTracer.prepare_scene()
        self.rayTracer.ray_draw()


class Scene:
    def __init__(self, scene_renderer: SceneRenderer, window):

        # Need to add this functionality
        self.outDated = True

        self.renderer = scene_renderer

        self.spheres = [
            Sphere(
                refraction_index=np.random.uniform(low=0.9, high=1.1),
                center=[
                    np.random.uniform(low=-10.0, high=10.0),
                    np.random.uniform(low=-10.0, high=10.0),
                    np.random.uniform(low=-10.0, high=10.0)
                ],
                radius=np.random.uniform(low=0.3, high=2.0),
                color=[
                    np.random.uniform(low=0.3, high=1.0),
                    np.random.uniform(low=0.3, high=1.0),
                    np.random.uniform(low=0.3, high=1.0)
                ]
            ) for _ in range(16)
        ]

        self.planes = [
            Plane(
                refraction_index=np.random.uniform(low=0.9, high=1.1),
                normal=[0, 0, 1],
                tangent=[1, 0, 0],
                bitangent=[0, 1, 0],
                u_min=-10,
                u_max=10,
                v_min=-10,
                v_max=10,
                center=[0, 0, -3],
                color=[
                    np.random.uniform(low=0.2, high=0.9),
                    np.random.uniform(low=0.2, high=0.9),
                    np.random.uniform(low=0.2, high=0.9)
                ]
            ),
        ]

        self.triangles = [
            Triangle(
                refraction_index=np.random.uniform(low=0.9, high=1.1),
                vert1=[0.0, 0.0, 0.0],
                vert2=[5.0, 5.0, 10.0],
                vert3=[10.0, 10.0, 5.0],
                color=[
                    np.random.uniform(low=0.3, high=0.9),
                    np.random.uniform(low=0.3, high=0.9),
                    np.random.uniform(low=0.3, high=0.9)
                ]
            ) # for i in range(9)
        ]

        self.camera = camera.Camera(
            position=[-10, 0, 0]
        )
        """self.lights = [lighting.Light(position=[np.random.uniform(low=-5.0, high=5.0),
                                                np.random.uniform(low=-5.0, high=5.0),
                                                np.random.uniform(low=5.0, high=10.0)],
                                      color=[np.random.uniform(low=0.7, high=1.0),
                                             np.random.uniform(low=0.7, high=1.0),
                                             np.random.uniform(low=0.7, high=1.0)],
                                      strength=np.random.uniform(low=1.0, high=3.0)
                                      ) for i in range(9)
                       ]"""
        self.renderer.add_camera(self.camera)
        for sphere in self.spheres:
            self.renderer.add_object(sphere)
        for triangle in self.triangles:
            self.renderer.add_object(triangle)
        for plane in self.planes:
            self.renderer.add_object(plane)

        """
        for light in self.lights:
            self.renderer.add_object(light)
        """

        window.attach_camera(self.camera)
