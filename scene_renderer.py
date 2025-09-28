import numpy as np

import camera
from game_object import GameObject, Quad, Cube
from ray_object import RayTracer, Sphere


class SceneRenderer:
    def __init__(self):
        self.camera = None
        self.rayTracer = None
        self.scene = None
        self._objects = []
        self._ray_objects = []
        self.info = {}
        GameObject.info = self.get_info()
        RayTracer.info = self.get_info()

    def add_camera(self, cam: camera.Camera):
        self.camera = cam
        self.rayTracer.add_camera(cam)

    def add_object(self, obj):
        if obj.isRayObj:
            self._ray_objects.append(obj)
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
        self.rayTracer.prepare_scene(self._ray_objects)
        self.rayTracer.ray_draw()


class Scene:
    def __init__(self, scene_renderer: SceneRenderer):
        self.renderer = scene_renderer
        self.renderer.add_object(Cube())

        self.spheres = [
            Sphere(
                refraction_index=np.random.uniform(low=0.9, high=1.1),
                center=[
                    np.random.uniform(low=3.0, high=10.0),
                    np.random.uniform(low=-8.0, high=8.0),
                    np.random.uniform(low=-8.0, high=8.0)
                ],
                radius=np.random.uniform(low=0.1, high=2.0),
                color=[
                    np.random.uniform(low=0.3, high=1.0),
                    np.random.uniform(low=0.3, high=1.0),
                    np.random.uniform(low=0.3, high=1.0)
                ]
            ) for _ in range(1024)
        ]
        self.camera = camera.Camera(
            position=[0, 0, 0]
        )
        self.renderer.add_camera(self.camera)
        for sphere in self.spheres:
            self.renderer.add_object(sphere)
