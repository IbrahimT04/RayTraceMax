from game_object import GameObject, Quad
from ray_object import RayTracer


class SceneRenderer:
    def __init__(self):
        self._objects = []
        self.info = {}
        GameObject.info = self.get_info()
        RayTracer.info = self.get_info()

    def add_object(self, obj):
        self._objects.append(obj)

    def input_info(self, input_name, input):
        self.info[input_name] = input

    def ray_tracer(self):
        pass

    def render(self):
        for obj in self._objects:
            obj.draw()

    def get_info(self):
        return self.info

    def destroy(self):
        for obj in self._objects:
            obj.destroy()

class Scene:
    def __init__(self, scene_renderer):
        scene_renderer.add_object(RayTracer())