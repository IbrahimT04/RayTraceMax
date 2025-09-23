from game_object import GameObject, Quad


class SceneRenderer:
    def __init__(self):
        self._objects = []
        self.add_object(Quad())

    def add_object(self, obj):
        self._objects.append(obj)


    def render(self):
        for obj in self._objects:
            obj.draw()

    def destroy(self):
        for obj in self._objects:
            obj.destroy()
