from OpenGL.GL import *
import glfw

from scene_renderer import SceneRenderer


class Window:
    def __init__(self, scene: SceneRenderer, width: int = 1800, height: int = 1200, title: str = "My Window"):
        if not glfw.init():
            raise RuntimeError('glfw not initialized')
        self.window = glfw.create_window(width, height, title, None, None)

        if not self.window:
            glfw.terminate()
            raise RuntimeError('glfw window not created')

        glfw.set_window_size(self.window, width, height)
        glfw.set_window_size_callback(self.window, self.window_resize)

        self.update_context(self.window)

        glClearColor(0.3, 0.2, 0.3, 0.0)

    @staticmethod
    def window_resize(window, width, height):
        glViewport(0, 0, width, height)

    def update_context(self, context=None):
        if context is None:
            glfw.make_context_current(self.window)
        else:
            glfw.make_context_current(context)

    def get_status(self):
        return glfw.window_should_close(self.window)

    def get_inputs(self):
        glfw.poll_events()

    def update_buffer(self):
        glClear(GL_COLOR_BUFFER_BIT)
        glfw.swap_buffers(self.window)

    def destroy(self):
        glfw.terminate()
