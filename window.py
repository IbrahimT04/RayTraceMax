from OpenGL.GL import *
import glfw

class Window:
    def __init__(self, width: int = 1800, height: int = 1200, title: str = "My Window"):
        self.width = width
        self.height = height
        self.title = title

        if not glfw.init():
            raise RuntimeError('glfw not initialized')
        self.window = glfw.create_window(width, height, title, None, None)

        if not self.window:
            glfw.terminate()
            raise RuntimeError('glfw window not created')

        glfw.set_window_pos(self.window, 400, 200)
        glfw.set_window_size_callback(self.window, self.window_resize)

        self.update_context(self.window)

        glClearColor(0.3, 0.2, 0.3, 0.0)

        # --- FPS tracking (minimal, non-breaking) ---
        self._frame_count = 0
        self._last_fps_time = glfw.get_time()
        self._fps = 0.0
        # update title every 0.5s by default; change if you want
        self.fps_update_interval = 0.5

    @staticmethod
    def window_resize(window, width, height):
        glViewport(0, 0, width, height)

    def get_window_info(self):
        return self.width, self.height, self.title

    def update_context(self, context=None):
        if context is None:
            glfw.make_context_current(self.window)
        else:
            glfw.make_context_current(context)

    def get_status(self):
        return glfw.window_should_close(self.window)

    def get_inputs(self):
        glfw.poll_events()

    def clear_buffer(self):
        glClear(GL_COLOR_BUFFER_BIT)

    def get_time(self):
        return glfw.get_time()

    def swap_buffer(self):
        glfw.swap_buffers(self.window)

        # --- FPS counting / title update ---
        self._frame_count += 1
        now = glfw.get_time()
        elapsed = now - self._last_fps_time
        if elapsed >= self.fps_update_interval:
            self._fps = self._frame_count / elapsed if elapsed > 0 else 0.0
            fps_title = f"{self.title} - {self._fps:.1f} FPS"
            glfw.set_window_title(self.window, fps_title)

            # reset counters
            self._frame_count = 0
            self._last_fps_time = now

    def get_fps(self):
        return self._fps

    def destroy(self):
        glfw.terminate()
