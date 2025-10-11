from OpenGL.GL import *
import glfw


class Window:
    def __init__(self, width: int = 1800, height: int = 1200, title: str = "My Window"):
        self.camera = None
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
        glfw.set_input_mode(self.window, glfw.CURSOR, glfw.CURSOR_DISABLED)

        self.update_context(self.window)

        glClearColor(0.3, 0.2, 0.3, 0.0)

        # FPS tracking
        self._frame_count = 0
        self._last_fps_time = glfw.get_time()
        self._fps = 0.0
        # Dynamic fps update control
        self.fps_update_interval = 0.5

        # Movement
        self._last_cursor = glfw.get_cursor_pos(self.window)

        self.mouse_dx = 0.0
        self.mouse_dy = 0.0

    def window_resize(self, window, width, height):
        fb_w, fb_h = glfw.get_framebuffer_size(window)
        glViewport(0, 0, fb_w, fb_h)
        if window == self.window:
            self.width = width
            self.height = height

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
        cx, cy = glfw.get_cursor_pos(self.window)
        lx, ly = self._last_cursor
        dx = cx - lx
        dy = cy - ly
        self._last_cursor = (cx, cy)
        self.mouse_dx = dx
        self.mouse_dy = dy

        if glfw.get_key(self.window, glfw.KEY_ESCAPE) == glfw.PRESS:
            glfw.set_window_should_close(self.window, True)

        # Calculate Camera updates
        self._handle_camera()

    def attach_camera(self, cam):
        self.camera = cam

    def _handle_camera(self):
        cam = self.camera
        if cam is None:
            self.mouse_dx = 0.0
            self.mouse_dy = 0.0
            self._last_input_time = self.get_time()
            return

        now = self.get_time()
        last = getattr(self, "_last_input_time", now)
        dt = now - last
        self._last_input_time = now
        sprint = (glfw.get_key(self.window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS) or (
                glfw.get_key(self.window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)

        forwards = (glfw.get_key(self.window, glfw.KEY_W) == glfw.PRESS) or (
                glfw.get_key(self.window, glfw.KEY_UP) == glfw.PRESS)

        backwards = (glfw.get_key(self.window, glfw.KEY_S) == glfw.PRESS) or (
                glfw.get_key(self.window, glfw.KEY_DOWN) == glfw.PRESS)

        right = (glfw.get_key(self.window, glfw.KEY_D) == glfw.PRESS) or (
                glfw.get_key(self.window, glfw.KEY_RIGHT) == glfw.PRESS)

        left = (glfw.get_key(self.window, glfw.KEY_A) == glfw.PRESS) or (
                glfw.get_key(self.window, glfw.KEY_LEFT) == glfw.PRESS)

        up = (glfw.get_key(self.window, glfw.KEY_SPACE) == glfw.PRESS) or (
                glfw.get_key(self.window, glfw.KEY_E) == glfw.PRESS)

        down = (glfw.get_key(self.window, glfw.KEY_LEFT_CONTROL) == glfw.PRESS) or (
                glfw.get_key(self.window, glfw.KEY_Q) == glfw.PRESS)

        forward_dir = (1.0 if forwards else 0.0) - (1.0 if backwards else 0.0)
        right_dir = (1.0 if right else 0.0) - (1.0 if left else 0.0)
        up_dir = (1.0 if up else 0.0) - (1.0 if down else 0.0)

        cam.apply_movement(forward_dir, right_dir, up_dir, dt, sprint)
        dx = self.mouse_dx
        dy = self.mouse_dy
        if dx != 0.0 or dy != 0.0:
            cam.apply_rotation_from_mouse(dx, dy)

        self.mouse_dx = 0.0
        self.mouse_dy = 0.0

    def clear_buffer(self):
        glClear(GL_COLOR_BUFFER_BIT)

    def get_time(self):
        return glfw.get_time()

    def swap_buffer(self):
        glfw.swap_buffers(self.window)

        # frame rate and title update
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
