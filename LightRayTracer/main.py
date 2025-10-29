from window import Window
from scene_renderer import SceneRenderer, Scene


class Main:
    def __init__(self):
        self.window = Window(1200,800,"My Window")
        self.sceneRenderer = SceneRenderer()

        self.sceneRenderer.input_info("window_info", self.window.get_window_info())
        self.sceneRenderer.prebake()
        self.sceneRenderer.scene = Scene(self.sceneRenderer, self.window)

    def main_loop(self):
        window = self.window
        scene = self.sceneRenderer
        while not window.get_status():
            window.get_inputs()

            window.clear_buffer()

            # scene.render()
            scene.ray_trace_render()

            window.swap_buffer()

    def destroy(self):
        self.sceneRenderer.destroy()
        self.window.destroy()


if __name__ == "__main__":
    main = Main()
    main.main_loop()
    main.destroy()
