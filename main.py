from window import Window
from scene_renderer import SceneRenderer, Scene


class Main:
    def __init__(self):
        self.window = Window()
        self.sceneRenderer = SceneRenderer()

        self.sceneRenderer.input_info("window_info", self.window.get_window_info())
        self.scene = Scene(self.sceneRenderer)

    def main_loop(self):
        window = self.window
        scene = self.sceneRenderer
        while not window.get_status():
            window.get_inputs()

            window.clear_buffer()

            scene.render()

            window.swap_buffer()

    def destroy(self):
        self.sceneRenderer.destroy()
        self.window.destroy()


if __name__ == "__main__":
    main = Main()
    main.main_loop()
    main.destroy()
