from window import Window
from scene_renderer import SceneRenderer


class Main:
    def __init__(self):
        self.sceneRenderer = SceneRenderer()
        self.window = Window(self.sceneRenderer)

    def main_loop(self):
        window = self.window
        while not window.get_status():
            window.get_inputs()
            window.update_buffer()

    def destroy(self):
        self.sceneRenderer.destroy()
        self.window.destroy()


if __name__ == "__main__":
    main = Main()
    main.main_loop()
    main.destroy()
