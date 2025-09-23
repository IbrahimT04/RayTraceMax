from window import Window
from scene_renderer import SceneRenderer


class Main:
    def __init__(self):
        self.window = Window()
        self.sceneRenderer = SceneRenderer()

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
