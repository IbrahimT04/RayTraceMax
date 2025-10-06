from OpenGL.GL import *
import numpy as np
from game_object import TexturedQuad
from lighting import Light
from utilities import calc_compute_shaders, get_textures
from PIL import Image

class Skybox:
    def __init__(self, filepath = 'textures/skybox/kisspng_skybox2.png'):
        self.filepath = filepath

        self.texture = glGenTextures(1)

        glBindTexture(GL_TEXTURE_CUBE_MAP, self.texture)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

        with Image.open(f"{filepath}_left.png", mode="r") as image:
            image_width, image_height = image.size