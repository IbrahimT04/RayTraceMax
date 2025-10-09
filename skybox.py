from OpenGL.GL import *
from PIL import Image

class Skybox:
    def __init__(self, filepath = 'textures/skybox/skybox'):
        self.filepath = filepath

        glActiveTexture(GL_TEXTURE4)
        self.texture = glGenTextures(1)

        glBindTexture(GL_TEXTURE_CUBE_MAP, self.texture)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

        with Image.open(f"{self.filepath}_left.png", mode="r") as image:
            image_width, image_height = image.size
            image = image.convert('RGBA')
            imageData = bytes(image.tobytes())
            glTexImage2D( GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGBA8,
                          image_width, image_height,0, GL_RGBA, GL_UNSIGNED_BYTE, imageData
            )

        with Image.open(f"{self.filepath}_right.png", mode="r") as image:
            image_width, image_height = image.size
            image = image.convert('RGBA')
            image = image.rotate(180, expand = True)
            imageData = bytes(image.tobytes())
            glTexImage2D( GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGBA8,
                          image_width, image_height,0, GL_RGBA, GL_UNSIGNED_BYTE, imageData
            )


        with Image.open(f"{self.filepath}_back.png", mode="r") as image:
            image_width, image_height = image.size
            image = image.convert('RGBA')
            image = image.rotate(-90, expand=True)
            imageData = bytes(image.tobytes())
            glTexImage2D( GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGBA8,
                          image_width, image_height,0, GL_RGBA, GL_UNSIGNED_BYTE, imageData
            )


        with Image.open(f"{self.filepath}_front.png", mode="r") as image:
            image_width, image_height = image.size
            image = image.convert('RGBA')
            image = image.rotate(90, expand=True)
            imageData = bytes(image.tobytes())
            glTexImage2D( GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA8,
                          image_width, image_height,0, GL_RGBA, GL_UNSIGNED_BYTE, imageData
            )


        with Image.open(f"{self.filepath}_top.png", mode="r") as image:
            image_width, image_height = image.size
            image = image.convert('RGBA')
            image = image.rotate(90, expand=True)
            imageData = bytes(image.tobytes())
            glTexImage2D( GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGBA8,
                          image_width, image_height,0, GL_RGBA, GL_UNSIGNED_BYTE, imageData
            )


        with Image.open(f"{self.filepath}_bottom.png", mode="r") as image:
            image_width, image_height = image.size
            image = image.convert('RGBA')
            image = image.rotate(90, expand=True)
            imageData = bytes(image.tobytes())
            glTexImage2D( GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGBA8,
                          image_width, image_height,0, GL_RGBA, GL_UNSIGNED_BYTE, imageData
            )


    def use(self):
        glActiveTexture(GL_TEXTURE5)
        glBindTexture(GL_TEXTURE_CUBE_MAP, self.texture)

    def destroy(self):
        glDeleteTextures(1, (self.texture,))