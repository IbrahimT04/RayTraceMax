from OpenGL.GL.shaders import compileShader, compileProgram
from OpenGL.GL import GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, GL_COMPUTE_SHADER
from PIL import Image


def get_shaders(shader_program_name):
    with open(f'shaders/{shader_program_name}.vert') as file:
        vertex_shader = file.read()
    with open(f'shaders/{shader_program_name}.frag') as file:
        fragment_shader = file.read()

    return vertex_shader, fragment_shader


def get_textures(texture="textures/trak_light2.jpg"):
    image = Image.open(texture)
    image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)

    image_data = image.convert('RGBA').tobytes()
    return image, image_data


def calc_shaders(shader_program_name):
    vertex_src, fragment_src = get_shaders(shader_program_name)
    shader = compileProgram(compileShader(vertex_src, GL_VERTEX_SHADER),
                            compileShader(fragment_src, GL_FRAGMENT_SHADER))
    return shader

def calc_compute_shaders(shader_program_name):
    with open(f'shaders/{shader_program_name}.comp.glsl') as file:
        compute_src = file.read()
    comp_shader = compileProgram(compileShader(compute_src, GL_COMPUTE_SHADER))
    return comp_shader