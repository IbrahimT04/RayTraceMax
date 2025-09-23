from OpenGL.GL.shaders import compileShader, compileProgram
from OpenGL.GL import GL_VERTEX_SHADER, GL_FRAGMENT_SHADER

def get_shaders(shader_program_name):
    with open(f'shaders/{shader_program_name}.vert') as file:
        vertex_shader = file.read()
    with open(f'shaders/{shader_program_name}.frag') as file:
        fragment_shader = file.read()

    return vertex_shader, fragment_shader


def get_textures(shader_program_name):
    with open(f'textures/{shader_program_name}.vert') as file:
        texture = file.read()
    return texture


def calc_shaders(shader_program_name):
    vertex_src, fragment_src = get_shaders(shader_program_name)
    shader = compileProgram(compileShader(vertex_src, GL_VERTEX_SHADER),
                            compileShader(fragment_src, GL_FRAGMENT_SHADER))
    return shader