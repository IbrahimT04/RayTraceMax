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
