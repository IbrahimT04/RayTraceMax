#version 460

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec2 a_texture;
//layout(location = 1) in vec3 a_color;

out vec2 fragTextCoord;
//out vec3 v_color;

void main() {
    gl_Position = vec4(a_position, 1.0);
    //v_color = a_color;
    fragTextCoord = a_texture;

}
