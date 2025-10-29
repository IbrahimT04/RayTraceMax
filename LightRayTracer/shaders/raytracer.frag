#version 460

in vec2 fragTextCoord;

uniform sampler2D framebuffer;
//in vec3 v_color;

out vec4 out_color;

void main() {
    out_color = texture(framebuffer, fragTextCoord);
}