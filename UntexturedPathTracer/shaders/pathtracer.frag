#version 460

in vec2 fragTextCoord;

uniform sampler3D framebuffer;
//in vec3 v_color;

out vec4 out_color;

void main() {
    vec3 temp_color = vec3(0.0);
    for (int i=0; i < 128; i++) {
        temp_color += vec3(fragTextCoord, (i - 64.0)/64.0);
    }
    out_color = texture(framebuffer, temp_color/128.0);
    // out_color = vec4(1.0, 1.0, 0.0, 1.0);
}