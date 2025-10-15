#version 460

in vec2 fragTextCoord;
uniform sampler3D framebuffer;
out vec4 out_color;

void main() {
    ivec3 sz = textureSize(framebuffer, 0);
    ivec2 xy = clamp(ivec2(floor(fragTextCoord * vec2(sz.xy))), ivec2(0), sz.xy - 1);

    vec3 acc = vec3(0.0);
    for (int i = 0; i < sz.z; ++i) {
        acc += texelFetch(framebuffer, ivec3(xy, i), 0).xyz;
    }

    out_color = vec4(acc / float(sz.z), 1.0);
}
