#version 460

struct Camera {
    vec3 position;
    vec3 forwards;
    vec3 right;
    vec3 up;
};

struct Light {
    vec3 position;
    vec3 color;
    float strenght;
};

struct Ray {
    vec3 origin;
    vec3 direction;
};

struct RenderState {
    float t;
    vec3 diffuse;
    bool hit;
    vec3 position;
    float metallic;
    vec3 emissive;
    vec3 normal;
    vec3 in_vec;
    vec3 out_vec;
};

struct PathSpreadVector {
    float x_val;
    float y_val;
};

struct Triangle {
    vec3 v0position;
    vec3 v1position;
    vec3 v2position;
    vec3 color;
    float metallic;
    vec3 emissive;
};

struct Sphere {
    vec3 center;
    float radius;
    vec3 color;
    float metallic;
    vec3 emissive;
};

struct Plane {
    vec3 center;
    float uMin;
    vec3 tangent;
    float uMax;
    vec3 bitangent;
    float vMin;
    vec3 normal;
    float vMax;
    vec3 color;
    float metallic;
    vec3 emissive;
};

struct InfPlane {
    vec3 normal;
    vec3 color;
    vec3 center;
    vec3 emissive;
};


//input/output
layout(local_size_x=8, local_size_y=8, local_size_z = 4) in;

// Scene input data
uniform Camera viewer;

layout(binding = 1) uniform samplerCube skybox;

layout(std430, binding = 2) readonly buffer sphereData {
    Sphere[] spheres;
};
uniform float sphere_count;

layout(std430, binding = 3) readonly buffer triangleData {
    Triangle[] triangles;
};
uniform float triangle_count;

layout(std430, binding = 4) readonly buffer planeData {
    Plane[] planes;
};
uniform float plane_count;

layout(rgba32f, binding = 7) uniform image2D ray_output;

uniform vec3 sun_pos = vec3(100.0);
uniform float sun_radius = 10.0;

void trace(Ray ray, out RenderState renderState);

RenderState hit(Ray ray, Sphere sphere, float tMin, float tMax, RenderState renderstate);
RenderState hit(Ray ray, Triangle triangle, float tMin, float tMax, RenderState renderstate);
RenderState hit(Ray ray, Plane plane, float tMin, float tMax, RenderState renderstate);

void ray_traversal(Ray ray, inout vec3 pixel, RenderState renderState);

void main() {
    ivec2 pixel_coords = ivec2(gl_GlobalInvocationID.xy);
    ivec2 screen_size = imageSize(ray_output);
    float horizontalCoefficient = ((float(pixel_coords.x) * 2 - screen_size.x) / screen_size.x);
    float verticalCoefficient   = ((float(pixel_coords.y) * 2 - screen_size.y) / screen_size.x);

    Ray ray;
    ray.origin = viewer.position;
    ray.direction = viewer.forwards + horizontalCoefficient * viewer.right + verticalCoefficient * viewer.up;

    vec3 pixel = vec3(1.0);

    RenderState renderState;
    ray_traversal(ray, pixel, renderState);

    ivec2 coord3 = ivec2(pixel_coords.x, pixel_coords.y);
    imageStore(ray_output, coord3, vec4(pixel, 1.0));
}

void trace(Ray ray, out RenderState renderState){

    float nearestHit = 9999999;
    renderState.hit = false;

    for (int i = 0; i < sphere_count; i++) {
        RenderState newRenderState = hit(ray, spheres[i], 0.001, nearestHit, renderState);


        if (newRenderState.hit){
            nearestHit = newRenderState.t;
            renderState = newRenderState;
        }
    }
    for (int i = 0; i < triangle_count; i++) {
        RenderState newRenderState = hit(ray, triangles[i], 0.001, nearestHit, renderState);


        if (newRenderState.hit){
            nearestHit = newRenderState.t;
            renderState = newRenderState;
        }
    }

    for (int i = 0; i < plane_count; i++) {
        RenderState newRenderState = hit(ray, planes[i], 0.001, nearestHit, renderState);

        if (newRenderState.hit){
            nearestHit = newRenderState.t;
            renderState = newRenderState;
        }
    }
    renderState.in_vec = normalize(-ray.direction);
}

void ray_traversal(Ray ray, inout vec3 pixel, RenderState renderState){
    for (int i = 0; i < 32; i++) {
        trace(ray, renderState);
        if (!renderState.hit){
            vec3 sky_text = vec3(texture(skybox, ray.direction));
            pixel = pixel * sky_text;
            break;
        }
        else {
            ray.origin = renderState.position;
            ray.direction = reflect(ray.direction, renderState.normal);
            renderState.out_vec = ray.direction;
            vec3 diffuse = (renderState.diffuse/3.1415) * dot(renderState.out_vec, renderState.normal);
            pixel = renderState.emissive + (1.0 - renderState.metallic) * (pixel * renderState.diffuse);
        }
    }
}

RenderState hit(Ray ray, Sphere sphere, float tMin, float tMax, RenderState renderstate){
    vec3 dist = ray.origin - sphere.center;
    float a = dot(ray.direction, ray.direction);
    float b = 2.0 * dot(ray.direction, dist);
    float c = dot(dist, dist) - sphere.radius * sphere.radius;

    float determinant = b * b - 4 * a * c;

    if (determinant > 0){
        float t = (-b - sqrt(determinant)) / (2 * a);

        if (t > tMin && t < tMax){

            renderstate.position = ray.origin + t * ray.direction;
            renderstate.normal = normalize(renderstate.position - sphere.center);
            renderstate.t = t;
            renderstate.diffuse = sphere.color;
            renderstate.emissive = sphere.emissive;
            renderstate.metallic = sphere.metallic;
            renderstate.hit = true;
            return renderstate;
        }
        return renderstate;
    }
    else {
        renderstate.hit = false;
        return renderstate;
    }
}

RenderState hit(Ray ray, Triangle triangle, float tMin, float tMax, RenderState renderstate){
    vec3 e1 = triangle.v1position - triangle.v0position;
    vec3 e2 = triangle.v2position - triangle.v0position;

    vec3 p0 = cross(ray.direction, e2);
    float det = dot(p0, e1);

    if (det != 0.0) {

        vec3 t0 = ray.origin - triangle.v0position;
        vec3 q = cross(t0, e1);

        float u = dot(t0, p0)/det;
        float v = dot(q, ray.direction)/det;

        float t = dot(e2, q)/det;

        if (u >= 0 && v >= 0 && u + v <= 1 && t > tMin && t < tMax) {

            renderstate.position = t * ray.direction + ray.origin;
            renderstate.normal = normalize(cross(e1, e2)) * det/abs(det);
            renderstate.t = t;
            renderstate.diffuse = triangle.color;
            renderstate.emissive = triangle.emissive;
            renderstate.metallic = triangle.metallic;
            renderstate.hit = true;
            return renderstate;
        }
    }

    renderstate.hit = false;

    return renderstate;
}

RenderState hit(Ray ray, Plane plane, float tMin, float tMax, RenderState renderstate){
    float denominator = dot(plane.normal, ray.direction);

    if (abs(denominator) > 0.00001) {

        float t = dot(plane.center - ray.origin, plane.normal) / denominator;

        if (t > tMin && t < tMax) {

            vec3 point = ray.origin + t * ray.direction;
            vec3 plane_dir = point - plane.center;

            float u_vec = dot(plane_dir, plane.tangent);
            float v_vec = dot(plane_dir, plane.bitangent);

            if (u_vec > plane.uMin && u_vec < plane.uMax && v_vec > plane.vMin && v_vec < plane.vMax) {
                renderstate.position = point;
                renderstate.normal = normalize(plane.normal * -denominator/abs(denominator));
                renderstate.t = t;
                renderstate.diffuse = plane.color;
                renderstate.emissive = plane.emissive;
                renderstate.metallic = plane.metallic;
                renderstate.hit = true;
                return renderstate;
            }
        }
    }

    renderstate.hit = false;

    return renderstate;
}
