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
    vec3 color;
    bool hit;
    vec3 position;
    vec3 normal;
};

struct Triangle {
    vec3 position;
    vec3 color;
};

struct Sphere {
    vec3 center;
    float radius;
    vec3 color;
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
};

struct InfPlane {
    vec3 normal;
    vec3 color;
    vec3 center;
};


//input/output
layout(local_size_x=8, local_size_y=8) in;
layout(rgba32f, binding = 0) uniform image2D img_output;

// Scene input data
uniform Camera viewer;

layout(std430, binding = 1) readonly buffer sphereData{
    Sphere[] spheres;
};
uniform float sphere_count;

layout(std430, binding = 2) readonly buffer triangleData{
    Triangle[] triangles;
};
uniform float triangle_count;

layout(std430, binding = 3) readonly buffer planeData{
    Plane[] planes;
};
uniform float plane_count;

layout(std430, binding = 4) readonly buffer lightData{
    Light[] lights;
};
layout(rgba32f, binding = 5) uniform samplerCube skybox;

uniform float light_count;

// AABB (slab) intersection. Returns true if hit; outputs tHit and hit normal.
bool intersectAABB(in vec3 ro, in vec3 rd, in vec3 bmin, in vec3 bmax, out float tHit, out vec3 outNormal);

RenderState trace(Ray ray);

RenderState hit(Ray ray, Sphere sphere, float tMin, float tMax, RenderState renderstate);
RenderState hit(Ray ray, Triangle triangle, float tMin, float tMax, RenderState renderstate);
RenderState hit(Ray ray, Plane plane, float tMin, float tMax, RenderState renderstate);

float distanceTo(Ray ray, Sphere sphere);
float distanceTo(Ray ray, Triangle triangle);
float distanceTo(Ray ray, Plane plane);

vec3 light_fragement(RenderState renderState);


void main() {
    ivec2 pixel_coords = ivec2(gl_GlobalInvocationID.xy);
    ivec2 screen_size = imageSize(img_output);
    float horizontalCoefficient = ((float(pixel_coords.x) * 2 - screen_size.x) / screen_size.x);
    float verticalCoefficient   = ((float(pixel_coords.y) * 2 - screen_size.y) / screen_size.x);

    Ray ray;
    ray.origin = viewer.position;
    ray.direction = viewer.forwards + horizontalCoefficient * viewer.right + verticalCoefficient * viewer.up;

    // vec3 pixel = trace(ray);
    vec3 pixel = vec3(1.0);
    // pixel = vec3(texture(skybox, ray.direction));

    for (int i = 0; i < 32; i++){

        RenderState renderState = trace(ray);
        if (!renderState.hit){
            pixel = pixel * vec3(texture(skybox, ray.direction));
            break;
        }
        pixel = pixel * renderState.color;

        ray.origin = renderState.position;
        ray.direction = reflect(ray.direction, renderState.normal);
    }

    imageStore(img_output, pixel_coords, vec4(pixel, 1.0));
}

RenderState trace(Ray ray){

    float nearestHit = 9999999;
    RenderState renderState;
    renderState.hit = false;

    for (int i = 0; i < sphere_count; i++){
        RenderState newRenderState = hit(ray, spheres[i], 0.001, nearestHit, renderState);


        if (newRenderState.hit){
            nearestHit = newRenderState.t;
            renderState = newRenderState;
        }
    }

    for (int i = 0; i < plane_count; i++){
        RenderState newRenderState = hit(ray, planes[i], 0.001, nearestHit, renderState);

        if (newRenderState.hit){
            nearestHit = newRenderState.t;
            renderState = newRenderState;
        }
    }

    return renderState;
}

RenderState hit(Ray ray, Sphere sphere, float tMin, float tMax, RenderState renderstate){
    vec3 dist = ray.origin - sphere.center;
    float a = dot(ray.direction, ray.direction);
    float b = 2.0 * dot(ray.direction, dist);
    float c = dot(dist, dist) - sphere.radius * sphere.radius;

    float discriminant = b * b - 4 * a * c;

    if (discriminant > 0){
        float t = (-b - sqrt(discriminant)) / (2 * a);

        if (t > tMin && t < tMax){

            renderstate.position = ray.origin + t * ray.direction;
            renderstate.normal = normalize(renderstate.position - sphere.center);
            renderstate.t = t;
            renderstate.color = sphere.color;
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
                renderstate.normal = plane.normal;
                renderstate.t = t;
                renderstate.color = plane.color;
                renderstate.hit = true;
                return renderstate;
            }
        }
    }

    renderstate.hit = false;
    return renderstate;
}

// AABB (slab) intersection. Returns true if hit; outputs tHit and hit normal.
bool intersectAABB(in vec3 ro, in vec3 rd, in vec3 bmin, in vec3 bmax, out float tHit, out vec3 outNormal) {
    // Avoid divisions by zero by using a large number when rd component is nearly zero
    vec3 safeRd = rd;
    const float EPS_ZERO = 1e-8;
    safeRd.x = (abs(safeRd.x) < EPS_ZERO) ? sign(safeRd.x + EPS_ZERO) * EPS_ZERO : safeRd.x;
    safeRd.y = (abs(safeRd.y) < EPS_ZERO) ? sign(safeRd.y + EPS_ZERO) * EPS_ZERO : safeRd.y;
    safeRd.z = (abs(safeRd.z) < EPS_ZERO) ? sign(safeRd.z + EPS_ZERO) * EPS_ZERO : safeRd.z;

    vec3 invD = vec3(1.0) / safeRd;

    vec3 t1 = (bmin - ro) * invD;
    vec3 t2 = (bmax - ro) * invD;

    vec3 tmin = min(t1, t2);
    vec3 tmax = max(t1, t2);

    float tEnter = max(max(tmin.x, tmin.y), tmin.z);
    float tExit  = min(min(tmax.x, tmax.y), tmax.z);

    // no intersection or box behind ray
    if (tEnter > tExit) return false;
    if (tExit < 0.0) return false;

    tHit = (tEnter >= 0.0) ? tEnter : tExit;

    // compute hit point and robustly determine face normal by comparing coordinates
    vec3 hitP = ro + rd * tHit;
    const float EPS_FACE = 1e-4;

    if (abs(hitP.x - bmin.x) < EPS_FACE) outNormal = vec3(-1.0, 0.0, 0.0);
    else if (abs(hitP.x - bmax.x) < EPS_FACE) outNormal = vec3( 1.0, 0.0, 0.0);
    else if (abs(hitP.y - bmin.y) < EPS_FACE) outNormal = vec3(0.0, -1.0, 0.0);
    else if (abs(hitP.y - bmax.y) < EPS_FACE) outNormal = vec3(0.0,  1.0, 0.0);
    else if (abs(hitP.z - bmin.z) < EPS_FACE) outNormal = vec3(0.0, 0.0, -1.0);
    else if (abs(hitP.z - bmax.z) < EPS_FACE) outNormal = vec3(0.0, 0.0,  1.0);
    else outNormal = vec3(0.0); // fallback (shouldn't usually happen)

    return true;
}