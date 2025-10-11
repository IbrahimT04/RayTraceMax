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
    vec3 v0position;
    vec3 v1position;
    vec3 v2position;
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

layout(binding = 1) uniform samplerCube skybox;

layout(std430, binding = 2) readonly buffer sphereData{
    Sphere[] spheres;
};
uniform float sphere_count;

layout(std430, binding = 3) readonly buffer triangleData{
    Triangle[] triangles;
};
uniform float triangle_count;

layout(std430, binding = 4) readonly buffer planeData{
    Plane[] planes;
};
uniform float plane_count;

layout(std430, binding = 5) readonly buffer lightData{
    Light[] lights;
};
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
    for (int i = 0; i < triangle_count; i++){
        RenderState newRenderState = hit(ray, triangles[i], 0.001, nearestHit, renderState);


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
            renderstate.normal = normalize(cross(e1, e2));
            renderstate.t = t;
            renderstate.color = triangle.color;
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


/*

    Moved from main method for testing
    // ---------- Cube (AABB) ----------
    // Define cube center and half-size (a cube is an AABB here)
    vec3 cubeCenter = vec3(6.0, 3.0, 2.0);
    float halfSize = 1.0;
    vec3 bmin = cubeCenter - vec3(halfSize);
    vec3 bmax = cubeCenter + vec3(halfSize);

    float tHit;
    vec3 hitNormal;
    bool hit = intersectAABB(ray.origin, ray.direction, bmin, bmax, tHit, hitNormal);

    if (hit) {
        // Simple face coloring: map normal -> color so faces are visibly different
        // (normal is in {-1,0,1}, shifting to [0,1] gives distinct colors per face)
        pixel = hitNormal * 0.5 + 0.5;

        // Optional: simple Lambert-like shading with a fixed light direction to add depth
        vec3 lightDir = normalize(vec3(-1.0, -1.0, -0.5));
        float diff = max(0.0, dot(normalize(hitNormal), lightDir));
        float ambient = 0.2;
        pixel = pixel * (ambient + 0.8 * diff);
    } else {
        // background color when cube not hit
        pixel += vec3(0.0); // black
    }
    */


    /*
    // Ray Trace Sphere
    // Quadratic Parameters x = (-b +- sqrt(b^2-4ac))/2a
    float a = dot(ray.direction, ray.direction);
    float b = 2.0 * dot(ray.direction, ray.origin - sphere.center);
    float c = dot(ray.origin - sphere.center, ray.origin - sphere.center) - sphere.radius * sphere.radius;

    float discriminant = b * b - 4 * a * c;

    if (discriminant > 0){
        pixel += sphere.color;
    }

    */
    /*
    // Ray Trace Plane
    InfPlane plane;
    plane.normal = vec3(0.0, 0.0, 1.0);
    plane.center = vec3(0.0, 0.0,-6.0);
    plane.color = vec3(0.0, 0.0, 1.0);


    float denominator = dot(plane.normal, ray.direction);

    if (denominator != 0){
        float t = (dot(plane.normal, ray.origin)- plane.center[2]) / denominator;
        if (t < 0){
            pixel += plane.color;
        }
    }
    */