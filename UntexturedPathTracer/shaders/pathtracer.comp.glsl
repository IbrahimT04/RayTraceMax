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
};

struct Sphere {
    vec3 center;
    float radius;
    vec3 color;
    float metallic;
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
};

struct InfPlane {
    vec3 normal;
    vec3 color;
    vec3 center;
};


//input/output
layout(local_size_x=8, local_size_y=8, local_size_z = 4) in;
layout(rgba32f, binding = 0) uniform image3D img_output;

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

layout(std430, binding = 5) readonly buffer Data {
    PathSpreadVector[] paths;
};
uniform float path_spread_length;

// AABB (slab) intersection. Returns true if hit; outputs tHit and hit normal.
// bool intersectAABB(in vec3 ro, in vec3 rd, in vec3 bmin, in vec3 bmax, out float tHit, out vec3 outNormal);

void trace(Ray ray, out RenderState renderState);

RenderState hit(Ray ray, Sphere sphere, float tMin, float tMax, RenderState renderstate);
RenderState hit(Ray ray, Triangle triangle, float tMin, float tMax, RenderState renderstate);
RenderState hit(Ray ray, Plane plane, float tMin, float tMax, RenderState renderstate);

void reflect_ray(inout Ray ray, inout RenderState renderState);

void first_pass(inout Ray ray, inout vec3 pixel, inout RenderState renderState);

void next_pass(Ray ray, inout vec3 pixel, RenderState renderState);

vec4 quatFromTo(vec3 a_in, vec3 b_in);
vec3 rotateVecByQuat(vec3 v, vec4 q);

void main() {
    ivec2 pixel_coords = ivec2(gl_GlobalInvocationID.xy);
    ivec3 screen_size = imageSize(img_output);
    float horizontalCoefficient = ((float(pixel_coords.x) * 2 - screen_size.x) / screen_size.x);
    float verticalCoefficient   = ((float(pixel_coords.y) * 2 - screen_size.y) / screen_size.x);

    Ray ray;
    ray.origin = viewer.position;
    ray.direction = viewer.forwards + horizontalCoefficient * viewer.right + verticalCoefficient * viewer.up;

    // vec3 pixel = trace(ray);
    vec3 pixel = vec3(1.0);
    // pixel = vec3(texture(skybox, ray.direction));

    RenderState renderState;
    //renderState.diffuse = vec3(1.0);
    trace(ray, renderState);
    first_pass(ray, pixel, renderState);
    if (renderState.hit){
        next_pass(ray, pixel, renderState);
    }

    ivec3 coord3 = ivec3(pixel_coords.x, pixel_coords.y, int(gl_GlobalInvocationID.z));
    imageStore(img_output, coord3, vec4(pixel, 1.0));
}

void trace(Ray ray, out RenderState renderState){

    float nearestHit = 9999999;
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
    renderState.in_vec = normalize(-ray.direction);
}

void first_pass(inout Ray ray, inout vec3 pixel, inout RenderState renderState){
    if (!renderState.hit){
        vec3 sky_text = vec3(texture(skybox, ray.direction));
        pixel = pixel * sky_text;
    }
    else {
        ray.origin = renderState.position;
        reflect_ray(ray, renderState);
        renderState.out_vec = normalize(ray.direction);
        pixel = pixel * renderState.diffuse * dot(renderState.out_vec, renderState.normal);
        // pixel = pixel * renderState.diffuse;
    }
}

void next_pass(Ray ray, inout vec3 pixel, RenderState renderState){
    for (int i = 0; i < 32; i++) {
        trace(ray, renderState);
        if (!renderState.hit){
            vec3 sky_text = vec3(texture(skybox, ray.direction));
            pixel = pixel * sky_text;
            // pixel = pixel * (1/3.14159 * 10.0) * tan(3.14159 * (length(sky_text)/2)) * sky_text;
            // pixel = pixel * sky_text * 1.0/5.0 * (pow(7, length(sky_text)) - 1) * sky_text;
            break;
        }
        else {
            // pixel = pixel * renderState.diffuse;
            ray.origin = renderState.position;
            ray.direction = reflect(ray.direction, renderState.normal);
            renderState.out_vec = ray.direction;
            pixel = pixel * renderState.diffuse * dot(renderState.out_vec, renderState.normal);
        }
    }
}

void reflect_ray(inout Ray ray, inout RenderState renderState) {
    PathSpreadVector planeSpread = paths[gl_GlobalInvocationID.z/2];
    float cos_theta = planeSpread.x_val;
    float sin_theta = sqrt(1.0 - cos_theta * cos_theta);
    float cos_phi = cos(2.0 * 3.14159 * planeSpread.y_val);
    float sin_phi = sin(2.0 * 3.14159 * planeSpread.y_val);
    vec3 spreadVector = vec3(cos_phi * sin_theta, sin_phi * sin_theta, cos_theta);
    vec4 quaternion = quatFromTo(vec3(0.0, 0.0, 1.0), renderState.normal);

    ray.direction = rotateVecByQuat(spreadVector, quaternion);
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
            renderstate.diffuse = sphere.color; // * dot(renderstate.normal, -ray.direction);
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
            renderstate.diffuse = triangle.color; // * dot(renderstate.normal, -ray.direction);
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
                renderstate.diffuse = plane.color; // * abs(dot(renderstate.normal, -ray.direction));
                renderstate.metallic = plane.metallic;
                renderstate.hit = true;
                return renderstate;
            }
        }
    }

    renderstate.hit = false;

    return renderstate;
}

const float EPS_LEN = 1e-6;
const float EPS_DOT = 1e-7;



vec4 quatFromTo(vec3 a_in, vec3 b_in) {
    // early length checks (avoid dividing by zero inside normalize)
    float la = length(a_in);
    float lb = length(b_in);
    if (la < EPS_LEN || lb < EPS_LEN) {
        return vec4(0.0, 0.0, 0.0, 1.0); // identity
    }

    vec3 a = a_in / la;
    vec3 b = b_in / lb;

    float d = dot(a, b);

    // nearly identical -> identity rotation
    if (d > 1.0 - EPS_DOT) {
        return vec4(0.0, 0.0, 0.0, 1.0);
    }

    // nearly opposite -> 180 degree rotation about some orthogonal axis
    if (d < -1.0 + EPS_DOT) {
        vec3 ortho = (abs(a.x) < 0.6) ? vec3(1.0, 0.0, 0.0) : vec3(0.0, 1.0, 0.0);
        vec3 axis = normalize(cross(a, ortho));
        return vec4(axis, 0.0); // w = 0 => 180°
    }

    // general case -> build quaternion then normalize
    vec3 v = cross(a, b);
    vec4 q = vec4(v, 1.0 + d);

    // normalize using inversesqrt, sometimes cheaper
    float invLen = inversesqrt(dot(q, q));
    q *= invLen;

    return q;
}

// rotate vector v by unit quaternion q (q must be normalized)
vec3 rotateVecByQuat(vec3 v, vec4 q) {
    vec3 qv = q.xyz;
    float qw = q.w;
    vec3 t = 2.0 * cross(qv, v);
    return v + qw * t + cross(qv, t);
}
