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
    float metallic;
    vec3 normal;
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
layout(local_size_x=8, local_size_y=8, local_size_z = 8) in;
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

void reflect_ray(inout Ray ray, in RenderState renderState);

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
    trace(ray, renderState);
    first_pass(ray, pixel, renderState);
    next_pass(ray, pixel, renderState);

    ivec3 coord3 = ivec3(pixel_coords.x, pixel_coords.y, int(gl_GlobalInvocationID.z));
    imageStore(img_output, coord3, vec4(pixel, 1.0));
}

void first_pass(inout Ray ray, inout vec3 pixel, inout RenderState renderState){
    if (!renderState.hit){
        pixel = pixel * vec3(texture(skybox, ray.direction));
    }
    else {
        pixel = pixel * renderState.color;
        // Add reflection logic here
        ray.origin = renderState.position;
        reflect_ray(ray, renderState);
    }
}

void next_pass(Ray ray, inout vec3 pixel, RenderState renderState){
    for (int i = 0; i < 32; i++) {
        trace(ray, renderState);
        if (!renderState.hit){
            pixel = pixel * vec3(texture(skybox, ray.direction));
            break;
        }
        else {
            pixel = pixel * renderState.color;
            ray.origin = renderState.position;
            ray.direction = reflect(ray.direction, renderState.normal);
        }
    }
}

void reflect_ray(inout Ray ray, inout RenderState renderState) {
    PathSpreadVector planeSpread = paths[gl_GlobalInvocationID.z/2];
    /*
    float spread_x = (planeSpread.x_val) * (1.0-renderState.metallic);
    float spread_y = (planeSpread.y_val) * (1.0-renderState.metallic);
    vec3 spreadVector = normalize(vec3(spread_x, spread_y, 1.00001-renderState.metallic));
    // vec3 spreadVector = vec3(0.0, 0.0, 1.0);
    vec3 exact_dir = reflect(ray.direction, renderState.normal);
    vec4 quaternion = quatFromTo(vec3(0.0, 0.0, 1.0), renderState.normal);
    // vec4 quaternion = quatFromTo(vec3(0.0, 0.0, 1.0), exact_dir);

    ray.direction = rotateVecByQuat(spreadVector, quaternion);
    */
    /*
    vec3 I = normalize(ray.direction);
    vec3 N = normalize(renderState.normal);

    // Ensure N points against I (so reflect() behaves)
    N = faceforward(N, I, N);  // pick the variant so that dot(N, I) < 0

    vec3 R = normalize(reflect(I, N)); // mirror direction

    float U1 = planeSpread.x_val;
    float U2 = planeSpread.y_val;

    // half-angle of the cone (use ROUGHNESS, not metallic, to control width)
    float maxTheta = mix( 0.0, 0.5, (1.0 - renderState.metallic)); // 0.5 rad ≈ 29°
    float phi = 2.0 * 3.1415 * U1;
    float theta = maxTheta * U2;

    float s = sin(theta), c = cos(theta);
    vec3 local = vec3(cos(phi) * s, sin(phi) * s, c);

    // rotate +Z → R (same as above)
    vec4 q = quatFromTo(vec3(0,0,1), R);
    ray.direction = normalize(rotateVecByQuat(local, q));
    */
}

void trace(Ray ray, out RenderState renderState){

    float nearestHit = 9999999;
    //RenderState renderState;
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

    // return renderState;
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
            renderstate.normal = normalize(cross(e1, e2));
            renderstate.t = t;
            renderstate.color = triangle.color;
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
                renderstate.normal = plane.normal;
                renderstate.t = t;
                renderstate.color = plane.color;
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


/*
// --- helpers ---

// Cosine-weighted hemisphere sample (local space, +Z is the normal)
vec3 cosineSampleHemisphere(float u1, float u2) {
    float r   = sqrt(u1);
    float phi = 6.283185307179586 * u2; // 2π
    float x = r * cos(phi);
    float y = r * sin(phi);
    float z = sqrt(max(0.0, 1.0 - u1));
    return vec3(x, y, z);
}

// Build an orthonormal basis around N (Frisvad 2012)
void buildONB(in vec3 N, out vec3 T, out vec3 B) {
    if (N.z < -0.9999999) {
        T = vec3(0.0, -1.0, 0.0);
        B = vec3(-1.0,  0.0, 0.0);
    } else {
        float a = 1.0 / (1.0 + N.z);
        float b = -N.x * N.y * a;
        T = vec3(1.0 - N.x * N.x * a, b, -N.x);
        B = vec3(b, 1.0 - N.y * N.y * a, -N.y);
    }
}

// --- replacement ---

void reflect_ray(inout Ray ray, inout RenderState renderState) {
    // Normalize inputs
    vec3 I = normalize(ray.direction);
    vec3 N = normalize(renderState.normal);

    // Make sure N faces the incoming ray so we sample the visible hemisphere
    N = faceforward(N, I, N);

    // Two uniform randoms in [0,1) for cosine-weighted sampling
    PathSpreadVector s = paths[gl_GlobalInvocationID.z / 2];
    float u1 = s.x_val;
    float u2 = s.y_val;

    // Sample a local direction around +Z (cosine-weighted)
    vec3 local = cosineSampleHemisphere(u1, u2);

    // Build tangent/bitangent and map to world (+Z -> N)
    vec3 T, B;
    buildONB(N, T, B);
    vec3 worldDir = normalize(local.x * T + local.y * B + local.z * N);

    // Pure diffuse bounce
    ray.direction = worldDir;

    // Optional: nudge origin to avoid self-intersections
    // ray.origin += N * 1e-4;
}
*/