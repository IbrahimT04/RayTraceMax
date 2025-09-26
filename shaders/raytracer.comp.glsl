#version 460

struct Sphere {
    vec3 center;
    float radius;
    vec3 color;
};

struct Camera {
    vec3 position;
    vec3 forwards;
    vec3 right;
    vec3 up;
};

struct Ray {
    vec3 origin;
    vec3 direction;
};
struct Plane {
    vec3 normal;
    vec3 color;
    float height;
};


//input/output
layout(local_size_x=8, local_size_y=8) in;
layout(rgba32f, binding = 0) uniform image2D img_output;

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

void main() {
    ivec2 pixel_coords = ivec2(gl_GlobalInvocationID);
    ivec2 screen_size = imageSize(img_output);
    float horizontalCoefficient = ((float(pixel_coords.x) * 2 - screen_size.x) / screen_size.x);
    float verticalCoefficient   = ((float(pixel_coords.y) * 2 - screen_size.y) / screen_size.x);

    vec3 pixel = vec3(0.0);

    Camera camera;
    camera.position = vec3(0.0);
    camera.forwards = vec3(1.0, 0.0, 0.0);
    camera.right    = vec3(0.0, 1.0, 0.0);
    camera.up       = vec3(0.0, 0.0, 1.0);

    Sphere sphere;
    sphere.center = vec3(5.0, -1.0, 0.0);
    sphere.radius = 1.0;
    sphere.color = vec3(1.0, 0.3, 0.7);

    Plane plane;
    plane.normal = vec3(0.0, 0.0, 1.0);
    plane.height = -1.0;
    plane.color = vec3(0.0, 0.0, 1.0);

    Ray ray;
    ray.origin = camera.position;
    ray.direction = camera.forwards + horizontalCoefficient * camera.right + verticalCoefficient * camera.up;

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
        pixel = vec3(0.0); // black
    }



    // Ray Trace Sphere
    // Quadratic Parameters x = (-b +- sqrt(b^2-4ac))/2a
    float a = dot(ray.direction, ray.direction);
    float b = 2.0 * dot(ray.direction, ray.origin - sphere.center);
    float c = dot(ray.origin - sphere.center, ray.origin - sphere.center) - sphere.radius * sphere.radius;

    float discriminant = b * b - 4 * a * c;

    if (discriminant > 0){
        pixel += sphere.color;
    }


    // Ray Trace Plane
    float denominator = dot(plane.normal, ray.direction);
    float t = (dot(plane.normal, ray.origin) + plane.height) / denominator;

    if (t > 0 && !isinf(t)){
        pixel += plane.color;
    }

    imageStore(img_output, pixel_coords, vec4(pixel, 1.0));
}
