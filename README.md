# RayTraceMax

A compute‑shader PBR renderer that experiments with **real‑time ray tracing** and **stochastic path tracing** in **GLSL 4.6**, driven from Python via **PyOpenGL** and **GLFW**. The project explores a **compute shader pipeline** that writes directly to storage images, with geometry and materials streamed to the GPU through SSBOs.

> **Subprojects**
> - **SimpleRayTracer/** – minimal ray tracer (spheres/planes/triangles) used for debugging and feature bring‑up.
> - **LightRayTracer/** – ray tracing with skybox and reflective shading.
> - **UntexturedPathTracer/** – multi‑bounce path tracer (metallic/roughness PBR, skybox IBL, precomputed sample sets).

Apache 2.0 licensed (see `LICENSE`).

---

## ✨ Features

- **Compute Shader Pipeline (GLSL 4.6):** primary + secondary ray stages that write to an `image2D` output; reflective rays are passed via a staging texture (`reflection_input`) for subsequent passes.
- **Physically Based Rendering (PBR):** metallic/roughness parameters and per‑pixel normals (see `UntexturedPathTracer/shaders/pathtracer*.comp.glsl`).  
- **Environment Lighting / IBL:** skybox sampling to approximate global illumination (`textures/skybox/*`).
- **Geometry via SSBOs:** spheres, planes, and triangles are uploaded to shader storage buffers and iterated in the compute kernels.
- **Precomputed stochastic samples:** hemisphere/equirectangular sampling sets (`samples128.py`) for stable convergence and debug repeatability.
- **Interactive camera:** WASD + Shift (sprint), mouse‑look, Q/E & Space/Ctrl for vertical motion (see `*/window.py`, `*/camera.py`).

> **Notes:** The path tracer currently uses a fixed maximum bounce depth; **Russian roulette termination** can be added on top of this (see TODO).

---

## 📁 Repository Layout

```
RayTraceMax/
├── SimpleRayTracer/            # Minimal ray tracer
│   ├── main.py                 # Window loop and dispatch
│   ├── ray_object.py           # SSBO setup, dispatch, draw quad
│   └── shaders/                # raytracer.{vert,frag}, raytracer_test.comp.glsl
├── LightRayTracer/             # Ray tracing + light sources
│   ├── main.py
│   ├── scene_renderer.py
│   └── shaders/                # raytracer.comp.glsl, pass shaders
├── UntexturedPathTracer/       # Multi‑bounce path tracer (PBR/IBL)
│   ├── main.py
│   ├── scene_renderer.py
│   ├── ray_object.py
│   ├── samples128.py
│   └── shaders/                # pathtracer{_pre,_working}.comp.glsl, pass shaders
└── textures/
    └── skybox/                 # Cubemap faces
```

---

## 🚀 Getting Started

### 1) Requirements
- Python **3.10+**
- GPU/driver with **OpenGL 4.6** (compute shaders)
- OS: Windows / Linux / macOS (Apple GL 4.1 lacks compute; use Linux/Windows or MoltenVK + GL translation)

### 2) Install dependencies
```bash
# from repo root
python -m venv .venv && source .venv/bin/activate   # (Windows: .venv\Scripts\activate)
pip install --upgrade pip
pip install numpy PyOpenGL glfw Pillow pyrr
# optional speedups
pip install PyOpenGL-accelerate
```

### 3) Run a demo
```bash
# Simple ray tracer
python -m SimpleRayTracer.main

# Ray tracing with skybox/reflections
python -m LightRayTracer.main

# Path tracer (multi‑bounce, PBR/IBL)
python -m UntexturedPathTracer.main
```

> Run from the **repository root** so that relative shader and texture paths resolve.

---

## 🧠 How It Works (High‑Level)

- **CPU (Python)** creates a GLFW window and sets up camera state. Scene primitives (spheres, planes, triangles) and materials are packed into contiguous arrays and uploaded as **SSBOs**.  
- **GPU (Compute)** kernels (`*.comp.glsl`) build primary rays from the camera basis (`forwards/right/up`) and traverse scene geometry, writing radiance into an `image2D` output. In the path tracer, a first pass evaluates the primary hit and reflection direction; later passes accumulate **indirect lighting** (optionally sampling the **skybox** for IBL).  
- **PBR** shading uses `metallic`/`roughness` fields and per‑hit normals; the kernels can be extended with microfacet BRDFs (e.g., GGX/Schlick).  
- A full‑screen textured quad presents the storage image to the default framebuffer.

---

## ⚙️ Controls (default)

- **Move:** `W/A/S/D`, **Sprint:** `Shift`  
- **Vertical:** `Q` / `E` or `Space` / `Ctrl`  
- **Look:** Mouse  
- **Quit:** `Esc`  
(See `*/window.py` for exact bindings and FPS overlay.)

---

## 📌 Implementation Notes

- **Shaders:** `UntexturedPathTracer/shaders/pathtracer*.comp.glsl` and `LightRayTracer/shaders/raytracer*.comp.glsl`
- **Scene setup:** `*/scene_renderer.py` seeds spheres/planes/triangles and updates SSBOs.
- **Sampling:** `samples128.py` provides deterministic 2‑D sequences for hemisphere sampling.
- **Textures:** skybox cubemap under `textures/skybox/`.

---

## 🛣️ Roadmap / TODO

- Add **Russian roulette** termination for deep paths (reduces bias and improves perf).
- Introduce **BVH** acceleration structures for sub‑linear intersection.
- Add **textured** PBR (albedo/normal/metallic/roughness) and **area lights**.
- Support **multiple importance sampling (MIS)** and **temporal accumulation**.
- Integrate a lightweight **denoiser** (A‑Trous/OWD or hardware NR).

---

## 🧾 License

This project is licensed under the **Apache License 2.0**. See `LICENSE` for details.
