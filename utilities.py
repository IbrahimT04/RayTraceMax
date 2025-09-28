from typing import Dict, List, Tuple, Optional

import numpy as np
from OpenGL.GL.shaders import compileShader, compileProgram
from OpenGL.GL import GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, GL_COMPUTE_SHADER
from PIL import Image


def get_shaders(shader_program_name):
    with open(f'shaders/{shader_program_name}.vert') as file:
        vertex_shader = file.read()
    with open(f'shaders/{shader_program_name}.frag') as file:
        fragment_shader = file.read()

    return vertex_shader, fragment_shader


def get_textures(texture="textures/trak_light2.jpg"):
    image = Image.open(texture)
    image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)

    image_data = image.convert('RGBA').tobytes()
    return image, image_data


def calc_shaders(shader_program_name):
    vertex_src, fragment_src = get_shaders(shader_program_name)
    shader = compileProgram(compileShader(vertex_src, GL_VERTEX_SHADER),
                            compileShader(fragment_src, GL_FRAGMENT_SHADER))
    return shader

def calc_compute_shaders(shader_program_name):
    with open(f'shaders/{shader_program_name}.comp.glsl') as file:
        compute_src = file.read()
    comp_shader = compileProgram(compileShader(compute_src, GL_COMPUTE_SHADER))
    return comp_shader

def load_object_files(object_name):
    import numpy as np
    from typing import Dict, Optional, Tuple, List

class ObjLoader:
    """
    Minimal OBJ loader that produces interleaved vertex data and an index array
    suitable for glDrawElements.

    Usage:
        out = ObjLoader.load_obj("model.obj")
        vertices = out['vertices']   # float32 interleaved: pos(3), normal(3), tex(2)
        indices  = out['indices']    # uint32
        has_normals = out['has_normals']
        has_texcoords = out['has_texcoords']
        # stride = vertices.strides[0] or 8 * 4 bytes = 32 bytes
    """

    @staticmethod
    def load_obj(path: str) -> Dict[str, object]:
        # Raw lists from file
        positions: List[Tuple[float, float, float]] = []
        texcoords: List[Tuple[float, float]] = []
        normals: List[Tuple[float, float, float]] = []

        # Final per-vertex unique mapping -> index
        vertex_map: Dict[Tuple[int, Optional[int], Optional[int]], int] = {}
        vertices_list: List[Tuple[float, ...]] = []
        indices_list: List[int] = []

        # Helper to parse an index that may be negative
        def fix_index(raw_idx: int, collection_len: int) -> int:
            return raw_idx - 1 if raw_idx > 0 else collection_len + raw_idx

        with open(path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                if not line or line[0] == '#':
                    continue
                parts = line.strip().split()
                if not parts:
                    continue
                tag = parts[0]
                if tag == 'v' and len(parts) >= 4:
                    positions.append((float(parts[1]), float(parts[2]), float(parts[3])))
                elif tag == 'vt' and len(parts) >= 3:
                    # keep only u,v (ignore possible w)
                    texcoords.append((float(parts[1]), float(parts[2])))
                elif tag == 'vn' and len(parts) >= 4:
                    normals.append((float(parts[1]), float(parts[2]), float(parts[3])))
                elif tag == 'f' and len(parts) >= 4:
                    # face entries: v, v/vt, v//vn, or v/vt/vn
                    face_verts = []
                    for vert in parts[1:]:
                        # split by '/'
                        comps = vert.split('/')
                        vi = int(comps[0])
                        vti = int(comps[1]) if len(comps) >= 2 and comps[1] != '' else None
                        vni = int(comps[2]) if len(comps) >= 3 and comps[2] != '' else None
                        face_verts.append((vi, vti, vni))

                    # Triangulate polygon with triangle fan
                    for i in range(1, len(face_verts) - 1):
                        tri = (face_verts[0], face_verts[i], face_verts[i + 1])
                        for vi_raw, vti_raw, vni_raw in tri:
                            vi = fix_index(vi_raw, len(positions))
                            vti = fix_index(vti_raw, len(texcoords)) if vti_raw is not None else None
                            vni = fix_index(vni_raw, len(normals)) if vni_raw is not None else None
                            key = (vi, vti, vni)
                            if key not in vertex_map:
                                vertex_map[key] = len(vertices_list)
                                # store indices for now as placeholders; actual arrays will be built later
                                vertices_list.append(key)  # temporarily store key; will expand later
                            indices_list.append(vertex_map[key])

        # Determine presence of attributes
        has_tex = any(k[1] is not None for k in vertex_map.keys())
        has_nrm = any(k[2] is not None for k in vertex_map.keys())

        # Build real interleaved vertex array: pos(3), normal(3), tex(2)
        # If normals or texcoords missing, fill zeros for now.
        vertex_count = len(vertices_list)
        interleaved = np.zeros((vertex_count, 8), dtype=np.float32)  # 3+3+2 = 8 floats

        # Fill positions, texcoords, normals (if available)
        for idx, key in enumerate(vertices_list):
            vi, vti, vni = key
            px, py, pz = positions[vi]
            interleaved[idx, 0:3] = (px, py, pz)
            if has_nrm and vni is not None:
                nx, ny, nz = normals[vni]
                interleaved[idx, 3:6] = (nx, ny, nz)
            # else leave zeros for normals (we may compute them later)

            if has_tex and vti is not None:
                u, v = texcoords[vti]
                interleaved[idx, 6:8] = (u, v)
            # else leave zeros for texcoords

        indices = np.array(indices_list, dtype=np.uint32)

        # If normals are missing, compute smooth normals from triangles
        if not has_nrm:
            # accumulate per-vertex normals
            accum = np.zeros((vertex_count, 3), dtype=np.float64)
            for i in range(0, len(indices), 3):
                ia, ib, ic = indices[i], indices[i + 1], indices[i + 2]
                pa = interleaved[ia, 0:3].astype(np.float64)
                pb = interleaved[ib, 0:3].astype(np.float64)
                pc = interleaved[ic, 0:3].astype(np.float64)
                # face normal
                ab = pb - pa
                ac = pc - pa
                fn = np.cross(ab, ac)
                norm = np.linalg.norm(fn)
                if norm != 0.0:
                    fn = fn / norm
                else:
                    fn = np.array((0.0, 0.0, 0.0))
                accum[ia] += fn
                accum[ib] += fn
                accum[ic] += fn
            # normalize accumulated normals and place into interleaved
            for vi in range(vertex_count):
                n = accum[vi]
                nlen = np.linalg.norm(n)
                if nlen > 0.0:
                    n = (n / nlen).astype(np.float32)
                else:
                    n = np.array((0.0, 0.0, 1.0), dtype=np.float32)
                interleaved[vi, 3:6] = n

        # Final arrays
        # Optionally, you might want to reshape interleaved to a flat 1D float32 array for uploading
        vertices_flat = interleaved.reshape(-1).astype(np.float32)

        return {
            'vertices': vertices_flat,  # 1D float32 array: [pos3,norm3,uv2] * N
            'vertex_count': vertex_count,
            'stride_floats': 8,
            'indices': indices,  # uint32 array
            'has_normals': True,
            'has_texcoords': has_tex,
            'interleaved_per_vertex': interleaved  # 2D array shape (N, 8) for convenience
        }

