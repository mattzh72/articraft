from __future__ import annotations

import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .assets import resolve_mesh_path


def _pad4(data: bytes, *, fill: bytes) -> bytes:
    padding = (-len(data)) % 4
    if padding == 0:
        return data
    return data + (fill * padding)


def _obj_vertex_index(token: str, vertex_count: int) -> int:
    raw = token.split("/", 1)[0].strip()
    if raw == "":
        raise ValueError("OBJ face token is missing a vertex index")
    index = int(raw)
    if index > 0:
        return index - 1
    if index < 0:
        return vertex_count + index
    raise ValueError("OBJ face indices are 1-based and may not be zero")


def load_obj_triangles(obj_path: Path) -> tuple[np.ndarray, np.ndarray]:
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    for raw_line in obj_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        if line.startswith("v "):
            parts = line.split()
            if len(parts) < 4:
                continue
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            continue

        if line.startswith("f "):
            tokens = line.split()[1:]
            if len(tokens) < 3:
                continue
            corner_indices = [_obj_vertex_index(token, len(vertices)) for token in tokens]
            for i in range(1, len(corner_indices) - 1):
                faces.append((corner_indices[0], corner_indices[i], corner_indices[i + 1]))

    if not vertices:
        raise ValueError(f"OBJ contains no vertices: {obj_path}")
    if not faces:
        raise ValueError(f"OBJ contains no faces: {obj_path}")

    return np.asarray(vertices, dtype=np.float32), np.asarray(faces, dtype=np.uint32)


def _compute_vertex_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    normals = np.zeros_like(vertices, dtype=np.float32)
    tri_vertices = vertices[faces]
    face_normals = np.cross(
        tri_vertices[:, 1] - tri_vertices[:, 0],
        tri_vertices[:, 2] - tri_vertices[:, 0],
    )

    for face, normal in zip(faces, face_normals, strict=True):
        normals[face[0]] += normal
        normals[face[1]] += normal
        normals[face[2]] += normal

    lengths = np.linalg.norm(normals, axis=1)
    nonzero = lengths > 1e-12
    normals[nonzero] /= lengths[nonzero][:, None]
    normals[~nonzero] = np.array((0.0, 0.0, 1.0), dtype=np.float32)
    return normals.astype(np.float32, copy=False)


def _cleanup_triangles_for_viewer(
    vertices: np.ndarray,
    faces: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if vertices.size == 0 or faces.size == 0:
        return vertices, faces

    try:
        import trimesh
    except Exception:
        return vertices, faces

    mesh = trimesh.Trimesh(
        vertices=np.asarray(vertices, dtype=np.float64),
        faces=np.asarray(faces, dtype=np.int64),
        process=False,
        validate=False,
    )
    mesh.process(validate=False)
    mesh.update_faces(mesh.unique_faces() & mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals(multibody=False)

    cleaned_vertices = np.asarray(mesh.vertices, dtype=np.float32)
    cleaned_faces = np.asarray(mesh.faces, dtype=np.uint32)
    if cleaned_vertices.size == 0 or cleaned_faces.size == 0:
        return vertices, faces
    return cleaned_vertices, cleaned_faces


def build_glb_bytes(vertices: np.ndarray, faces: np.ndarray) -> bytes:
    if vertices.ndim != 2 or vertices.shape[1] != 3:
        raise ValueError("Vertices must have shape (N, 3)")
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError("Faces must have shape (M, 3)")

    vertices, faces = _cleanup_triangles_for_viewer(vertices, faces)
    vertices = np.ascontiguousarray(vertices, dtype=np.float32)
    normals = np.ascontiguousarray(_compute_vertex_normals(vertices, faces), dtype=np.float32)
    indices = np.ascontiguousarray(faces.reshape(-1), dtype=np.uint32)

    positions_blob = vertices.tobytes()
    normals_blob = normals.tobytes()
    indices_blob = indices.tobytes()

    pos_offset = 0
    norm_offset = len(positions_blob)
    index_offset = norm_offset + len(normals_blob)
    binary_blob = positions_blob + normals_blob + indices_blob
    binary_blob = _pad4(binary_blob, fill=b"\x00")

    mins = vertices.min(axis=0).tolist()
    maxs = vertices.max(axis=0).tolist()
    index_min = int(indices.min())
    index_max = int(indices.max())

    gltf = {
        "asset": {"version": "2.0", "generator": "synth-urdf-agent"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [
            {
                "primitives": [
                    {
                        "attributes": {"POSITION": 0, "NORMAL": 1},
                        "indices": 2,
                        "mode": 4,
                    }
                ]
            }
        ],
        "buffers": [{"byteLength": len(binary_blob)}],
        "bufferViews": [
            {
                "buffer": 0,
                "byteOffset": pos_offset,
                "byteLength": len(positions_blob),
                "target": 34962,
            },
            {
                "buffer": 0,
                "byteOffset": norm_offset,
                "byteLength": len(normals_blob),
                "target": 34962,
            },
            {
                "buffer": 0,
                "byteOffset": index_offset,
                "byteLength": len(indices_blob),
                "target": 34963,
            },
        ],
        "accessors": [
            {
                "bufferView": 0,
                "componentType": 5126,
                "count": int(vertices.shape[0]),
                "type": "VEC3",
                "min": mins,
                "max": maxs,
            },
            {
                "bufferView": 1,
                "componentType": 5126,
                "count": int(normals.shape[0]),
                "type": "VEC3",
            },
            {
                "bufferView": 2,
                "componentType": 5125,
                "count": int(indices.shape[0]),
                "type": "SCALAR",
                "min": [index_min],
                "max": [index_max],
            },
        ],
    }

    json_blob = json.dumps(gltf, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    json_blob = _pad4(json_blob, fill=b" ")

    total_length = 12 + 8 + len(json_blob) + 8 + len(binary_blob)
    header = struct.pack("<4sII", b"glTF", 2, total_length)
    json_chunk = struct.pack("<I4s", len(json_blob), b"JSON") + json_blob
    bin_chunk = struct.pack("<I4s", len(binary_blob), b"BIN\x00") + binary_blob
    return header + json_chunk + bin_chunk


def convert_obj_file_to_glb(obj_path: Path, glb_path: Path | None = None) -> Path:
    obj_path = Path(obj_path)
    out_path = Path(glb_path) if glb_path is not None else obj_path.with_suffix(".glb")

    vertices, faces = load_obj_triangles(obj_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(build_glb_bytes(vertices, faces))
    return out_path


def convert_urdf_visual_meshes_to_glb(
    urdf_xml: str,
    *,
    asset_root: Path,
) -> tuple[str, list[str]]:
    root = ET.fromstring(urdf_xml)
    warnings: list[str] = []
    converted: dict[Path, Path] = {}

    for mesh_el in root.findall(".//visual/geometry/mesh"):
        filename = mesh_el.attrib.get("filename")
        if not isinstance(filename, str) or filename.strip() == "":
            continue
        if not filename.lower().endswith(".obj"):
            continue

        mesh_path = resolve_mesh_path(filename, assets=asset_root)

        glb_path = mesh_path.with_suffix(".glb")
        try:
            needs_refresh = (not glb_path.exists()) or (
                mesh_path.stat().st_mtime_ns > glb_path.stat().st_mtime_ns
            )
            if needs_refresh:
                convert_obj_file_to_glb(mesh_path, glb_path)
            converted[mesh_path] = glb_path
        except Exception as exc:
            warnings.append(f"Failed to convert mesh to GLB: {mesh_path} ({exc})")
            continue

        mesh_el.set("filename", filename[:-4] + ".glb")

    try:
        ET.indent(root, space="  ")
    except Exception:
        pass
    return ET.tostring(root, encoding="unicode"), warnings
