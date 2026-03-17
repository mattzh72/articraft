from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
from dataclasses import asdict
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Iterable, Optional

import coacd
import numpy as np
import trimesh

from .assets import AssetContext, coerce_asset_context, resolve_asset_root
from .errors import ValidationError
from .types import Box, Collision, Cylinder, Mesh, Origin, Part, Sphere, Visual

if TYPE_CHECKING:
    from .articulated_object import ArticulatedObject


_COMPILED_MODEL_CACHE_ATTR = "_sdk_generated_collision_model_cache"
_CACHE_VERSION = 1
_AFFINE_TOL = 1e-6

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]
Mat4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


@dataclass(frozen=True)
class CollisionGenerationSettings:
    threshold: float = 0.08
    max_convex_hulls: int = 16
    preprocess_resolution: int = 24
    max_ch_vertex: int = 64
    mcts_max_depth: int = 3
    mcts_nodes: int = 12
    mcts_iterations: int = 60
    merge: bool = True
    seed: int = 0


def collision_generation_settings_from_env() -> CollisionGenerationSettings:
    def _env_float(name: str, default: float) -> float:
        raw = os.environ.get(name)
        if raw is None:
            return default
        try:
            return float(raw.strip())
        except Exception:
            return default

    def _env_int(name: str, default: int) -> int:
        raw = os.environ.get(name)
        if raw is None:
            return default
        try:
            return int(raw.strip())
        except Exception:
            return default

    def _env_bool(name: str, default: bool) -> bool:
        raw = os.environ.get(name)
        if raw is None:
            return default
        return raw.strip().lower() in {"1", "true", "yes", "on"}

    return CollisionGenerationSettings(
        threshold=_env_float("SDK_COLLISION_COACD_THRESHOLD", 0.08),
        max_convex_hulls=_env_int("SDK_COLLISION_COACD_MAX_HULLS", 16),
        preprocess_resolution=_env_int("SDK_COLLISION_COACD_PREPROCESS_RESOLUTION", 24),
        max_ch_vertex=_env_int("SDK_COLLISION_COACD_MAX_CH_VERTEX", 64),
        mcts_max_depth=_env_int("SDK_COLLISION_COACD_MCTS_MAX_DEPTH", 3),
        mcts_nodes=_env_int("SDK_COLLISION_COACD_MCTS_NODES", 12),
        mcts_iterations=_env_int("SDK_COLLISION_COACD_MCTS_ITERATIONS", 60),
        merge=_env_bool("SDK_COLLISION_COACD_MERGE", True),
        seed=_env_int("SDK_COLLISION_COACD_SEED", 0),
    )


def compile_object_model_with_generated_collisions(
    object_model: "ArticulatedObject",
    *,
    asset_root: Optional[AssetContext | str | Path] = None,
    settings: Optional[CollisionGenerationSettings] = None,
) -> "ArticulatedObject":
    object_model.validate(strict=True)
    assets = _resolve_compile_assets(object_model, asset_root=asset_root)
    cfg = settings or collision_generation_settings_from_env()
    cache_key = _compiled_model_cache_key(
        object_model,
        assets=assets,
        settings=cfg,
    )
    cached_models = getattr(object_model, _COMPILED_MODEL_CACHE_ATTR, None)
    if isinstance(cached_models, dict):
        cached_model = cached_models.get(cache_key)
        if cached_model is not None:
            return cached_model

    from .articulated_object import ArticulatedObject

    compiled = ArticulatedObject(
        object_model.name,
        meta=copy.deepcopy(object_model.meta),
        assets=assets,
    )
    compiled.materials = [copy.deepcopy(material) for material in object_model.materials]
    compiled.articulations = [copy.deepcopy(articulation) for articulation in object_model.articulations]

    for source_part in object_model.parts:
        compiled_part = Part(
            name=source_part.name,
            visuals=[copy.deepcopy(visual) for visual in source_part.visuals],
            collisions=[],
            inertial=copy.deepcopy(source_part.inertial),
            meta=copy.deepcopy(source_part.meta),
            assets=coerce_asset_context(getattr(source_part, "assets", None)) or assets,
        )
        compiled_part.collisions = _generate_part_collisions(
            source_part,
            assets=compiled_part.assets or assets,
            settings=cfg,
        )
        compiled.parts.append(compiled_part)

    if not isinstance(cached_models, dict):
        cached_models = {}
        setattr(object_model, _COMPILED_MODEL_CACHE_ATTR, cached_models)
    cached_models[cache_key] = compiled
    return compiled


def _resolve_compile_assets(
    object_model: "ArticulatedObject",
    *,
    asset_root: Optional[AssetContext | str | Path],
) -> AssetContext:
    resolved = coerce_asset_context(asset_root)
    if resolved is not None:
        return resolved

    model_assets = coerce_asset_context(getattr(object_model, "assets", None))
    if model_assets is not None:
        return model_assets

    root = resolve_asset_root(asset_root, object_model)
    if root is not None:
        return AssetContext(root)

    return AssetContext(Path(".").resolve())


def _generate_part_collisions(
    part: Part,
    *,
    assets: AssetContext,
    settings: CollisionGenerationSettings,
) -> list[Collision]:
    collisions: list[Collision] = []
    for index, visual in enumerate(part.visuals, start=1):
        collisions.extend(
            _collisions_from_visual(
                part,
                visual,
                visual_index=index,
                assets=assets,
                settings=settings,
            )
        )
    return collisions


def _collisions_from_visual(
    part: Part,
    visual: Visual,
    *,
    visual_index: int,
    assets: AssetContext,
    settings: CollisionGenerationSettings,
) -> list[Collision]:
    geometry = visual.geometry
    name_prefix = _safe_name(part.name)
    visual_key = _safe_name(visual.name or f"visual_{visual_index}")
    if isinstance(geometry, (Box, Cylinder, Sphere)):
        return [
            Collision(
                geometry=copy.deepcopy(geometry),
                origin=copy.deepcopy(visual.origin),
                name=f"{name_prefix}__{visual_key}",
            )
        ]

    if not isinstance(geometry, Mesh):
        raise ValidationError(
            f"Unsupported visual geometry for collision generation: {type(geometry).__name__}"
        )

    mesh_path = Path(geometry.filename)
    if not mesh_path.is_absolute():
        mesh_path = (assets.asset_root / mesh_path).resolve()
    if mesh_path.suffix.lower() != ".obj":
        raise ValidationError(
            f"Collision generation only supports OBJ meshes (got: {mesh_path.name!r})"
        )
    if not mesh_path.exists():
        raise ValidationError(f"Mesh file not found: {mesh_path}")

    primitive_collision = _collision_from_primitive_backed_mesh(
        geometry=geometry,
        visual_origin=visual.origin,
        name=f"{name_prefix}__{visual_key}",
    )
    if primitive_collision is not None:
        return [primitive_collision]

    try:
        loaded = trimesh.load_mesh(mesh_path, force="mesh")
    except Exception as exc:
        raise ValidationError(f"Failed to load mesh for collision generation: {mesh_path} ({exc})") from exc
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValidationError(f"Expected mesh geometry at {mesh_path}, got {type(loaded).__name__}")

    tm = loaded.copy()
    if geometry.scale is not None:
        tm.apply_scale(tuple(float(v) for v in geometry.scale))

    cached_hull_paths = _load_cached_hull_paths(
        mesh_path=mesh_path,
        geometry=geometry,
        assets=assets,
        settings=settings,
    )
    if cached_hull_paths is None:
        cached_hull_paths = _write_cached_hull_paths(
            tm=tm,
            mesh_path=mesh_path,
            geometry=geometry,
            assets=assets,
            settings=settings,
        )

    collisions: list[Collision] = []
    for hull_index, hull_path in enumerate(cached_hull_paths, start=1):
        rel_path = hull_path.relative_to(assets.mesh_dir)
        collisions.append(
            Collision(
                geometry=Mesh(filename=assets.mesh_ref(rel_path)),
                origin=copy.deepcopy(visual.origin),
                name=f"{name_prefix}__{visual_key}__hull_{hull_index:03d}",
            )
        )
    return collisions


def _collision_from_primitive_backed_mesh(
    *,
    geometry: Mesh,
    visual_origin: Origin,
    name: str,
) -> Optional[Collision]:
    source = geometry.source_geometry
    if not isinstance(source, (Box, Cylinder, Sphere)):
        return None

    local_tf = _mesh_source_affine(geometry)
    decomposed = _decompose_primitive_affine(source, local_tf)
    if decomposed is None:
        return None

    collision_geometry, local_rigid_tf = decomposed
    final_tf = _mat4_mul(_origin_to_mat4(visual_origin), local_rigid_tf)
    return Collision(
        geometry=collision_geometry,
        origin=_origin_from_mat4(final_tf),
        name=name,
    )


def _mesh_source_affine(geometry: Mesh) -> Mat4:
    base = _coerce_mat4(geometry.source_transform) or _identity4()
    if geometry.scale is None:
        return base
    sx, sy, sz = (float(v) for v in geometry.scale)
    scale_tf: Mat4 = (
        (sx, 0.0, 0.0, 0.0),
        (0.0, sy, 0.0, 0.0),
        (0.0, 0.0, sz, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    return _mat4_mul(scale_tf, base)


def _decompose_primitive_affine(
    primitive: Box | Cylinder | Sphere,
    affine: Mat4,
) -> Optional[tuple[Box | Cylinder | Sphere, Mat4]]:
    linear = tuple(
        tuple(float(affine[row][col]) for col in range(3))
        for row in range(3)
    )
    translation = (
        float(affine[0][3]),
        float(affine[1][3]),
        float(affine[2][3]),
    )
    columns = [
        (linear[0][col], linear[1][col], linear[2][col])
        for col in range(3)
    ]
    scales = [math.sqrt(sum(component * component for component in col)) for col in columns]
    if any(scale <= _AFFINE_TOL for scale in scales):
        return None

    for i in range(3):
        for j in range(i + 1, 3):
            dot = sum(columns[i][k] * columns[j][k] for k in range(3))
            limit = _AFFINE_TOL * max(1.0, scales[i] * scales[j])
            if abs(dot) > limit:
                return None

    rot_cols = [
        tuple(component / scales[idx] for component in columns[idx])
        for idx in range(3)
    ]
    rotation = tuple(
        tuple(rot_cols[col][row] for col in range(3))
        for row in range(3)
    )
    det = _mat3_det(rotation)
    if abs(det) <= _AFFINE_TOL:
        return None
    if det < 0.0:
        rot_cols[0] = tuple(-component for component in rot_cols[0])
        rotation = tuple(
            tuple(rot_cols[col][row] for col in range(3))
            for row in range(3)
        )

    sx, sy, sz = (float(scale) for scale in scales)
    if isinstance(primitive, Box):
        collision_geometry: Box | Cylinder | Sphere = Box(
            (
                float(primitive.size[0]) * sx,
                float(primitive.size[1]) * sy,
                float(primitive.size[2]) * sz,
            )
        )
    elif isinstance(primitive, Cylinder):
        if not _approx_equal(sx, sy):
            return None
        collision_geometry = Cylinder(
            radius=float(primitive.radius) * ((sx + sy) * 0.5),
            length=float(primitive.length) * sz,
        )
    else:
        if not (_approx_equal(sx, sy) and _approx_equal(sx, sz)):
            return None
        collision_geometry = Sphere(
            radius=float(primitive.radius) * ((sx + sy + sz) / 3.0)
        )

    rigid_tf: Mat4 = (
        (rotation[0][0], rotation[0][1], rotation[0][2], translation[0]),
        (rotation[1][0], rotation[1][1], rotation[1][2], translation[1]),
        (rotation[2][0], rotation[2][1], rotation[2][2], translation[2]),
        (0.0, 0.0, 0.0, 1.0),
    )
    return collision_geometry, rigid_tf


def _approx_equal(a: float, b: float, *, tol: float = _AFFINE_TOL) -> bool:
    return abs(float(a) - float(b)) <= float(tol) * max(1.0, abs(float(a)), abs(float(b)))


def _coerce_mat4(value: object) -> Optional[Mat4]:
    if not isinstance(value, tuple) or len(value) != 4:
        return None
    rows: list[tuple[float, float, float, float]] = []
    for row in value:
        if not isinstance(row, tuple) or len(row) != 4:
            return None
        rows.append((float(row[0]), float(row[1]), float(row[2]), float(row[3])))
    return (rows[0], rows[1], rows[2], rows[3])


def _mat3_det(mat: Mat3) -> float:
    return (
        mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1])
        - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
        + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0])
    )


def _identity4() -> Mat4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _mat4_mul(a: Mat4, b: Mat4) -> Mat4:
    out: list[list[float]] = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = (
                float(a[i][0]) * float(b[0][j])
                + float(a[i][1]) * float(b[1][j])
                + float(a[i][2]) * float(b[2][j])
                + float(a[i][3]) * float(b[3][j])
            )
    return (
        (out[0][0], out[0][1], out[0][2], out[0][3]),
        (out[1][0], out[1][1], out[1][2], out[1][3]),
        (out[2][0], out[2][1], out[2][2], out[2][3]),
        (out[3][0], out[3][1], out[3][2], out[3][3]),
    )


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> Mat3:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _origin_to_mat4(origin: Origin) -> Mat4:
    roll, pitch, yaw = (float(v) for v in origin.rpy)
    rotation = _rpy_matrix(roll, pitch, yaw)
    ox, oy, oz = (float(v) for v in origin.xyz)
    return (
        (rotation[0][0], rotation[0][1], rotation[0][2], ox),
        (rotation[1][0], rotation[1][1], rotation[1][2], oy),
        (rotation[2][0], rotation[2][1], rotation[2][2], oz),
        (0.0, 0.0, 0.0, 1.0),
    )


def _origin_from_mat4(mat: Mat4) -> Origin:
    rotation: Mat3 = (
        (float(mat[0][0]), float(mat[0][1]), float(mat[0][2])),
        (float(mat[1][0]), float(mat[1][1]), float(mat[1][2])),
        (float(mat[2][0]), float(mat[2][1]), float(mat[2][2])),
    )
    return Origin(
        xyz=(float(mat[0][3]), float(mat[1][3]), float(mat[2][3])),
        rpy=_rpy_from_matrix(rotation),
    )


def _rpy_from_matrix(mat: Mat3) -> Vec3:
    pitch = math.asin(max(-1.0, min(1.0, -float(mat[2][0]))))
    cp = math.cos(pitch)
    if abs(cp) > _AFFINE_TOL:
        roll = math.atan2(float(mat[2][1]), float(mat[2][2]))
        yaw = math.atan2(float(mat[1][0]), float(mat[0][0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(mat[0][1]), float(mat[1][1]))
    return (roll, pitch, yaw)


def _write_cached_hull_paths(
    *,
    tm: trimesh.Trimesh,
    mesh_path: Path,
    geometry: Mesh,
    assets: AssetContext,
    settings: CollisionGenerationSettings,
) -> list[Path]:
    try:
        mesh = coacd.Mesh(
            np.asarray(tm.vertices, dtype=np.float64),
            np.asarray(tm.faces, dtype=np.int32),
        )
        hulls = coacd.run_coacd(
            mesh,
            threshold=float(settings.threshold),
            max_convex_hull=int(settings.max_convex_hulls),
            preprocess_resolution=int(settings.preprocess_resolution),
            max_ch_vertex=int(settings.max_ch_vertex),
            mcts_max_depth=int(settings.mcts_max_depth),
            mcts_nodes=int(settings.mcts_nodes),
            mcts_iterations=int(settings.mcts_iterations),
            merge=bool(settings.merge),
            seed=int(settings.seed),
        )
    except Exception as exc:
        raise ValidationError(
            f"Failed to decompose mesh for collision generation: {mesh_path} ({exc})"
        ) from exc

    cache_key = _mesh_collision_cache_key(
        mesh_path=mesh_path,
        geometry=geometry,
        settings=settings,
    )
    cache_dir = _collision_cache_dir(assets)
    cache_dir.mkdir(parents=True, exist_ok=True)
    hull_paths: list[Path] = []
    for hull_index, hull in enumerate(hulls, start=1):
        if not isinstance(hull, Iterable):
            raise ValidationError(f"Unexpected CoACD hull payload for {mesh_path}")
        hull_vertices, hull_faces = hull
        hull_mesh = trimesh.Trimesh(
            vertices=np.asarray(hull_vertices, dtype=np.float64),
            faces=np.asarray(hull_faces, dtype=np.int64),
            process=False,
        )
        out_name = f"{cache_key}__hull_{hull_index:03d}.obj"
        out_path = cache_dir / out_name
        hull_mesh.export(out_path)
        hull_paths.append(out_path)

    manifest_path = _collision_cache_manifest_path(cache_dir, cache_key)
    manifest_path.write_text(
        json.dumps(
            {
                "version": _CACHE_VERSION,
                "mesh": mesh_path.as_posix(),
                "hulls": [path.name for path in hull_paths],
            },
            sort_keys=True,
            separators=(",", ":"),
        ),
        encoding="utf-8",
    )
    return hull_paths


def _load_cached_hull_paths(
    *,
    mesh_path: Path,
    geometry: Mesh,
    assets: AssetContext,
    settings: CollisionGenerationSettings,
) -> Optional[list[Path]]:
    cache_key = _mesh_collision_cache_key(
        mesh_path=mesh_path,
        geometry=geometry,
        settings=settings,
    )
    cache_dir = _collision_cache_dir(assets)
    manifest_path = _collision_cache_manifest_path(cache_dir, cache_key)
    if not manifest_path.exists():
        return None
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    if int(payload.get("version", -1)) != _CACHE_VERSION:
        return None
    hull_names = payload.get("hulls")
    if not isinstance(hull_names, list) or not hull_names:
        return None
    hull_paths = [cache_dir / str(name) for name in hull_names]
    if not all(path.exists() for path in hull_paths):
        return None
    return hull_paths


def _collision_cache_dir(assets: AssetContext) -> Path:
    return assets.mesh_dir / "collision" / "cache"


def _collision_cache_manifest_path(cache_dir: Path, cache_key: str) -> Path:
    return cache_dir / f"{cache_key}.json"


def _compiled_model_cache_key(
    object_model: "ArticulatedObject",
    *,
    assets: AssetContext,
    settings: CollisionGenerationSettings,
) -> str:
    payload = {
        "version": _CACHE_VERSION,
        "asset_root": assets.asset_root.as_posix(),
        "settings": asdict(settings),
        "model": _serialize_object_model(object_model),
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), default=_json_default).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _mesh_collision_cache_key(
    *,
    mesh_path: Path,
    geometry: Mesh,
    settings: CollisionGenerationSettings,
) -> str:
    digest = hashlib.sha256()
    digest.update(f"v{_CACHE_VERSION}".encode("utf-8"))
    digest.update(mesh_path.read_bytes())
    digest.update(json.dumps(asdict(settings), sort_keys=True, separators=(",", ":")).encode("utf-8"))
    digest.update(json.dumps(list(geometry.scale) if geometry.scale is not None else None).encode("utf-8"))
    digest.update(
        json.dumps(
            _serialize_geometry(geometry.source_geometry) if geometry.source_geometry is not None else None,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    )
    digest.update(
        json.dumps(
            None if geometry.source_transform is None else [list(row) for row in geometry.source_transform],
            separators=(",", ":"),
        ).encode("utf-8")
    )
    return digest.hexdigest()[:24]


def _serialize_object_model(object_model: "ArticulatedObject") -> dict[str, object]:
    return {
        "name": object_model.name,
        "meta": object_model.meta,
        "assets": _serialize_assets(getattr(object_model, "assets", None)),
        "materials": [
            {
                "name": material.name,
                "rgba": material.rgba,
                "texture": material.texture,
            }
            for material in getattr(object_model, "materials", [])
        ],
        "parts": [
            {
                "name": part.name,
                "visuals": [
                    {
                        "geometry": _serialize_geometry(visual.geometry),
                        "origin": {
                            "xyz": tuple(visual.origin.xyz),
                            "rpy": tuple(visual.origin.rpy),
                        },
                        "material": _serialize_material_ref(visual.material),
                        "name": visual.name,
                    }
                    for visual in part.visuals
                ],
                "inertial": _serialize_inertial(part.inertial),
                "meta": part.meta,
                "assets": _serialize_assets(getattr(part, "assets", None)),
            }
            for part in getattr(object_model, "parts", [])
        ],
        "articulations": [
            {
                "name": articulation.name,
                "type": str(articulation.articulation_type),
                "parent": articulation.parent,
                "child": articulation.child,
                "origin": {
                    "xyz": tuple(articulation.origin.xyz),
                    "rpy": tuple(articulation.origin.rpy),
                },
                "axis": tuple(articulation.axis),
                "motion_limits": (
                    None
                    if articulation.motion_limits is None
                    else {
                        "effort": articulation.motion_limits.effort,
                        "velocity": articulation.motion_limits.velocity,
                        "lower": articulation.motion_limits.lower,
                        "upper": articulation.motion_limits.upper,
                    }
                ),
                "motion_properties": (
                    None
                    if articulation.motion_properties is None
                    else {
                        "damping": articulation.motion_properties.damping,
                        "friction": articulation.motion_properties.friction,
                    }
                ),
                "meta": articulation.meta,
            }
            for articulation in getattr(object_model, "articulations", [])
        ],
    }


def _serialize_geometry(geometry: Box | Cylinder | Sphere | Mesh) -> dict[str, object]:
    if isinstance(geometry, Box):
        return {"type": "box", "size": tuple(geometry.size)}
    if isinstance(geometry, Cylinder):
        return {"type": "cylinder", "radius": geometry.radius, "length": geometry.length}
    if isinstance(geometry, Sphere):
        return {"type": "sphere", "radius": geometry.radius}
    return {
        "type": "mesh",
        "filename": os.fspath(geometry.filename),
        "scale": None if geometry.scale is None else tuple(geometry.scale),
        "source_geometry": (
            None
            if geometry.source_geometry is None
            else _serialize_geometry(geometry.source_geometry)
        ),
        "source_transform": (
            None
            if geometry.source_transform is None
            else tuple(tuple(row) for row in geometry.source_transform)
        ),
    }


def _serialize_material_ref(value: object) -> object:
    if value is None:
        return None
    if hasattr(value, "name"):
        name = getattr(value, "name", None)
        if isinstance(name, str):
            return name
    return str(value)


def _serialize_inertial(inertial: object) -> object:
    if inertial is None:
        return None
    inertia = getattr(inertial, "inertia", None)
    origin = getattr(inertial, "origin", Origin())
    return {
        "mass": getattr(inertial, "mass", None),
        "origin": {
            "xyz": tuple(origin.xyz),
            "rpy": tuple(origin.rpy),
        },
        "inertia": None
        if inertia is None
        else {
            "ixx": inertia.ixx,
            "ixy": inertia.ixy,
            "ixz": inertia.ixz,
            "iyy": inertia.iyy,
            "iyz": inertia.iyz,
            "izz": inertia.izz,
        },
    }


def _serialize_assets(assets: object) -> object:
    ctx = coerce_asset_context(assets)
    if ctx is None:
        return None
    return {
        "root": ctx.asset_root.as_posix(),
        "mesh_subdir": ctx.mesh_subdir,
    }


def _json_default(value: object) -> object:
    if isinstance(value, Path):
        return value.as_posix()
    return repr(value)


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", str(value).strip())
    cleaned = cleaned.strip("_")
    return cleaned or "item"
