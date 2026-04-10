from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
from typing import TYPE_CHECKING, Optional

from .assets import AssetContext, coerce_asset_context, resolve_asset_root
from .errors import ValidationError
from .types import Box, Collision, Cylinder, Mesh, Origin, Part, Sphere, Visual

if TYPE_CHECKING:
    from .articulated_object import ArticulatedObject


_COMPILED_MODEL_CACHE_ATTR = "_sdk_exact_collision_model_cache"
_ALLOW_EXPLICIT_COLLISIONS_ATTR = "_sdk_allow_explicit_collisions"
_CACHE_VERSION = 6


def compile_object_model_with_exact_collisions(
    object_model: "ArticulatedObject",
    *,
    asset_root: Optional[AssetContext | str | Path] = None,
    validate: bool = True,
) -> "ArticulatedObject":
    if validate:
        object_model.validate(strict=True)
    assets = _resolve_compile_assets(object_model, asset_root=asset_root)
    cache_key = _compiled_model_cache_key(object_model, assets=assets)
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
    setattr(compiled, _ALLOW_EXPLICIT_COLLISIONS_ATTR, True)
    compiled.materials = [copy.deepcopy(material) for material in object_model.materials]
    compiled.articulations = [
        copy.deepcopy(articulation) for articulation in object_model.articulations
    ]

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
) -> list[Collision]:
    collisions: list[Collision] = []
    for visual in part.visuals:
        collisions.extend(_collisions_from_visual(visual, assets=assets))
    return collisions


def _collisions_from_visual(
    visual: Visual,
    *,
    assets: AssetContext,
) -> list[Collision]:
    geometry = visual.geometry
    if isinstance(geometry, (Box, Cylinder, Sphere)):
        return [
            Collision(
                geometry=copy.deepcopy(geometry),
                origin=copy.deepcopy(visual.origin),
                name=visual.name,
            )
        ]

    if not isinstance(geometry, Mesh):
        raise ValidationError(
            f"Unsupported visual geometry for collision derivation: {type(geometry).__name__}"
        )

    mesh_path = assets.mesh_path(geometry.filename, ensure_dir=False).resolve()
    if mesh_path.suffix.lower() != ".obj":
        raise ValidationError(
            f"Exact collision derivation only supports OBJ meshes (got: {mesh_path.name!r})"
        )
    if not mesh_path.exists():
        raise ValidationError(f"Mesh file not found: {mesh_path}")

    return [
        Collision(
            geometry=copy.deepcopy(geometry),
            origin=copy.deepcopy(visual.origin),
            name=visual.name,
        )
    ]


def _compiled_model_cache_key(
    object_model: "ArticulatedObject",
    *,
    assets: AssetContext,
) -> str:
    payload = {
        "version": _CACHE_VERSION,
        "asset_root": assets.asset_root.as_posix(),
        "model": _serialize_object_model(object_model),
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), default=_json_default
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


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
