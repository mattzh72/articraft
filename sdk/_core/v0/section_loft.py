from __future__ import annotations

from contextlib import suppress
from dataclasses import dataclass
from typing import Literal, Mapping, Optional, Sequence

import numpy as np

from sdk._dependencies import require_cadquery

from .errors import ValidationError
from .mesh import MeshGeometry, _geometry_from_manifold, _manifold_from_geometry

Vec3 = tuple[float, float, float]

Continuity = Literal["C1", "C2", "C3"]
Parametrization = Literal["uniform", "chordal", "centripetal"]
RepairMode = Literal["off", "mesh", "kernel", "auto"]
SymmetryMode = Literal["mirror_yz"]

__all__ = [
    "LoftSection",
    "LoftTessellation",
    "SectionLoftSpec",
    "section_loft",
    "repair_loft",
]


def _as_vec3(values: Sequence[float], *, name: str) -> Vec3:
    if len(values) != 3:
        raise ValidationError(f"{name} must have 3 elements")
    return (float(values[0]), float(values[1]), float(values[2]))


@dataclass(frozen=True)
class LoftTessellation:
    tolerance: float = 0.001
    angular_tolerance: float = 0.1

    def __post_init__(self) -> None:
        if float(self.tolerance) <= 0.0:
            raise ValidationError("tessellation.tolerance must be > 0")
        if float(self.angular_tolerance) <= 0.0:
            raise ValidationError("tessellation.angular_tolerance must be > 0")
        object.__setattr__(self, "tolerance", float(self.tolerance))
        object.__setattr__(self, "angular_tolerance", float(self.angular_tolerance))


@dataclass(frozen=True)
class LoftSection:
    points: tuple[Vec3, ...]

    def __post_init__(self) -> None:
        coerced = tuple(_normalize_section_points(self.points, name="section.points"))
        object.__setattr__(self, "points", coerced)


@dataclass(frozen=True)
class SectionLoftSpec:
    sections: tuple[LoftSection, ...]
    path: Optional[tuple[Vec3, ...]] = None
    guide_curves: Optional[Mapping[str, tuple[Vec3, ...]]] = None
    cap: bool = True
    solid: bool = True
    symmetry: Optional[SymmetryMode] = None
    ruled: bool = False
    continuity: Continuity = "C2"
    parametrization: Parametrization = "uniform"
    degree: int = 3
    compat: bool = True
    smoothing: bool = False
    weights: tuple[float, float, float] = (1.0, 1.0, 1.0)
    repair: RepairMode = "auto"
    tessellation: LoftTessellation = LoftTessellation()

    def __post_init__(self) -> None:
        sections = tuple(_coerce_loft_section(section) for section in self.sections)
        if len(sections) < 2:
            raise ValidationError("SectionLoftSpec.sections must contain at least two sections")
        object.__setattr__(self, "sections", sections)

        if self.path is not None:
            object.__setattr__(
                self,
                "path",
                tuple(_as_vec3(point, name="path[]") for point in self.path),
            )
        if self.guide_curves is not None:
            guide_curves = {
                str(name): tuple(_as_vec3(point, name=f"guide_curves[{name!r}][]") for point in pts)
                for name, pts in self.guide_curves.items()
            }
            object.__setattr__(self, "guide_curves", guide_curves)

        if self.symmetry is not None and self.symmetry != "mirror_yz":
            raise ValidationError("symmetry must be 'mirror_yz' or None")
        if self.degree < 1:
            raise ValidationError("degree must be >= 1")
        if len(self.weights) != 3:
            raise ValidationError("weights must contain exactly 3 values")
        object.__setattr__(self, "degree", int(self.degree))
        object.__setattr__(
            self,
            "weights",
            (float(self.weights[0]), float(self.weights[1]), float(self.weights[2])),
        )


def _normalize_section_points(points: Sequence[Sequence[float] | Vec3], *, name: str) -> list[Vec3]:
    raw = [_as_vec3(point, name=name) for point in points]
    if len(raw) < 3:
        raise ValidationError(f"{name} must contain at least 3 points")

    out: list[Vec3] = []
    for point in raw:
        if not out or point != out[-1]:
            out.append(point)

    if len(out) >= 2 and out[0] == out[-1]:
        out.pop()

    if len(out) < 3:
        raise ValidationError(f"{name} must contain at least 3 distinct points")
    return out


def _coerce_loft_section(value: LoftSection | Sequence[Sequence[float] | Vec3]) -> LoftSection:
    if isinstance(value, LoftSection):
        return value
    return LoftSection(points=tuple(value))


def _coerce_loft_spec(
    spec: SectionLoftSpec | Sequence[LoftSection | Sequence[Sequence[float] | Vec3]],
) -> SectionLoftSpec:
    if isinstance(spec, SectionLoftSpec):
        return spec
    return SectionLoftSpec(sections=tuple(_coerce_loft_section(section) for section in spec))


def _require_cadquery():
    cq = require_cadquery(feature="section lofts")
    try:
        return cq.occ_impl.shapes
    except Exception as exc:  # pragma: no cover - defensive
        raise RuntimeError(
            "section lofts require a working `cadquery` installation. "
            "Run `uv sync --group dev` from the repository root, then retry."
        ) from exc


def _require_trimesh():
    try:
        import trimesh
    except Exception as exc:  # pragma: no cover - optional import path
        raise RuntimeError(
            "repair_loft requires the `trimesh` package, but it is not installed in the current "
            "environment. Run `uv sync --group dev` from the repository root, then retry."
        ) from exc
    return trimesh


def _build_section_wires(spec: SectionLoftSpec):
    cq_shapes = _require_cadquery()
    wires = []
    for idx, section in enumerate(spec.sections):
        points = tuple(section.points)
        try:
            wire = cq_shapes.polygon(*points)
        except Exception as exc:
            raise ValidationError(f"Failed to build wire for section[{idx}]") from exc
        wires.append(wire)
    return wires


def _curve_from_points(
    points: Sequence[Vec3],
    *,
    name: str,
):
    cq_shapes = _require_cadquery()
    if len(points) < 2:
        raise ValidationError(f"{name} must contain at least 2 points")
    if len(points) == 2:
        return cq_shapes.segment(points[0], points[1])
    try:
        return cq_shapes.spline(*points)
    except Exception:
        try:
            return cq_shapes.polyline(*points)
        except Exception as exc:
            raise ValidationError(f"Failed to build {name}") from exc


def _resolve_path_points(spec: SectionLoftSpec) -> Optional[tuple[Vec3, ...]]:
    if spec.path is not None:
        return spec.path
    if spec.guide_curves is None:
        return None
    spine = spec.guide_curves.get("spine")
    return spine


def _resolve_aux_spine_points(spec: SectionLoftSpec) -> Optional[tuple[Vec3, ...]]:
    if spec.guide_curves is None:
        return None
    aux = spec.guide_curves.get("aux_spine")
    if aux is not None:
        return aux
    return spec.guide_curves.get("binormal")


def _validate_guide_curve_names(spec: SectionLoftSpec) -> None:
    if spec.guide_curves is None:
        return
    supported = {"spine", "aux_spine", "binormal"}
    unsupported = sorted(set(spec.guide_curves) - supported)
    if unsupported:
        raise ValidationError(
            "Unsupported guide_curves keys: "
            + ", ".join(repr(name) for name in unsupported)
            + ". Supported keys are 'spine', 'aux_spine', and 'binormal'."
        )


def _mirror_geometry_yz(geometry: MeshGeometry) -> MeshGeometry:
    mirrored = MeshGeometry(
        vertices=[(-float(x), float(y), float(z)) for (x, y, z) in geometry.vertices],
        faces=[(int(a), int(c), int(b)) for (a, b, c) in geometry.faces],
    )
    return mirrored


def _apply_symmetry_shape(shape, *, symmetry: Optional[SymmetryMode]):
    if symmetry is None:
        return shape
    if symmetry != "mirror_yz":
        raise ValidationError(f"Unsupported symmetry mode: {symmetry!r}")

    try:
        mirrored = shape.mirror("YZ")
        if hasattr(shape, "fuse"):
            fused = shape.fuse(mirrored, glue=True)
        else:
            fused = shape
        if hasattr(fused, "fix"):
            fused = fused.fix()
        if hasattr(fused, "clean"):
            fused = fused.clean()
        return fused
    except Exception as exc:
        raise ValidationError("Failed to apply mirror_yz symmetry") from exc


def _heal_shape(shape, *, solid: bool):
    cq_shapes = _require_cadquery()

    if hasattr(shape, "fix"):
        shape = shape.fix()

    if solid:
        with suppress(Exception):
            shape = cq_shapes.solid(shape)
        if hasattr(shape, "fix"):
            shape = shape.fix()

    return shape


def _mesh_from_shape(shape, *, tessellation: LoftTessellation) -> MeshGeometry:
    try:
        vertices_raw, triangles_raw = shape.tessellate(
            float(tessellation.tolerance),
            float(tessellation.angular_tolerance),
        )
    except TypeError:
        vertices_raw, triangles_raw = shape.tessellate(float(tessellation.tolerance))

    vertices = [
        (float(vertex.x), float(vertex.y), float(vertex.z))
        if hasattr(vertex, "x")
        else (float(vertex[0]), float(vertex[1]), float(vertex[2]))
        for vertex in vertices_raw
    ]
    faces = [(int(face[0]), int(face[1]), int(face[2])) for face in triangles_raw]
    if not vertices or not faces:
        raise ValueError("CadQuery loft tessellation produced an empty mesh")
    return MeshGeometry(vertices=vertices, faces=faces)


def _repair_mesh_geometry(geometry: MeshGeometry) -> MeshGeometry:
    trimesh = _require_trimesh()
    if not geometry.vertices or not geometry.faces:
        return geometry.copy()

    mesh = trimesh.Trimesh(
        vertices=np.asarray(geometry.vertices, dtype=np.float64),
        faces=np.asarray(geometry.faces, dtype=np.int64),
        process=False,
        validate=False,
    )
    mesh.process(validate=False)
    mesh.update_faces(mesh.unique_faces() & mesh.nondegenerate_faces())
    mesh.fill_holes()
    mesh.fix_normals(multibody=False)
    mesh.remove_unreferenced_vertices()

    repaired = MeshGeometry(
        vertices=[(float(v[0]), float(v[1]), float(v[2])) for v in mesh.vertices],
        faces=[(int(f[0]), int(f[1]), int(f[2])) for f in mesh.faces],
    )

    with suppress(Exception):
        repaired = _geometry_from_manifold(_manifold_from_geometry(repaired, name="geometry"))

    return repaired


def section_loft(
    spec: SectionLoftSpec | Sequence[LoftSection | Sequence[Sequence[float] | Vec3]],
    /,
    **overrides,
) -> MeshGeometry:
    if overrides:
        spec = SectionLoftSpec(**({**_coerce_loft_spec(spec).__dict__, **overrides}))
    else:
        spec = _coerce_loft_spec(spec)

    cq_shapes = _require_cadquery()
    _validate_guide_curve_names(spec)
    wires = _build_section_wires(spec)
    path_points = _resolve_path_points(spec)
    aux_spine_points = _resolve_aux_spine_points(spec)

    if path_points is None:
        shape = cq_shapes.loft(
            wires,
            cap=bool(spec.cap and spec.solid),
            ruled=bool(spec.ruled),
            continuity=spec.continuity,
            parametrization=spec.parametrization,
            degree=int(spec.degree),
            compat=bool(spec.compat),
            smoothing=bool(spec.smoothing),
            weights=spec.weights,
        )
    else:
        path_curve = _curve_from_points(path_points, name="path")
        mode = None
        if aux_spine_points is not None:
            mode = _curve_from_points(aux_spine_points, name="guide_curves['aux_spine']")
        shape = cq_shapes.Solid.sweep_multi(
            wires,
            path_curve,
            makeSolid=bool(spec.solid and spec.cap),
            isFrenet=False,
            mode=mode,
        )

    shape = _apply_symmetry_shape(shape, symmetry=spec.symmetry)

    if spec.repair in {"auto", "kernel"}:
        shape = _heal_shape(shape, solid=bool(spec.solid and spec.cap))

    geometry = _mesh_from_shape(shape, tessellation=spec.tessellation)

    if spec.repair in {"auto", "mesh"}:
        geometry = _repair_mesh_geometry(geometry)

    return geometry


def repair_loft(
    geometry_or_spec: MeshGeometry
    | SectionLoftSpec
    | Sequence[LoftSection | Sequence[Sequence[float] | Vec3]],
    /,
    *,
    repair: RepairMode = "auto",
) -> MeshGeometry:
    if isinstance(geometry_or_spec, MeshGeometry):
        if repair == "off":
            return geometry_or_spec.copy()
        return _repair_mesh_geometry(geometry_or_spec)

    spec = _coerce_loft_spec(geometry_or_spec)
    if repair != "auto":
        spec = SectionLoftSpec(**{**spec.__dict__, "repair": repair})
    return section_loft(spec)
