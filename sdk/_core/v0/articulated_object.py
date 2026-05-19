from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Union

from .assets import AssetContext, AssetSession, coerce_asset_context, resolve_asset_context
from .errors import ValidationError
from .types import (
    Articulation,
    ArticulationType,
    Box,
    Cylinder,
    Geometry,
    Inertial,
    Material,
    Mesh,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    Part,
    Sphere,
    Vec3,
    Visual,
    _normalize_material_rgba,
)

_ALLOW_EXPLICIT_COLLISIONS_ATTR = "_sdk_allow_explicit_collisions"
_SCALAR_ARTICULATION_TYPES = {
    ArticulationType.REVOLUTE,
    ArticulationType.PRISMATIC,
    ArticulationType.CONTINUOUS,
}
_ANGULAR_ARTICULATION_TYPES = {
    ArticulationType.REVOLUTE,
    ArticulationType.CONTINUOUS,
}


def _part_name_ref(value: Union[str, Part], *, field_name: str) -> str:
    if isinstance(value, str):
        name = value
    else:
        name = getattr(value, "name", None)
        if not isinstance(name, str):
            raise ValidationError(f"{field_name} must be a part name or Part instance")
    name = name.strip()
    if not name:
        raise ValidationError(f"{field_name} must be non-empty")
    return name


def _articulation_name_ref(value: Union[str, Articulation], *, field_name: str) -> str:
    if isinstance(value, str):
        name = value
    else:
        name = getattr(value, "name", None)
        if not isinstance(name, str):
            raise ValidationError(
                f"{field_name} must be an articulation name or Articulation instance"
            )
    name = name.strip()
    if not name:
        raise ValidationError(f"{field_name} must be non-empty")
    return name


@dataclass
class ArticulatedObject:
    """Root object model for articulated products and assemblies."""

    name: str
    parts: List[Part] = field(default_factory=list)
    articulations: List[Articulation] = field(default_factory=list)
    materials: List[Material] = field(default_factory=list)
    meta: Dict[str, object] = field(default_factory=dict)
    assets: Optional[AssetContext] = None
    _part_index: Dict[str, Part] = field(default_factory=dict, init=False, repr=False)
    _articulation_index: Dict[str, Articulation] = field(
        default_factory=dict, init=False, repr=False
    )

    def __post_init__(self) -> None:
        self.assets = resolve_asset_context(self.assets)
        for part in self.parts:
            part.assets = coerce_asset_context(getattr(part, "assets", None)) or self.assets

    @property
    def links(self) -> List[Part]:
        return self.parts

    @links.setter
    def links(self, value: List[Part]) -> None:
        self.parts = value

    @property
    def joints(self) -> List[Articulation]:
        return self.articulations

    @joints.setter
    def joints(self, value: List[Articulation]) -> None:
        self.articulations = value

    def part(
        self,
        name: str,
        *,
        visuals: Optional[Iterable[Visual]] = None,
        inertial: Optional[Inertial] = None,
        meta: Optional[Dict[str, object]] = None,
    ) -> Part:
        part = Part(
            name=name,
            visuals=list(visuals or []),
            collisions=[],
            inertial=inertial,
            meta=dict(meta or {}),
            assets=self.assets,
        )
        self.parts.append(part)
        self._part_index[name] = part
        return part

    def link(
        self,
        name: str,
        *,
        visuals: Optional[Iterable[Visual]] = None,
        inertial: Optional[Inertial] = None,
        meta: Optional[Dict[str, object]] = None,
    ) -> Part:
        return self.part(
            name,
            visuals=visuals,
            inertial=inertial,
            meta=meta,
        )

    def articulation(
        self,
        name: str,
        articulation_type: Union[ArticulationType, str],
        parent: Union[str, Part],
        child: Union[str, Part],
        *,
        origin: Optional[Origin] = None,
        axis: Optional[Vec3] = None,
        motion_limits: Optional[MotionLimits] = None,
        motion_properties: Optional[MotionProperties] = None,
        mimic: Optional[Mimic] = None,
        meta: Optional[Dict[str, object]] = None,
    ) -> Articulation:
        try:
            resolved_type = (
                articulation_type
                if isinstance(articulation_type, ArticulationType)
                else ArticulationType(str(articulation_type))
            )
        except ValueError as exc:
            raise ValidationError(f"Unknown articulation type: {articulation_type}") from exc

        articulation = Articulation(
            name=name,
            articulation_type=resolved_type,
            parent=_part_name_ref(parent, field_name="parent"),
            child=_part_name_ref(child, field_name="child"),
            origin=origin or Origin(),
            axis=axis or (0.0, 0.0, 1.0),
            motion_limits=motion_limits,
            motion_properties=motion_properties,
            mimic=mimic,
            meta=dict(meta or {}),
        )
        self.articulations.append(articulation)
        self._articulation_index[name] = articulation
        return articulation

    def joint(
        self,
        name: str,
        joint_type: Union[ArticulationType, str],
        parent: Union[str, Part],
        child: Union[str, Part],
        *,
        origin: Optional[Origin] = None,
        axis: Optional[Vec3] = None,
        limit: Optional[MotionLimits] = None,
        dynamics: Optional[MotionProperties] = None,
        mimic: Optional[Mimic] = None,
        meta: Optional[Dict[str, object]] = None,
    ) -> Articulation:
        return self.articulation(
            name,
            articulation_type=joint_type,
            parent=parent,
            child=child,
            origin=origin,
            axis=axis,
            motion_limits=limit,
            motion_properties=dynamics,
            mimic=mimic,
            meta=meta,
        )

    def material(
        self,
        name: str,
        *,
        rgba: Optional[Sequence[float]] = None,
        color: Optional[Sequence[float]] = None,
        texture: Optional[str] = None,
    ) -> Material:
        if rgba is not None and color is not None:
            raise ValidationError("Material cannot set both rgba and color")
        material = Material(
            name=name,
            rgba=_normalize_material_rgba(
                rgba if rgba is not None else color,
                field_name="Material rgba",
            ),
            texture=texture,
        )
        self.materials.append(material)
        return material

    def set_assets(
        self,
        assets: Optional[Union[AssetContext, AssetSession, str, Path]],
    ) -> Optional[AssetContext]:
        self.assets = resolve_asset_context(assets)
        for part in self.parts:
            part.assets = self.assets
        return self.assets

    def _rebuild_indices_if_needed(self) -> None:
        self._part_index = {part.name: part for part in self.parts}
        self._articulation_index = {
            articulation.name: articulation for articulation in self.articulations
        }

    def get_part(self, name: Union[str, Part]) -> Part:
        self._rebuild_indices_if_needed()
        key = _part_name_ref(name, field_name="name")
        part = self._part_index.get(key)
        if part is None:
            self._part_index = {p.name: p for p in self.parts}
            part = self._part_index.get(key)
        if part is None:
            raise ValidationError(f"Unknown part: {name!r}")
        return part

    def get_link(self, name: Union[str, Part]) -> Part:
        return self.get_part(name)

    def get_articulation(self, name: Union[str, Articulation]) -> Articulation:
        self._rebuild_indices_if_needed()
        key = _articulation_name_ref(name, field_name="name")
        articulation = self._articulation_index.get(key)
        if articulation is None:
            self._articulation_index = {a.name: a for a in self.articulations}
            articulation = self._articulation_index.get(key)
        if articulation is None:
            raise ValidationError(f"Unknown articulation: {name!r}")
        return articulation

    def get_joint(self, name: Union[str, Articulation]) -> Articulation:
        return self.get_articulation(name)

    def root_parts(self) -> List[Part]:
        child_names = {articulation.child for articulation in self.articulations}
        return [part for part in self.parts if part.name not in child_names]

    def root_links(self) -> List[Part]:
        return self.root_parts()

    def validate(
        self,
        *,
        strict: bool = True,
        strict_mesh_paths: bool = False,
    ) -> None:
        if not self.parts:
            raise ValidationError("Articulated object must contain at least one part")

        part_names = [part.name for part in self.parts]
        if len(set(part_names)) != len(part_names):
            raise ValidationError("Part names must be unique")

        articulation_names = [articulation.name for articulation in self.articulations]
        if len(set(articulation_names)) != len(articulation_names):
            raise ValidationError("Articulation names must be unique")
        articulation_lookup = {
            articulation.name: articulation for articulation in self.articulations
        }

        material_names = [material.name for material in self.materials]
        if len(set(material_names)) != len(material_names):
            raise ValidationError("Material names must be unique")
        material_name_set = set(material_names)
        for material in self.materials:
            _validate_material(material, f"material {material.name!r}")

        part_name_set = set(part_names)
        child_to_articulation: Dict[str, Articulation] = {}
        for articulation in self.articulations:
            if articulation.parent not in part_name_set:
                raise ValidationError(
                    f"Articulation {articulation.name!r} references missing parent part {articulation.parent!r}"
                )
            if articulation.child not in part_name_set:
                raise ValidationError(
                    f"Articulation {articulation.name!r} references missing child part {articulation.child!r}"
                )
            if articulation.parent == articulation.child:
                raise ValidationError(
                    f"Articulation {articulation.name!r} parent and child cannot be the same"
                )
            if articulation.child in child_to_articulation:
                raise ValidationError(
                    f"Part {articulation.child!r} has multiple parent articulations: "
                    f"{child_to_articulation[articulation.child].name!r} and {articulation.name!r}"
                )
            child_to_articulation[articulation.child] = articulation

            if strict and articulation.articulation_type in _SCALAR_ARTICULATION_TYPES:
                if len(articulation.axis) != 3:
                    raise ValidationError(
                        f"Articulation {articulation.name!r} axis must have 3 values"
                    )
                if _norm(articulation.axis) == 0.0:
                    raise ValidationError(
                        f"Articulation {articulation.name!r} axis must be non-zero"
                    )

            if strict and articulation.articulation_type in {
                ArticulationType.REVOLUTE,
                ArticulationType.PRISMATIC,
            }:
                if articulation.motion_limits is None:
                    raise ValidationError(
                        f"Articulation {articulation.name!r} must include motion limits"
                    )
                if (
                    articulation.motion_limits.lower is None
                    or articulation.motion_limits.upper is None
                ):
                    raise ValidationError(
                        f"Articulation {articulation.name!r} requires lower and upper limits"
                    )

            if strict and articulation.articulation_type == ArticulationType.CONTINUOUS:
                if articulation.motion_limits is None:
                    raise ValidationError(
                        f"Continuous articulation {articulation.name!r} must set "
                        "motion_limits=MotionLimits(effort=..., velocity=...) and "
                        "cannot set lower/upper limits"
                    )
                if (
                    articulation.motion_limits.lower is not None
                    or articulation.motion_limits.upper is not None
                ):
                    raise ValidationError(
                        f"Articulation {articulation.name!r} cannot include lower/upper limits"
                    )

            if (
                articulation.motion_limits
                and articulation.articulation_type not in _SCALAR_ARTICULATION_TYPES
            ):
                raise ValidationError(
                    f"Articulation {articulation.name!r} does not support motion limits"
                )

            if articulation.motion_limits:
                if (
                    articulation.motion_limits.effort <= 0
                    or articulation.motion_limits.velocity <= 0
                ):
                    raise ValidationError(
                        f"Articulation {articulation.name!r} limits must be positive"
                    )
                if (
                    articulation.motion_limits.lower is not None
                    and articulation.motion_limits.upper is not None
                    and articulation.motion_limits.lower > articulation.motion_limits.upper
                ):
                    raise ValidationError(
                        f"Articulation {articulation.name!r} lower limit exceeds upper limit"
                    )

        for articulation in self.articulations:
            mimic = articulation.mimic
            if mimic is None:
                continue
            if _motion_domain(articulation.articulation_type) is None:
                raise ValidationError(
                    f"Articulation {articulation.name!r} only supports mimic on "
                    "revolute, continuous, or prismatic articulations"
                )
            source = articulation_lookup.get(mimic.joint)
            if source is None:
                raise ValidationError(
                    f"Articulation {articulation.name!r} mimic references missing articulation "
                    f"{mimic.joint!r}"
                )
            if source.name == articulation.name:
                raise ValidationError(f"Articulation {articulation.name!r} cannot mimic itself")
            if _motion_domain(source.articulation_type) is None:
                raise ValidationError(
                    f"Articulation {articulation.name!r} mimic source {source.name!r} must be "
                    "revolute, continuous, or prismatic"
                )
            if _motion_domain(source.articulation_type) != _motion_domain(
                articulation.articulation_type
            ):
                raise ValidationError(
                    f"Articulation {articulation.name!r} mimic source {source.name!r} must have "
                    "a compatible motion domain"
                )

        _validate_mimic_cycles(articulation_lookup)

        for part in self.parts:
            if part.inertial and part.inertial.mass <= 0:
                raise ValidationError(f"Part {part.name!r} inertial mass must be positive")
            for visual in part.visuals:
                if visual.material is not None:
                    if isinstance(visual.material, str):
                        if str(visual.material) not in material_name_set:
                            raise ValidationError(
                                f"Part {part.name!r} visual references unknown material {visual.material!r}"
                            )
                    elif not isinstance(visual.material, Material):
                        raise ValidationError(
                            f"Part {part.name!r} visual material must be a Material or str name"
                        )
                    else:
                        _validate_material(
                            visual.material,
                            f"part {part.name!r} visual material {visual.material.name!r}",
                        )
                _validate_geometry(
                    visual.geometry,
                    f"part {part.name} visual",
                    strict_mesh_paths=strict_mesh_paths,
                )
            if part.collisions and not bool(getattr(self, _ALLOW_EXPLICIT_COLLISIONS_ATTR, False)):
                raise ValidationError(
                    f"Part {part.name!r} defines explicit collisions. "
                    "Source models must express geometry only through visuals and "
                    "primitive provenance instead of "
                    "authoring Part.collisions directly."
                )
            for collision in part.collisions:
                _validate_geometry(
                    collision.geometry,
                    f"part {part.name} collision",
                    strict_mesh_paths=strict_mesh_paths,
                )
        if strict:
            self._validate_connectivity(part_name_set)

    def to_urdf(
        self,
        *,
        pretty: bool = True,
        asset_root: Optional[Union[str, Path]] = None,
    ) -> str:
        """
        Compatibility shim for older generated scripts.

        URDF export is intentionally an internal implementation detail; new code
        should let the harness compile `object_model` directly.
        """

        from ._urdf_export import compile_object_to_urdf_xml

        return compile_object_to_urdf_xml(self, pretty=pretty, asset_root=asset_root)

    def _validate_connectivity(self, part_name_set: set[str]) -> None:
        if len(part_name_set) <= 1:
            return

        parent_to_children: Dict[str, List[str]] = {name: [] for name in part_name_set}
        child_set: set[str] = set()
        for articulation in self.articulations:
            parent_to_children.setdefault(articulation.parent, []).append(articulation.child)
            child_set.add(articulation.child)

        roots = sorted(part_name_set - child_set)
        if not roots:
            raise ValidationError(
                "Articulated object has no root part (cycle or every part is a child)"
            )
        if len(roots) > 1:
            raise ValidationError(
                "Articulated object must have exactly one root part; "
                f"found multiple root parts: {roots}"
            )

        visited: set[str] = set()
        queue: List[str] = list(roots)
        while queue:
            current = queue.pop()
            if current in visited:
                continue
            visited.add(current)
            queue.extend(parent_to_children.get(current, []))

        if visited != part_name_set:
            missing = sorted(part_name_set - visited)
            raise ValidationError(
                "Articulated object contains unreachable parts (cycle or broken articulations): "
                f"{missing}"
            )


def _norm(values: Sequence[float]) -> float:
    return sum(float(v) ** 2 for v in values) ** 0.5


def _motion_domain(articulation_type: ArticulationType) -> str | None:
    if articulation_type in _ANGULAR_ARTICULATION_TYPES:
        return "angular"
    if articulation_type == ArticulationType.PRISMATIC:
        return "linear"
    return None


def _validate_mimic_cycles(articulation_lookup: Dict[str, Articulation]) -> None:
    visiting: set[str] = set()
    visited: set[str] = set()
    path: list[str] = []

    def visit(name: str) -> None:
        if name in visited:
            return
        if name in visiting:
            cycle_start = path.index(name)
            cycle = path[cycle_start:] + [name]
            raise ValidationError(f"Mimic cycle detected: {' -> '.join(cycle)}")

        visiting.add(name)
        path.append(name)
        mimic = articulation_lookup[name].mimic
        if mimic is not None and mimic.joint in articulation_lookup:
            visit(mimic.joint)
        path.pop()
        visiting.remove(name)
        visited.add(name)

    for name in articulation_lookup:
        visit(name)


def _validate_geometry(
    geometry: Geometry,
    context: str,
    *,
    strict_mesh_paths: bool = False,
) -> None:
    if isinstance(geometry, Box):
        if len(geometry.size) != 3:
            raise ValidationError(f"{context} box size must have 3 values")
        if any(float(v) <= 0 for v in geometry.size):
            raise ValidationError(f"{context} box size must be positive")
    elif isinstance(geometry, Cylinder):
        if geometry.radius <= 0 or geometry.length <= 0:
            raise ValidationError(f"{context} cylinder radius/length must be positive")
    elif isinstance(geometry, Sphere):
        if geometry.radius <= 0:
            raise ValidationError(f"{context} sphere radius must be positive")
    elif isinstance(geometry, Mesh):
        if not geometry.filename and not geometry.name:
            raise ValidationError(f"{context} mesh filename or name is required")
        if strict_mesh_paths:
            filename = str(geometry.filename or "")
            if os.path.isabs(filename):
                raise ValidationError(
                    f"{context} mesh filename must be relative (got absolute path): {filename}"
                )
            if ".." in Path(filename).parts:
                raise ValidationError(
                    f"{context} mesh filename must not contain '..' segments: {filename}"
                )
        if geometry.scale:
            if len(geometry.scale) != 3:
                raise ValidationError(f"{context} mesh scale must have 3 values")
            if any(float(v) <= 0 for v in geometry.scale):
                raise ValidationError(f"{context} mesh scale must be positive")
    else:
        raise ValidationError(f"{context} has unsupported geometry type")


def _validate_material(material: Material, context: str) -> None:
    if not isinstance(material.name, str) or not material.name:
        raise ValidationError(f"{context} name is required")
    if material.rgba is not None and len(material.rgba) not in (3, 4):
        raise ValidationError(f"{context} rgba must have 3 or 4 values")
