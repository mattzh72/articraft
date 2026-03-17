from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Union

from .assets import AssetContext, coerce_asset_context, resolve_asset_root
from .errors import ValidationError
from .geometry_qc import (
    default_overlap_tol_from_env,
    default_overlap_volume_tol_from_env,
    _prefer_collisions_from_source,
    validate_no_geometry_overlaps,
)
from .types import (
    Articulation,
    ArticulationType,
    Box,
    Collision,
    Cylinder,
    Geometry,
    Inertial,
    Material,
    MaterialRef,
    Mesh,
    MotionLimits,
    MotionProperties,
    Origin,
    Part,
    Sphere,
    Vec3,
    Visual,
)


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
    _articulation_index: Dict[str, Articulation] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self) -> None:
        self.assets = coerce_asset_context(self.assets)
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
        rgba_values = rgba if rgba is not None else color
        if rgba_values is not None and len(rgba_values) not in (3, 4):
            raise ValidationError("Material rgba must have 3 or 4 values")
        if rgba_values is not None and len(rgba_values) == 3:
            rgba_values = tuple(rgba_values) + (1.0,)
        material = Material(
            name=name,
            rgba=tuple(float(v) for v in rgba_values) if rgba_values is not None else None,
            texture=texture,
        )
        self.materials.append(material)
        return material

    def set_assets(
        self,
        assets: Optional[Union[AssetContext, str, Path]],
    ) -> Optional[AssetContext]:
        self.assets = coerce_asset_context(assets)
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

            if strict and articulation.articulation_type in {
                ArticulationType.REVOLUTE,
                ArticulationType.PRISMATIC,
                ArticulationType.CONTINUOUS,
            }:
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
                        f"Articulation {articulation.name!r} must include effort and velocity limits"
                    )
                if (
                    articulation.motion_limits.lower is not None
                    or articulation.motion_limits.upper is not None
                ):
                    raise ValidationError(
                        f"Articulation {articulation.name!r} cannot include lower/upper limits"
                    )

            if articulation.motion_limits and articulation.articulation_type not in {
                ArticulationType.REVOLUTE,
                ArticulationType.PRISMATIC,
                ArticulationType.CONTINUOUS,
            }:
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

        for part in self.parts:
            if part.inertial and part.inertial.mass <= 0:
                raise ValidationError(
                    f"Part {part.name!r} inertial mass must be positive"
                )
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
            for collision in part.collisions:
                _validate_geometry(
                    collision.geometry,
                    f"part {part.name} collision",
                    strict_mesh_paths=strict_mesh_paths,
                )
        if strict:
            self._validate_connectivity(part_name_set)

    def validate_geometry(
        self,
        *,
        asset_root: Optional[Union[str, Path]] = None,
        geometry_source: str = "collision",
        max_pose_samples: int = 256,
        overlap_tol: Optional[float] = None,
        overlap_volume_tol: Optional[float] = None,
        ignore_adjacent: bool = True,
        ignore_fixed: bool = True,
        allowed_pairs: Optional[Iterable[tuple[str, str]]] = None,
        seed: int = 0,
    ) -> None:
        """Validate that part geometry does not self-intersect across sampled articulation poses."""

        self.validate(strict=True)

        root_path = resolve_asset_root(asset_root, self)
        prefer_collisions = _prefer_collisions_from_source(geometry_source)
        tol = default_overlap_tol_from_env() if overlap_tol is None else float(overlap_tol)
        vol_tol = (
            default_overlap_volume_tol_from_env()
            if overlap_volume_tol is None
            else float(overlap_volume_tol)
        )
        resolved_allowed_pairs: Optional[List[tuple[str, str]]] = None
        if allowed_pairs is not None:
            resolved_allowed_pairs = [
                (
                    _part_name_ref(pair[0], field_name="allowed_pairs parent"),
                    _part_name_ref(pair[1], field_name="allowed_pairs child"),
                )
                for pair in allowed_pairs
            ]
        elif ignore_adjacent or ignore_fixed:
            pairs: list[tuple[str, str]] = []
            for articulation in self.articulations:
                if ignore_adjacent or (
                    ignore_fixed and articulation.articulation_type == ArticulationType.FIXED
                ):
                    pairs.append((articulation.parent, articulation.child))
            resolved_allowed_pairs = pairs or None

        validate_no_geometry_overlaps(
            self,
            asset_root=root_path,
            geometry_source="collision" if prefer_collisions else "visual",
            max_pose_samples=max_pose_samples,
            overlap_tol=tol,
            overlap_volume_tol=vol_tol,
            allowed_pairs=resolved_allowed_pairs,
            seed=int(seed),
        )

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
        if not geometry.filename:
            raise ValidationError(f"{context} mesh filename is required")
        if strict_mesh_paths:
            filename = str(geometry.filename)
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
