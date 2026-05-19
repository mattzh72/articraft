from __future__ import annotations

import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple, Union

from .assets import AssetContext, coerce_asset_context
from .errors import ValidationError

Vec3 = Tuple[float, float, float]
Mat4 = Tuple[Tuple[float, float, float, float], ...]


def _as_vec3(values: Sequence[float], *, name: str) -> Vec3:
    if len(values) != 3:
        raise ValidationError(f"{name} must have 3 elements")
    return (float(values[0]), float(values[1]), float(values[2]))


def _as_mat4(values: Sequence[Sequence[float]], *, name: str) -> Mat4:
    if len(values) != 4:
        raise ValidationError(f"{name} must have 4 rows")
    rows: list[tuple[float, float, float, float]] = []
    for idx, row in enumerate(values):
        if len(row) != 4:
            raise ValidationError(f"{name}[{idx}] must have 4 elements")
        rows.append((float(row[0]), float(row[1]), float(row[2]), float(row[3])))
    return (rows[0], rows[1], rows[2], rows[3])


@dataclass(frozen=True)
class Origin:
    xyz: Vec3 = (0.0, 0.0, 0.0)
    rpy: Vec3 = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "xyz", _as_vec3(self.xyz, name="origin.xyz"))
        object.__setattr__(self, "rpy", _as_vec3(self.rpy, name="origin.rpy"))


@dataclass(frozen=True)
class Box:
    size: Vec3


@dataclass(frozen=True, init=False)
class Cylinder:
    radius: float
    length: float

    def __init__(
        self,
        radius: float,
        length: float | None = None,
        *,
        height: float | None = None,
    ) -> None:
        if length is not None and height is not None:
            raise TypeError("Cylinder() accepts only one of 'length' or alias 'height'")
        resolved_length = height if length is None else length
        if resolved_length is None:
            raise TypeError("Cylinder() missing required argument: 'length'")
        object.__setattr__(self, "radius", float(radius))
        object.__setattr__(self, "length", float(resolved_length))

    @property
    def height(self) -> float:
        return self.length


@dataclass(frozen=True)
class Sphere:
    radius: float


PrimitiveGeometry = Union[Box, Cylinder, Sphere]


@dataclass(frozen=True)
class Mesh:
    filename: Union[str, os.PathLike[str], None] = None
    name: Optional[str] = None
    scale: Optional[Vec3] = None
    source_geometry: Optional[PrimitiveGeometry] = None
    source_transform: Optional[Mat4] = None
    materialized_path: Optional[str] = field(default=None, compare=False, repr=False)

    def __post_init__(self) -> None:
        filename_value = self.filename
        filename = os.fspath(filename_value) if filename_value is not None else ""
        logical_name = None if self.name is None else str(self.name).strip()
        if not filename and not logical_name:
            raise ValidationError("mesh.filename or mesh.name is required")
        if filename:
            object.__setattr__(self, "filename", filename)
        else:
            object.__setattr__(self, "filename", None)
        if logical_name is not None:
            if not logical_name:
                raise ValidationError("mesh.name must be non-empty when provided")
            object.__setattr__(self, "name", logical_name)
        if self.scale is not None:
            object.__setattr__(self, "scale", _as_vec3(self.scale, name="mesh.scale"))
        if self.source_geometry is not None and not isinstance(
            self.source_geometry, (Box, Cylinder, Sphere)
        ):
            raise ValidationError("mesh.source_geometry must be a Box, Cylinder, Sphere, or None")
        if self.source_transform is not None:
            if self.source_geometry is None:
                raise ValidationError("mesh.source_transform requires mesh.source_geometry")
            object.__setattr__(
                self,
                "source_transform",
                _as_mat4(self.source_transform, name="mesh.source_transform"),
            )
        if self.materialized_path is not None:
            object.__setattr__(self, "materialized_path", os.fspath(self.materialized_path))


Geometry = Union[Box, Cylinder, Sphere, Mesh]


def _normalize_material_rgba(
    rgba: Optional[Sequence[float]],
    *,
    field_name: str = "material.rgba",
) -> Optional[Tuple[float, float, float, float]]:
    if rgba is None:
        return None
    normalized = tuple(float(v) for v in rgba)
    if len(normalized) not in (3, 4):
        raise ValidationError(f"{field_name} must have 3 or 4 values")
    if len(normalized) == 3:
        normalized = normalized + (1.0,)
    return normalized


@dataclass(init=False)
class Material:
    name: str
    rgba: Optional[Tuple[float, float, float, float]] = None
    texture: Optional[str] = None

    def __init__(
        self,
        name: str,
        rgba: Optional[Sequence[float]] = None,
        texture: Optional[str] = None,
        *,
        color: Optional[Sequence[float]] = None,
    ) -> None:
        if rgba is not None and color is not None:
            raise ValidationError("Material cannot set both rgba and color")
        self.name = str(name)
        self.rgba = _normalize_material_rgba(rgba if rgba is not None else color)
        self.texture = texture
        self.__post_init__()

    def __post_init__(self) -> None:
        self.name = str(self.name)
        if not self.name:
            raise ValidationError("material.name is required")
        self.rgba = _normalize_material_rgba(self.rgba)


MaterialRef = Union[Material, str]


def _coerce_visual_material_ref(
    *,
    material: Optional[MaterialRef],
    color: object,
) -> Optional[MaterialRef]:
    if material is not None and color is not None:
        raise TypeError("Part.visual() accepts only one of 'material' or alias 'color'")
    if color is None:
        return material
    if isinstance(color, (Material, str)):
        return color
    if isinstance(color, Sequence) and not isinstance(color, (str, bytes)):
        return Material(name="inline_color", rgba=tuple(float(v) for v in color))
    raise TypeError(
        "Part.visual() alias 'color' must be a material name, Material, or 3/4-value color tuple"
    )


@dataclass
class Visual:
    geometry: Geometry
    origin: Origin = field(default_factory=Origin)
    material: Optional[MaterialRef] = None
    name: Optional[str] = None


@dataclass
class Collision:
    geometry: Geometry
    origin: Origin = field(default_factory=Origin)
    name: Optional[str] = None


@dataclass(frozen=True)
class Inertia:
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float


@dataclass
class Inertial:
    mass: float
    inertia: Inertia
    origin: Origin = field(default_factory=Origin)

    @staticmethod
    def from_geometry(
        geometry: Geometry, mass: float, *, origin: Optional[Origin] = None
    ) -> "Inertial":
        mass = float(mass)
        if mass <= 0:
            raise ValidationError("Mass must be positive")

        if isinstance(geometry, Box):
            x, y, z = _as_vec3(geometry.size, name="box.size")
            ixx = (1.0 / 12.0) * mass * (y * y + z * z)
            iyy = (1.0 / 12.0) * mass * (x * x + z * z)
            izz = (1.0 / 12.0) * mass * (x * x + y * y)
        elif isinstance(geometry, Cylinder):
            r = float(geometry.radius)
            length = float(geometry.length)
            ixx = (1.0 / 12.0) * mass * (3 * r * r + length * length)
            iyy = ixx
            izz = 0.5 * mass * r * r
        elif isinstance(geometry, Sphere):
            r = float(geometry.radius)
            ixx = (2.0 / 5.0) * mass * r * r
            iyy = ixx
            izz = ixx
        else:
            raise ValidationError("Cannot derive inertia for mesh geometry")

        inertia = Inertia(ixx=ixx, ixy=0.0, ixz=0.0, iyy=iyy, iyz=0.0, izz=izz)
        return Inertial(mass=mass, inertia=inertia, origin=origin or Origin())


class ArticulationType(str, Enum):
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FIXED = "fixed"
    FLOATING = "floating"


def _coerce_articulation_type(
    value: Union["ArticulationType", str],
) -> ArticulationType:
    try:
        return value if isinstance(value, ArticulationType) else ArticulationType(str(value))
    except ValueError as exc:
        raise ValidationError(f"Unknown articulation type: {value}") from exc


@dataclass(frozen=True)
class MotionLimits:
    effort: float = 1.0
    velocity: float = 1.0
    lower: Optional[float] = None
    upper: Optional[float] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "effort", float(self.effort))
        object.__setattr__(self, "velocity", float(self.velocity))
        if self.lower is not None:
            object.__setattr__(self, "lower", float(self.lower))
        if self.upper is not None:
            object.__setattr__(self, "upper", float(self.upper))


@dataclass(frozen=True)
class MotionProperties:
    damping: Optional[float] = None
    friction: Optional[float] = None


@dataclass(frozen=True)
class Mimic:
    joint: str
    multiplier: float = 1.0
    offset: float = 0.0

    def __post_init__(self) -> None:
        joint_name = str(self.joint).strip()
        if not joint_name:
            raise ValidationError("mimic.joint is required")
        object.__setattr__(self, "joint", joint_name)
        object.__setattr__(self, "multiplier", float(self.multiplier))
        object.__setattr__(self, "offset", float(self.offset))


@dataclass
class Part:
    name: str
    visuals: List[Visual] = field(default_factory=list)
    collisions: List[Collision] = field(default_factory=list)
    inertial: Optional[Inertial] = None
    meta: Dict[str, object] = field(default_factory=dict)
    assets: Optional[AssetContext] = None

    def __post_init__(self) -> None:
        self.assets = coerce_asset_context(self.assets)

    def visual(
        self,
        geometry: Geometry,
        *,
        origin: Optional[Origin] = None,
        material: Optional[MaterialRef] = None,
        color: object = None,
        name: Optional[str] = None,
    ) -> Visual:
        visual = Visual(
            geometry=geometry,
            origin=origin or Origin(),
            material=_coerce_visual_material_ref(material=material, color=color),
            name=name,
        )
        self.visuals.append(visual)
        return visual

    def get_visual(self, name: str) -> Visual:
        key = str(name).strip()
        if not key:
            raise ValidationError("visual name is required")
        for visual in self.visuals:
            if visual.name == key:
                return visual
        raise ValidationError(f"Unknown visual on part {self.name!r}: {name!r}")


@dataclass(eq=False)
class Articulation:
    name: str
    articulation_type: Union[ArticulationType, str]
    parent: str
    child: str
    origin: Origin = field(default_factory=Origin)
    axis: Vec3 = (0.0, 0.0, 1.0)
    motion_limits: Optional[MotionLimits] = None
    motion_properties: Optional[MotionProperties] = None
    mimic: Optional[Mimic] = None
    meta: Dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.articulation_type = _coerce_articulation_type(self.articulation_type)
        self.axis = _as_vec3(self.axis, name="articulation.axis")
        if self.mimic is not None and not isinstance(self.mimic, Mimic):
            if isinstance(self.mimic, str):
                self.mimic = Mimic(joint=self.mimic)
            else:
                joint = getattr(self.mimic, "joint", None)
                if joint is None:
                    raise ValidationError("articulation.mimic must be a Mimic or mimic-like object")
                self.mimic = Mimic(
                    joint=joint,
                    multiplier=getattr(self.mimic, "multiplier", 1.0),
                    offset=getattr(self.mimic, "offset", 0.0),
                )

    @property
    def joint_type(self) -> ArticulationType:
        return self.articulation_type

    @joint_type.setter
    def joint_type(self, value: Union[ArticulationType, str]) -> None:
        self.articulation_type = _coerce_articulation_type(value)

    @property
    def limit(self) -> Optional[MotionLimits]:
        return self.motion_limits

    @limit.setter
    def limit(self, value: Optional[MotionLimits]) -> None:
        self.motion_limits = value

    @property
    def dynamics(self) -> Optional[MotionProperties]:
        return self.motion_properties

    @dynamics.setter
    def dynamics(self, value: Optional[MotionProperties]) -> None:
        self.motion_properties = value
