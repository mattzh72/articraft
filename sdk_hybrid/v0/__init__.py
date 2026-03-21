from .articulated_object import ArticulatedObject
from .assets import AssetContext
from .cadquery import (
    CadQueryMeshExport,
    cadquery_local_aabb,
    export_cadquery_components,
    export_cadquery_mesh,
    mesh_components_from_cadquery,
    mesh_from_cadquery,
    save_cadquery_obj,
    tessellate_cadquery,
)
from .errors import SDKError, ValidationError
from .generated_collisions import (
    CollisionGenerationSettings,
    collision_generation_settings_from_env,
)
from .geometry_qc import (
    GeometryOverlap,
    UnsupportedPartFinding,
    default_contact_tol_from_env,
    default_overlap_tol_from_env,
    default_overlap_volume_tol_from_env,
    find_geometry_overlaps,
    find_unsupported_parts,
    validate_no_geometry_overlaps,
)
from .placement import (
    SurfaceFrame,
    SurfaceWrapMapping,
    align_centers_xy,
    part_local_aabb,
    place_in_front_of,
    place_on_face,
    place_on_face_uv,
    place_on_surface,
    place_on_top,
    proud_for_flush_mount,
    surface_frame,
    wrap_mesh_onto_surface,
    wrap_profile_onto_surface,
)
from .testing import TestContext, TestFailure, TestReport
from .types import (
    Articulation,
    ArticulationType,
    Box,
    Cylinder,
    Inertia,
    Inertial,
    Material,
    Mesh,
    MotionLimits,
    MotionProperties,
    Origin,
    Part,
    Sphere,
    Visual,
)

_GEAR_EXPORTS = {
    "GearBase",
    "SpurGear",
    "HerringboneGear",
    "RingGear",
    "HerringboneRingGear",
    "PlanetaryGearset",
    "HerringbonePlanetaryGearset",
    "BevelGear",
    "BevelGearPair",
    "RackGear",
    "HerringboneRackGear",
    "Worm",
    "CrossedHelicalGear",
    "CrossedGearPair",
    "HyperbolicGear",
    "HyperbolicGearPair",
    "gear",
    "addGear",
}

Link = Part
Joint = Articulation
JointType = ArticulationType
Limit = MotionLimits
Dynamics = MotionProperties

__all__ = [
    "SDKError",
    "ValidationError",
    "Link",
    "Joint",
    "JointType",
    "Limit",
    "Dynamics",
    "AssetContext",
    "CadQueryMeshExport",
    "cadquery_local_aabb",
    "export_cadquery_components",
    "export_cadquery_mesh",
    "mesh_components_from_cadquery",
    "tessellate_cadquery",
    "save_cadquery_obj",
    "mesh_from_cadquery",
    "ArticulatedObject",
    "GeometryOverlap",
    "UnsupportedPartFinding",
    "default_contact_tol_from_env",
    "default_overlap_tol_from_env",
    "default_overlap_volume_tol_from_env",
    "find_geometry_overlaps",
    "find_unsupported_parts",
    "validate_no_geometry_overlaps",
    "CollisionGenerationSettings",
    "collision_generation_settings_from_env",
    "TestContext",
    "TestFailure",
    "TestReport",
    "SurfaceFrame",
    "SurfaceWrapMapping",
    "align_centers_xy",
    "part_local_aabb",
    "place_in_front_of",
    "place_on_face",
    "place_on_face_uv",
    "place_on_surface",
    "place_on_top",
    "proud_for_flush_mount",
    "surface_frame",
    "wrap_profile_onto_surface",
    "wrap_mesh_onto_surface",
    "GearBase",
    "SpurGear",
    "HerringboneGear",
    "RingGear",
    "HerringboneRingGear",
    "PlanetaryGearset",
    "HerringbonePlanetaryGearset",
    "BevelGear",
    "BevelGearPair",
    "RackGear",
    "HerringboneRackGear",
    "Worm",
    "CrossedHelicalGear",
    "CrossedGearPair",
    "HyperbolicGear",
    "HyperbolicGearPair",
    "gear",
    "addGear",
    "Box",
    "Cylinder",
    "MotionProperties",
    "Inertia",
    "Inertial",
    "Articulation",
    "ArticulationType",
    "MotionLimits",
    "Material",
    "Mesh",
    "Origin",
    "Part",
    "Sphere",
    "Visual",
]


def __getattr__(name: str):
    if name in _GEAR_EXPORTS:
        from . import gears as _gears

        value = getattr(_gears, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
