from .assets import AssetContext
from .errors import SDKError, ValidationError
from .cadquery import (
    CadQueryMeshExport,
    cadquery_local_aabb,
    export_cadquery_mesh,
    mesh_from_cadquery,
    save_cadquery_obj,
    tessellate_cadquery,
)
from .articulated_object import ArticulatedObject
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
from .generated_collisions import CollisionGenerationSettings, collision_generation_settings_from_env
from .testing import TestContext, TestFailure, TestReport
from .placement import (
    align_centers_xy,
    part_local_aabb,
    place_in_front_of,
    place_on_face,
    place_on_face_uv,
    place_on_top,
    proud_for_flush_mount,
)
from .types import (
    Box,
    Cylinder,
    MotionProperties,
    Inertia,
    Inertial,
    Articulation,
    ArticulationType,
    MotionLimits,
    Material,
    Mesh,
    Origin,
    Part,
    Sphere,
    Visual,
)

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
    "export_cadquery_mesh",
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
    "align_centers_xy",
    "part_local_aabb",
    "place_in_front_of",
    "place_on_face",
    "place_on_face_uv",
    "place_on_top",
    "proud_for_flush_mount",
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
