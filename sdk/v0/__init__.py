from .articulated_object import ArticulatedObject
from .assets import mesh_from_input
from .errors import SDKError, ValidationError
from .geometry_scale import scale_geometry_to_size
from .mesh import (
    ArcPipeGeometry,
    BoxGeometry,
    CapsuleGeometry,
    ConeGeometry,
    CylinderGeometry,
    DomeGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    PipeGeometry,
    SphereGeometry,
    SweepGeometry,
    TorusGeometry,
    VentGrilleGeometry,
    WirePath,
    WirePolylineGeometry,
    boolean_difference,
    boolean_intersection,
    boolean_union,
    cut_opening_on_face,
    mesh_from_geometry,
    resample_side_sections,
    rounded_rect_profile,
    sample_arc_3d,
    sample_catmull_rom_spline_2d,
    sample_catmull_rom_spline_3d,
    sample_cubic_bezier_spline_2d,
    sample_cubic_bezier_spline_3d,
    split_superellipse_side_loft,
    superellipse_profile,
    superellipse_side_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
    wire_from_points,
)
from .placement import (
    SurfaceFrame,
    SurfaceWrapMapping,
    align_centers,
    part_local_aabb,
    place_on_face,
    place_on_face_uv,
    place_on_surface,
    proud_for_flush_mount,
    surface_frame,
    wrap_mesh_onto_surface,
    wrap_profile_onto_surface,
)
from .section_loft import (
    LoftSection,
    LoftTessellation,
    SectionLoftSpec,
    repair_loft,
    section_loft,
)
from .shell_partition import (
    ShellPartitionRegion,
    ShellPartitionSpec,
    partition_shell,
)
from .testing import AllowedOverlap, TestContext, TestFailure, TestReport
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

__all__ = [
    "SDKError",
    "ValidationError",
    "scale_geometry_to_size",
    "MeshGeometry",
    "WirePath",
    "BoxGeometry",
    "CapsuleGeometry",
    "CylinderGeometry",
    "ConeGeometry",
    "DomeGeometry",
    "SphereGeometry",
    "ExtrudeGeometry",
    "ExtrudeWithHolesGeometry",
    "LoftGeometry",
    "SweepGeometry",
    "PipeGeometry",
    "ArcPipeGeometry",
    "WirePolylineGeometry",
    "TorusGeometry",
    "LatheGeometry",
    "VentGrilleGeometry",
    "cut_opening_on_face",
    "boolean_union",
    "boolean_difference",
    "boolean_intersection",
    "wire_from_points",
    "tube_from_spline_points",
    "sweep_profile_along_spline",
    "mesh_from_geometry",
    "mesh_from_input",
    "LoftSection",
    "LoftTessellation",
    "SectionLoftSpec",
    "section_loft",
    "repair_loft",
    "ShellPartitionRegion",
    "ShellPartitionSpec",
    "partition_shell",
    "sample_cubic_bezier_spline_2d",
    "sample_cubic_bezier_spline_3d",
    "sample_catmull_rom_spline_2d",
    "sample_catmull_rom_spline_3d",
    "sample_arc_3d",
    "superellipse_side_loft",
    "split_superellipse_side_loft",
    "resample_side_sections",
    "rounded_rect_profile",
    "superellipse_profile",
    "ArticulatedObject",
    "AllowedOverlap",
    "TestContext",
    "TestFailure",
    "TestReport",
    "align_centers",
    "SurfaceFrame",
    "SurfaceWrapMapping",
    "part_local_aabb",
    "place_on_face",
    "place_on_face_uv",
    "place_on_surface",
    "proud_for_flush_mount",
    "surface_frame",
    "wrap_profile_onto_surface",
    "wrap_mesh_onto_surface",
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
    if name == "AssetContext":
        from .assets import AssetContext

        globals()[name] = AssetContext
        return AssetContext
    if name == "LouverPanelGeometry":
        raise AttributeError(
            f"module {__name__!r} no longer exposes {name!r}; "
            "use 'VentGrilleGeometry' for public vent/grille mesh helpers"
        )
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
