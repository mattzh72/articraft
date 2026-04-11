from .articulated_object import ArticulatedObject
from .assets import mesh_from_input
from .errors import SDKError, ValidationError
from .geometry_scale import scale_geometry_to_size
from .mesh import (
    ArcPipeGeometry,
    BlowerWheelGeometry,
    BoxGeometry,
    CapsuleGeometry,
    ClevisBracketGeometry,
    ConeGeometry,
    CylinderGeometry,
    DomeGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    FanRotorGeometry,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    PerforatedPanelGeometry,
    PipeGeometry,
    PivotForkGeometry,
    SlotPatternPanelGeometry,
    SphereGeometry,
    SweepGeometry,
    TorusGeometry,
    TrunnionYokeGeometry,
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

_CADQUERY_EXPORTS = {
    "CadQueryMeshExport",
    "cadquery_local_aabb",
    "export_cadquery_components",
    "export_cadquery_mesh",
    "mesh_components_from_cadquery",
    "mesh_from_cadquery",
    "tessellate_cadquery",
}

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
    "PerforatedPanelGeometry",
    "SlotPatternPanelGeometry",
    "ClevisBracketGeometry",
    "PivotForkGeometry",
    "TrunnionYokeGeometry",
    "FanRotorGeometry",
    "BlowerWheelGeometry",
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
    "CadQueryMeshExport",
    "cadquery_local_aabb",
    "export_cadquery_components",
    "export_cadquery_mesh",
    "mesh_components_from_cadquery",
    "mesh_from_cadquery",
    "tessellate_cadquery",
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
]


def __getattr__(name: str):
    if name in _CADQUERY_EXPORTS:
        from sdk._extensions.cadquery import v0 as _cadquery_v0

        value = getattr(_cadquery_v0, name)
        globals()[name] = value
        return value
    if name in _GEAR_EXPORTS:
        from . import gears as _gears

        value = getattr(_gears, name)
        globals()[name] = value
        return value
    if name == "AssetContext":
        from .assets import AssetContext

        globals()[name] = AssetContext
        return AssetContext
    if name == "save_cadquery_obj":
        from sdk._extensions.cadquery.v0 import save_cadquery_obj

        globals()[name] = save_cadquery_obj
        return save_cadquery_obj
    if name == "LouverPanelGeometry":
        raise AttributeError(
            f"module {__name__!r} no longer exposes {name!r}; "
            "use 'VentGrilleGeometry' for public vent/grille mesh helpers"
        )
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
