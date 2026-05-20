from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _polar_xy(radius: float, angle_rad: float) -> tuple[float, float]:
    return (radius * math.cos(angle_rad), radius * math.sin(angle_rad))


def _midpoint(a: float, b: float) -> float:
    return 0.5 * (a + b)


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wing_revolving_door")

    polished_steel = model.material("polished_steel", rgba=(0.82, 0.85, 0.88, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.70, 0.74, 1.0))
    drum_glass = model.material("drum_glass", rgba=(0.78, 0.88, 0.94, 0.20))
    wing_glass = model.material("wing_glass", rgba=(0.72, 0.84, 0.90, 0.28))
    dark_floor = model.material("dark_floor", rgba=(0.20, 0.21, 0.23, 1.0))

    drum_radius = 1.92
    panel_radius = 1.78
    threshold_thickness = 0.08
    canopy_thickness = 0.18
    threshold_top = threshold_thickness
    canopy_bottom = 2.62
    canopy_center_z = canopy_bottom + canopy_thickness * 0.5
    sidewall_height = canopy_bottom - threshold_top
    sidewall_center_z = threshold_top + sidewall_height * 0.5

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=drum_radius, length=threshold_thickness),
        origin=Origin(xyz=(0.0, 0.0, threshold_thickness * 0.5)),
        material=dark_floor,
        name="threshold_disk",
    )
    drum.visual(
        Cylinder(radius=drum_radius, length=canopy_thickness),
        origin=Origin(xyz=(0.0, 0.0, canopy_center_z)),
        material=polished_steel,
        name="canopy_disk",
    )
    drum.visual(
        Cylinder(radius=2.00, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_steel,
        name="outer_threshold_band",
    )
    drum.visual(
        Cylinder(radius=2.00, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.84)),
        material=polished_steel,
        name="canopy_cap",
    )

    panel_step = math.radians(15.0)
    panel_width = 2.0 * panel_radius * math.sin(panel_step * 0.5) - 0.02
    side_panel_angles_deg = tuple(range(45, 136, 15)) + tuple(range(225, 316, 15))
    for index, angle_deg in enumerate(side_panel_angles_deg):
        angle = math.radians(angle_deg)
        x, y = _polar_xy(panel_radius, angle)
        drum.visual(
            Box((panel_width, 0.020, sidewall_height)),
            origin=Origin(
                xyz=(x, y, sidewall_center_z),
                rpy=(0.0, 0.0, angle + math.pi * 0.5),
            ),
            material=drum_glass,
            name=f"side_panel_{index:02d}",
        )

    mullion_angles_deg = (45, 90, 135, 225, 270, 315)
    for index, angle_deg in enumerate(mullion_angles_deg):
        x, y = _polar_xy(panel_radius, math.radians(angle_deg))
        drum.visual(
            Cylinder(radius=0.035, length=sidewall_height),
            origin=Origin(xyz=(x, y, sidewall_center_z)),
            material=polished_steel,
            name=f"mullion_{index:02d}",
        )

    drum.inertial = Inertial.from_geometry(
        Box((4.00, 4.00, 2.90)),
        mass=480.0,
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
    )

    wing_assembly = model.part("wing_assembly")
    post_radius = 0.085
    wing_outer_radius = 1.66
    wing_base_z = 0.18
    wing_height = 2.18
    wing_center_z = wing_base_z + wing_height * 0.5
    inner_stile_thickness = 0.070
    outer_stile_thickness = 0.050
    frame_depth = 0.060
    rail_height = 0.060

    wing_assembly.visual(
        Cylinder(radius=post_radius, length=canopy_bottom - threshold_top),
        origin=Origin(xyz=(0.0, 0.0, sidewall_center_z)),
        material=polished_steel,
        name="central_post",
    )
    wing_assembly.visual(
        Cylinder(radius=0.145, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=satin_steel,
        name="bottom_hub_collar",
    )
    wing_assembly.visual(
        Cylinder(radius=0.145, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 2.590)),
        material=satin_steel,
        name="top_hub_collar",
    )

    inner_stile_center_x = 0.102
    outer_stile_center_x = wing_outer_radius - outer_stile_thickness * 0.5
    rail_inner_x = inner_stile_center_x + inner_stile_thickness * 0.5 - 0.008
    rail_outer_x = wing_outer_radius
    rail_length = rail_outer_x - rail_inner_x
    rail_center_x = _midpoint(rail_inner_x, rail_outer_x)
    glass_inner_x = inner_stile_center_x + inner_stile_thickness * 0.5 - 0.012
    glass_outer_x = outer_stile_center_x - outer_stile_thickness * 0.5 + 0.012
    glass_length = glass_outer_x - glass_inner_x
    glass_center_x = _midpoint(glass_inner_x, glass_outer_x)

    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        wing_assembly.visual(
            Box((inner_stile_thickness, frame_depth, wing_height)),
            origin=Origin(
                xyz=(
                    inner_stile_center_x * math.cos(angle),
                    inner_stile_center_x * math.sin(angle),
                    wing_center_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=polished_steel,
            name=f"wing_inner_stile_{index}",
        )
        wing_assembly.visual(
            Box((outer_stile_thickness, frame_depth, wing_height)),
            origin=Origin(
                xyz=(
                    outer_stile_center_x * math.cos(angle),
                    outer_stile_center_x * math.sin(angle),
                    wing_center_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=polished_steel,
            name=f"wing_outer_stile_{index}",
        )
        wing_assembly.visual(
            Box((rail_length, frame_depth, rail_height)),
            origin=Origin(
                xyz=(
                    rail_center_x * math.cos(angle),
                    rail_center_x * math.sin(angle),
                    wing_base_z + rail_height * 0.5,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=polished_steel,
            name=f"wing_bottom_rail_{index}",
        )
        wing_assembly.visual(
            Box((rail_length, frame_depth, rail_height)),
            origin=Origin(
                xyz=(
                    rail_center_x * math.cos(angle),
                    rail_center_x * math.sin(angle),
                    wing_base_z + wing_height - rail_height * 0.5,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=polished_steel,
            name=f"wing_top_rail_{index}",
        )
        wing_assembly.visual(
            Box((glass_length, 0.014, wing_height - 0.10)),
            origin=Origin(
                xyz=(
                    glass_center_x * math.cos(angle),
                    glass_center_x * math.sin(angle),
                    wing_center_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=wing_glass,
            name=f"wing_glass_{index}",
        )

    wing_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=1.70, length=2.62),
        mass=160.0,
        origin=Origin(xyz=(0.0, 0.0, 1.31)),
    )

    model.articulation(
        "drum_to_wing_rotation",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=wing_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    drum = object_model.get_part("drum")
    wing_assembly = object_model.get_part("wing_assembly")
    rotation = object_model.get_articulation("drum_to_wing_rotation")

    threshold_disk = drum.get_visual("threshold_disk")
    canopy_disk = drum.get_visual("canopy_disk")
    wing_bottom_rail_0 = wing_assembly.get_visual("wing_bottom_rail_0")
    wing_top_rail_0 = wing_assembly.get_visual("wing_top_rail_0")
    outer_stile_0 = wing_assembly.get_visual("wing_outer_stile_0")

    limits = rotation.motion_limits
    ctx.check(
        "continuous vertical rotation joint is authored",
        rotation.articulation_type == ArticulationType.CONTINUOUS
        and rotation.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={rotation.articulation_type}, axis={rotation.axis}, "
            f"limits=({None if limits is None else limits.lower}, "
            f"{None if limits is None else limits.upper})"
        ),
    )

    with ctx.pose({rotation: 0.0}):
        ctx.expect_gap(
            wing_assembly,
            drum,
            axis="z",
            positive_elem=wing_bottom_rail_0,
            negative_elem=threshold_disk,
            min_gap=0.09,
            max_gap=0.13,
            name="wings sit just above the threshold",
        )
        ctx.expect_gap(
            drum,
            wing_assembly,
            axis="z",
            positive_elem=canopy_disk,
            negative_elem=wing_top_rail_0,
            min_gap=0.20,
            max_gap=0.30,
            name="wings clear the canopy underside",
        )
        ctx.expect_within(
            wing_assembly,
            drum,
            axes="xy",
            margin=0.0,
            name="wing assembly stays within the drum footprint at rest",
        )

    rest_center = _aabb_center(ctx.part_element_world_aabb(wing_assembly, elem=outer_stile_0))
    with ctx.pose({rotation: math.pi * 0.5}):
        ctx.expect_within(
            wing_assembly,
            drum,
            axes="xy",
            margin=0.0,
            name="wing assembly stays within the drum footprint while rotated",
        )
        turned_center = _aabb_center(ctx.part_element_world_aabb(wing_assembly, elem=outer_stile_0))

    moved_azimuthally = (
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 1.4
        and abs(rest_center[1]) < 0.08
        and turned_center[1] > 1.4
        and abs(turned_center[0]) < 0.08
    )
    ctx.check(
        "wing assembly rotates continuously about the central vertical post",
        moved_azimuthally,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
