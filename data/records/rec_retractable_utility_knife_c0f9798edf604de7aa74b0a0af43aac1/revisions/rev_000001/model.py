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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _profile_from_yz(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-z, y) for y, z in points]


def _u_channel_mesh(
    *,
    length: float,
    outer_width: float,
    outer_height: float,
    wall_thickness: float,
    bottom_thickness: float,
):
    inner_width = outer_width - 2.0 * wall_thickness
    inner_floor_z = -outer_height * 0.5 + bottom_thickness
    yz_profile = [
        (-outer_width * 0.5, outer_height * 0.5),
        (-inner_width * 0.5, outer_height * 0.5),
        (-inner_width * 0.5, inner_floor_z),
        (inner_width * 0.5, inner_floor_z),
        (inner_width * 0.5, outer_height * 0.5),
        (outer_width * 0.5, outer_height * 0.5),
        (outer_width * 0.5, -outer_height * 0.5),
        (-outer_width * 0.5, -outer_height * 0.5),
    ]
    return mesh_from_geometry(
        ExtrudeGeometry(_profile_from_yz(yz_profile), length).rotate_y(math.pi / 2.0),
        "handle_channel_shell",
    )


def _blade_mesh(thickness: float):
    blade_profile = [
        (-0.010, 0.0046),
        (0.010, 0.0058),
        (0.046, 0.0052),
        (0.073, 0.0008),
        (0.047, -0.0070),
        (0.016, -0.0037),
        (-0.002, -0.0037),
        (-0.010, 0.0046),
    ]
    return mesh_from_geometry(
        ExtrudeGeometry(blade_profile, thickness).rotate_x(-math.pi / 2.0),
        "utility_blade_plate",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_utility_knife")

    body_metal = model.material("body_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    darker_metal = model.material("darker_metal", rgba=(0.44, 0.46, 0.49, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.86, 0.88, 0.90, 1.0))
    slider_polymer = model.material("slider_polymer", rgba=(0.14, 0.15, 0.16, 1.0))

    shell_length = 0.118
    shell_width = 0.024
    shell_height = 0.014
    wall_thickness = 0.0016
    bottom_thickness = 0.0018

    handle_shell = model.part("handle_shell")
    handle_shell.visual(
        _u_channel_mesh(
            length=shell_length,
            outer_width=shell_width,
            outer_height=shell_height,
            wall_thickness=wall_thickness,
            bottom_thickness=bottom_thickness,
        ),
        origin=Origin(xyz=(-0.001, 0.0, 0.0)),
        material=body_metal,
        name="channel_shell",
    )
    handle_shell.visual(
        Box((0.014, 0.0046, 0.0092)),
        origin=Origin(xyz=(0.065, -0.0088, -0.0004)),
        material=body_metal,
        name="nose_cheek_left",
    )
    handle_shell.visual(
        Box((0.014, 0.0046, 0.0092)),
        origin=Origin(xyz=(0.065, 0.0088, -0.0004)),
        material=body_metal,
        name="nose_cheek_right",
    )
    handle_shell.visual(
        Box((0.014, 0.0162, 0.0018)),
        origin=Origin(xyz=(0.065, 0.0, 0.0061)),
        material=body_metal,
        name="nose_bridge",
    )
    handle_shell.visual(
        Box((0.010, 0.0026, 0.0036)),
        origin=Origin(xyz=(0.064, -0.0062, 0.0049)),
        material=body_metal,
        name="nose_gusset_left",
    )
    handle_shell.visual(
        Box((0.010, 0.0026, 0.0036)),
        origin=Origin(xyz=(0.064, 0.0062, 0.0049)),
        material=body_metal,
        name="nose_gusset_right",
    )
    handle_shell.visual(
        Box((0.012, 0.0208, 0.0018)),
        origin=Origin(xyz=(-0.053, 0.0, 0.0058)),
        material=darker_metal,
        name="rear_bridge",
    )
    handle_shell.visual(
        Box((0.006, 0.0020, 0.0060)),
        origin=Origin(xyz=(-0.062, -0.0110, 0.0045)),
        material=darker_metal,
        name="pivot_ear_left",
    )
    handle_shell.visual(
        Box((0.006, 0.0020, 0.0060)),
        origin=Origin(xyz=(-0.062, 0.0110, 0.0045)),
        material=darker_metal,
        name="pivot_ear_right",
    )
    handle_shell.visual(
        Box((0.030, 0.018, 0.0012)),
        origin=Origin(xyz=(-0.010, 0.0, -0.0068)),
        material=darker_metal,
        name="belly_reinforcement",
    )
    handle_shell.inertial = Inertial.from_geometry(
        Box((0.144, shell_width, shell_height)),
        mass=0.24,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    blade_carriage = model.part("blade_carriage")
    blade_carriage.visual(
        Box((0.034, 0.0184, 0.0078)),
        origin=Origin(xyz=(0.0, 0.0, -0.0012)),
        material=darker_metal,
        name="carriage_body",
    )
    blade_carriage.visual(
        Box((0.010, 0.010, 0.0032)),
        origin=Origin(xyz=(-0.006, 0.0, -0.0046)),
        material=darker_metal,
        name="carriage_tail_shoe",
    )
    blade_carriage.visual(
        Box((0.014, 0.010, 0.0030)),
        origin=Origin(xyz=(0.017, 0.0, 0.0008)),
        material=darker_metal,
        name="blade_clamp",
    )
    blade_carriage.visual(
        Box((0.013, 0.008, 0.0080)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0067)),
        material=slider_polymer,
        name="thumb_slider",
    )
    blade_carriage.visual(
        _blade_mesh(0.0008),
        origin=Origin(xyz=(0.010, 0.0, -0.0018)),
        material=blade_steel,
        name="utility_blade",
    )
    blade_carriage.inertial = Inertial.from_geometry(
        Box((0.084, 0.019, 0.014)),
        mass=0.06,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
    )

    rear_cap_latch = model.part("rear_cap_latch")
    rear_cap_latch.visual(
        Cylinder(radius=0.0020, length=0.0210),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_metal,
        name="latch_barrel",
    )
    rear_cap_latch.visual(
        Box((0.0024, 0.0190, 0.0100)),
        origin=Origin(xyz=(-0.0024, 0.0, -0.0055)),
        material=body_metal,
        name="tail_cap",
    )
    rear_cap_latch.visual(
        Box((0.0052, 0.0100, 0.0026)),
        origin=Origin(xyz=(-0.0054, 0.0, -0.0035)),
        material=darker_metal,
        name="latch_tab",
    )
    rear_cap_latch.inertial = Inertial.from_geometry(
        Box((0.010, 0.021, 0.014)),
        mass=0.018,
        origin=Origin(xyz=(-0.002, 0.0, -0.004)),
    )

    model.articulation(
        "handle_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=blade_carriage,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=-0.018,
            upper=0.018,
        ),
    )
    model.articulation(
        "handle_to_rear_cap_latch",
        ArticulationType.REVOLUTE,
        parent=handle_shell,
        child=rear_cap_latch,
        origin=Origin(xyz=(-0.061, 0.0, 0.0045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
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

    handle_shell = object_model.get_part("handle_shell")
    blade_carriage = object_model.get_part("blade_carriage")
    rear_cap_latch = object_model.get_part("rear_cap_latch")
    slide = object_model.get_articulation("handle_to_blade_carriage")
    latch = object_model.get_articulation("handle_to_rear_cap_latch")

    ctx.check(
        "all major parts are present",
        all(part is not None for part in (handle_shell, blade_carriage, rear_cap_latch)),
    )
    ctx.check(
        "blade carriage slides along the handle axis",
        tuple(round(v, 3) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "rear latch pivots across the handle width",
        tuple(round(v, 3) for v in latch.axis) == (0.0, 1.0, 0.0),
        details=f"axis={latch.axis}",
    )

    with ctx.pose({slide: 0.0, latch: 0.0}):
        ctx.expect_within(
            blade_carriage,
            handle_shell,
            axes="yz",
            inner_elem="carriage_body",
            outer_elem="channel_shell",
            margin=0.0006,
            name="carriage body stays guided within the shell channel",
        )
        ctx.expect_overlap(
            blade_carriage,
            handle_shell,
            axes="x",
            elem_a="carriage_body",
            elem_b="channel_shell",
            min_overlap=0.022,
            name="carriage remains inserted in the handle at nominal pose",
        )
        ctx.expect_contact(
            rear_cap_latch,
            handle_shell,
            elem_a="latch_barrel",
            name="rear latch barrel seats against the tail hinge ears",
        )

    handle_aabb = ctx.part_world_aabb(handle_shell)
    with ctx.pose({slide: slide.motion_limits.lower, latch: 0.0}):
        retracted_blade_aabb = ctx.part_element_world_aabb(blade_carriage, elem="utility_blade")
    with ctx.pose({slide: slide.motion_limits.upper, latch: 0.0}):
        extended_blade_aabb = ctx.part_element_world_aabb(blade_carriage, elem="utility_blade")
        ctx.expect_overlap(
            blade_carriage,
            handle_shell,
            axes="x",
            elem_a="carriage_body",
            elem_b="channel_shell",
            min_overlap=0.010,
            name="carriage retains guided insertion at full blade extension",
        )

    ctx.check(
        "blade retracts behind the shell nose",
        handle_aabb is not None
        and retracted_blade_aabb is not None
        and retracted_blade_aabb[1][0] < handle_aabb[1][0] + 0.0005,
        details=f"handle={handle_aabb}, retracted_blade={retracted_blade_aabb}",
    )
    ctx.check(
        "blade extends beyond the shell nose",
        handle_aabb is not None
        and extended_blade_aabb is not None
        and extended_blade_aabb[1][0] > handle_aabb[1][0] + 0.014,
        details=f"handle={handle_aabb}, extended_blade={extended_blade_aabb}",
    )

    with ctx.pose({latch: 0.0}):
        closed_cap_aabb = ctx.part_element_world_aabb(rear_cap_latch, elem="tail_cap")
    with ctx.pose({latch: latch.motion_limits.upper}):
        open_cap_aabb = ctx.part_element_world_aabb(rear_cap_latch, elem="tail_cap")
    ctx.check(
        "rear cap latch swings up and back when opened",
        closed_cap_aabb is not None
        and open_cap_aabb is not None
        and open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.003
        and open_cap_aabb[0][0] < closed_cap_aabb[0][0] - 0.002,
        details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
