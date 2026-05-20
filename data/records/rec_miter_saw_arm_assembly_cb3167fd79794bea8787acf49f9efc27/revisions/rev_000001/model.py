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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_compound_miter_saw")

    dark_body = model.material("dark_body", rgba=(0.24, 0.25, 0.27, 1.0))
    darker_body = model.material("darker_body", rgba=(0.16, 0.17, 0.19, 1.0))
    cast_aluminum = model.material("cast_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.82, 0.84, 0.87, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.85, 0.86, 0.88, 1.0))
    accent_red = model.material("accent_red", rgba=(0.72, 0.16, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.48, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_body,
        name="lower_base",
    )
    base.visual(
        Box((0.54, 0.22, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=darker_body,
        name="upper_base_shell",
    )
    base.visual(
        Box((0.30, 0.20, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=cast_aluminum,
        name="pivot_plinth",
    )
    base.visual(
        Box((0.28, 0.08, 0.022)),
        origin=Origin(xyz=(0.0, 0.17, 0.054)),
        material=cast_aluminum,
        name="front_control_beam",
    )
    for x in (-0.27, 0.27):
        for y in (-0.16, 0.16):
            base.visual(
                Box((0.09, 0.06, 0.010)),
                origin=Origin(xyz=(x, y, 0.005)),
                material=rubber,
                name=f"foot_{'r' if x > 0 else 'l'}_{'f' if y > 0 else 'r'}",
            )
    base.visual(
        Cylinder(radius=0.016, length=0.090),
        origin=Origin(xyz=(0.0, 0.205, 0.082), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_red,
        name="miter_lock_handle",
    )
    base.visual(
        Box((0.020, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.205, 0.066)),
        material=accent_red,
        name="miter_lock_stem",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.48, 0.135)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
    )

    table_assembly = model.part("table_assembly")
    table_assembly.visual(
        Cylinder(radius=0.17, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=cast_aluminum,
        name="turntable_disc",
    )
    table_assembly.visual(
        Box((0.10, 0.16, 0.004)),
        origin=Origin(xyz=(0.0, 0.015, 0.010)),
        material=darker_body,
        name="table_insert",
    )
    table_assembly.visual(
        Box((0.27, 0.17, 0.016)),
        origin=Origin(xyz=(-0.22, -0.005, 0.008)),
        material=cast_aluminum,
        name="left_table_wing",
    )
    table_assembly.visual(
        Box((0.27, 0.17, 0.016)),
        origin=Origin(xyz=(0.22, -0.005, 0.008)),
        material=cast_aluminum,
        name="right_table_wing",
    )
    table_assembly.visual(
        Box((0.28, 0.026, 0.060)),
        origin=Origin(xyz=(-0.17, -0.185, 0.046)),
        material=cast_aluminum,
        name="left_fence_face",
    )
    table_assembly.visual(
        Box((0.28, 0.026, 0.060)),
        origin=Origin(xyz=(0.17, -0.185, 0.046)),
        material=cast_aluminum,
        name="right_fence_face",
    )
    table_assembly.visual(
        Box((0.64, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, -0.160, 0.008)),
        material=darker_body,
        name="rear_fence_base",
    )
    table_assembly.visual(
        Box((0.20, 0.11, 0.080)),
        origin=Origin(xyz=(0.0, -0.150, 0.050)),
        material=cast_aluminum,
        name="rear_casting",
    )
    table_assembly.visual(
        Box((0.05, 0.08, 0.120)),
        origin=Origin(xyz=(-0.090, -0.200, 0.100)),
        material=cast_aluminum,
        name="left_rail_upright",
    )
    table_assembly.visual(
        Box((0.05, 0.08, 0.120)),
        origin=Origin(xyz=(0.090, -0.200, 0.100)),
        material=cast_aluminum,
        name="right_rail_upright",
    )
    table_assembly.visual(
        Box((0.20, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, -0.025, 0.105)),
        material=cast_aluminum,
        name="front_rail_boss",
    )
    table_assembly.visual(
        Box((0.24, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, -0.365, 0.115)),
        material=cast_aluminum,
        name="rear_rail_bridge",
    )
    table_assembly.visual(
        Cylinder(radius=0.011, length=0.480),
        origin=Origin(xyz=(-0.060, -0.200, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="left_rail",
    )
    table_assembly.visual(
        Cylinder(radius=0.011, length=0.480),
        origin=Origin(xyz=(0.060, -0.200, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="right_rail",
    )
    table_assembly.inertial = Inertial.from_geometry(
        Box((0.66, 0.42, 0.180)),
        mass=7.0,
        origin=Origin(xyz=(0.0, -0.08, 0.090)),
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        Box((0.050, 0.120, 0.016)),
        origin=Origin(xyz=(-0.060, -0.080, -0.026)),
        material=darker_body,
        name="left_saddle_top",
    )
    slide_carriage.visual(
        Box((0.008, 0.120, 0.030)),
        origin=Origin(xyz=(-0.075, -0.080, -0.032)),
        material=darker_body,
        name="left_saddle_outer",
    )
    slide_carriage.visual(
        Box((0.008, 0.120, 0.030)),
        origin=Origin(xyz=(-0.045, -0.080, -0.032)),
        material=darker_body,
        name="left_saddle_inner",
    )
    slide_carriage.visual(
        Box((0.050, 0.120, 0.016)),
        origin=Origin(xyz=(0.060, -0.080, -0.026)),
        material=darker_body,
        name="right_saddle_top",
    )
    slide_carriage.visual(
        Box((0.008, 0.120, 0.030)),
        origin=Origin(xyz=(0.045, -0.080, -0.032)),
        material=darker_body,
        name="right_saddle_inner",
    )
    slide_carriage.visual(
        Box((0.008, 0.120, 0.030)),
        origin=Origin(xyz=(0.075, -0.080, -0.032)),
        material=darker_body,
        name="right_saddle_outer",
    )
    slide_carriage.visual(
        Box((0.18, 0.090, 0.022)),
        origin=Origin(xyz=(0.0, -0.030, -0.007)),
        material=dark_body,
        name="carriage_bridge",
    )
    slide_carriage.visual(
        Box((0.14, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, -0.005, 0.018)),
        material=accent_red,
        name="pivot_mount",
    )
    slide_carriage.visual(
        Box((0.022, 0.060, 0.085)),
        origin=Origin(xyz=(-0.050, 0.000, 0.060)),
        material=dark_body,
        name="left_hinge_cheek",
    )
    slide_carriage.visual(
        Box((0.022, 0.060, 0.085)),
        origin=Origin(xyz=(0.050, 0.000, 0.060)),
        material=dark_body,
        name="right_hinge_cheek",
    )
    slide_carriage.visual(
        Box((0.120, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, -0.050, 0.100)),
        material=dark_body,
        name="top_cross_tie",
    )
    slide_carriage.inertial = Inertial.from_geometry(
        Box((0.20, 0.16, 0.12)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -0.040, 0.000)),
    )

    saw_head = model.part("saw_head")
    arm_angle = -0.55
    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.034, 0.045, 0.060),
                (-0.028, 0.075, 0.075),
                (0.000, 0.100, 0.082),
                (0.028, 0.075, 0.075),
                (0.034, 0.045, 0.060),
            ],
            radius=0.008,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
        "miter_saw_top_handle",
    )
    saw_head.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    saw_head.visual(
        Box((0.078, 0.185, 0.050)),
        origin=Origin(xyz=(0.0, 0.082, -0.047), rpy=(arm_angle, 0.0, 0.0)),
        material=accent_red,
        name="arm_beam",
    )
    saw_head.visual(
        Box((0.072, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.040, -0.018), rpy=(arm_angle, 0.0, 0.0)),
        material=darker_body,
        name="arm_shoulder",
    )
    saw_head.visual(
        Box((0.090, 0.095, 0.105)),
        origin=Origin(xyz=(0.055, 0.155, -0.085)),
        material=darker_body,
        name="gearbox_housing",
    )
    saw_head.visual(
        Cylinder(radius=0.055, length=0.165),
        origin=Origin(xyz=(0.135, 0.155, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_red,
        name="motor_housing",
    )
    saw_head.visual(
        Cylinder(radius=0.034, length=0.055),
        origin=Origin(xyz=(0.238, 0.155, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=darker_body,
        name="motor_endcap",
    )
    saw_head.visual(
        Cylinder(radius=0.100, length=0.055),
        origin=Origin(xyz=(0.032, 0.185, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_aluminum,
        name="blade_guard",
    )
    saw_head.visual(
        Box((0.085, 0.120, 0.055)),
        origin=Origin(xyz=(0.055, 0.090, -0.012), rpy=(0.20, 0.0, 0.0)),
        material=cast_aluminum,
        name="guard_crown",
    )
    saw_head.visual(
        Cylinder(radius=0.127, length=0.003),
        origin=Origin(xyz=(-0.006, 0.155, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_steel,
        name="blade_disc",
    )
    saw_head.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.016, 0.155, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="blade_hub",
    )
    saw_head.visual(
        handle_mesh,
        material=darker_body,
        name="top_handle",
    )
    saw_head.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.0, 0.103, 0.038), rpy=(0.0, 0.0, 0.16)),
        material=rubber,
        name="grip_bar",
    )
    saw_head.visual(
        Box((0.032, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, 0.118, 0.010)),
        material=darker_body,
        name="trigger_handle",
    )
    saw_head.visual(
        Box((0.028, 0.070, 0.070)),
        origin=Origin(xyz=(0.018, 0.138, -0.045)),
        material=cast_aluminum,
        name="guard_side_plate",
    )
    saw_head.inertial = Inertial.from_geometry(
        Box((0.34, 0.28, 0.24)),
        mass=6.0,
        origin=Origin(xyz=(0.10, 0.12, -0.030)),
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.3,
            lower=-math.radians(50.0),
            upper=math.radians(50.0),
        ),
    )
    model.articulation(
        "table_to_slide",
        ArticulationType.PRISMATIC,
        parent=table_assembly,
        child=slide_carriage,
        origin=Origin(xyz=(0.0, -0.010, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.30,
            lower=0.0,
            upper=0.140,
        ),
    )
    model.articulation(
        "slide_to_head",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=saw_head,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=0.0,
            upper=0.95,
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

    base = object_model.get_part("base")
    table_assembly = object_model.get_part("table_assembly")
    slide_carriage = object_model.get_part("slide_carriage")
    saw_head = object_model.get_part("saw_head")

    base_to_table = object_model.get_articulation("base_to_table")
    table_to_slide = object_model.get_articulation("table_to_slide")
    slide_to_head = object_model.get_articulation("slide_to_head")

    ctx.expect_gap(
        table_assembly,
        base,
        axis="z",
        positive_elem="turntable_disc",
        negative_elem="pivot_plinth",
        max_gap=0.001,
        max_penetration=1e-6,
        name="turntable seats on pivot plinth",
    )

    with ctx.pose({table_to_slide: 0.0}):
        ctx.expect_gap(
            slide_carriage,
            table_assembly,
            axis="z",
            positive_elem="left_saddle_top",
            negative_elem="left_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="left carriage clamp seats on the left rail at rest",
        )
        ctx.expect_gap(
            slide_carriage,
            table_assembly,
            axis="z",
            positive_elem="right_saddle_top",
            negative_elem="right_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="right carriage clamp seats on the right rail at rest",
        )

    slide_upper = table_to_slide.motion_limits.upper if table_to_slide.motion_limits else 0.0
    with ctx.pose({table_to_slide: slide_upper}):
        ctx.expect_gap(
            slide_carriage,
            table_assembly,
            axis="z",
            positive_elem="left_saddle_top",
            negative_elem="left_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="left carriage clamp stays seated while extended",
        )
        ctx.expect_gap(
            slide_carriage,
            table_assembly,
            axis="z",
            positive_elem="right_saddle_top",
            negative_elem="right_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="right carriage clamp stays seated while extended",
        )
        ctx.expect_overlap(
            slide_carriage,
            table_assembly,
            axes="y",
            elem_a="left_saddle_top",
            elem_b="left_rail",
            min_overlap=0.045,
            name="left rail retains insertion at full slide extension",
        )
        ctx.expect_overlap(
            slide_carriage,
            table_assembly,
            axes="y",
            elem_a="right_saddle_top",
            elem_b="right_rail",
            min_overlap=0.045,
            name="right rail retains insertion at full slide extension",
        )

    rest_head_pos = ctx.part_world_position(saw_head)
    with ctx.pose({table_to_slide: slide_upper}):
        extended_head_pos = ctx.part_world_position(saw_head)
    ctx.check(
        "head assembly slides forward along the twin rails",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[1] > rest_head_pos[1] + 0.10,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    def blade_center_xyz() -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(saw_head, elem="blade_disc")
        if aabb is None:
            return None
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    rest_blade_xyz = blade_center_xyz()
    with ctx.pose({base_to_table: math.radians(35.0)}):
        turned_blade_xyz = blade_center_xyz()
    ctx.check(
        "miter table rotates the upper assembly about a vertical axis",
        rest_blade_xyz is not None
        and turned_blade_xyz is not None
        and abs(turned_blade_xyz[0] - rest_blade_xyz[0]) > 0.08,
        details=f"rest={rest_blade_xyz}, turned={turned_blade_xyz}",
    )

    def blade_center_z() -> float | None:
        center = blade_center_xyz()
        if center is None:
            return None
        return center[2]

    rest_blade_z = blade_center_z()
    chop_upper = slide_to_head.motion_limits.upper if slide_to_head.motion_limits else 0.0
    with ctx.pose({slide_to_head: chop_upper}):
        chopped_blade_z = blade_center_z()
    ctx.check(
        "chop hinge lowers the blade toward the table",
        rest_blade_z is not None
        and chopped_blade_z is not None
        and chopped_blade_z < rest_blade_z - 0.08,
        details=f"rest_z={rest_blade_z}, chopped_z={chopped_blade_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
