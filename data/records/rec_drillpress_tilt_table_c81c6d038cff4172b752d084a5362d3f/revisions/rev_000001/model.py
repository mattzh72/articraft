from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_drill_press_stand")

    painted_base = model.material("painted_base", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_housing = model.material("dark_housing", rgba=(0.15, 0.15, 0.16, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))
    fence_gray = model.material("fence_gray", rgba=(0.56, 0.58, 0.60, 1.0))

    base_width = 0.30
    base_depth = 0.22
    base_thickness = 0.016
    column_mount_y = -0.070
    column_height = 0.420
    column_radius = 0.0175

    base = model.part("base")
    base.visual(
        Box((base_width, base_depth, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=painted_base,
        name="base_plate",
    )
    base.visual(
        Box((0.085, 0.055, 0.022)),
        origin=Origin(xyz=(0.0, column_mount_y, base_thickness + 0.011)),
        material=painted_base,
        name="column_plinth",
    )
    base.visual(
        Box((0.240, 0.065, 0.006)),
        origin=Origin(xyz=(0.0, 0.055, base_thickness + 0.003)),
        material=painted_base,
        name="fence_track",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=column_radius, length=column_height),
        origin=Origin(xyz=(0.0, 0.0, column_height / 2.0)),
        material=satin_steel,
        name="steel_column",
    )
    column.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=satin_steel,
        name="column_socket",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.020, 0.055, 0.110)),
        origin=Origin(xyz=(-0.0315, 0.0, 0.0)),
        material=dark_housing,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.020, 0.055, 0.110)),
        origin=Origin(xyz=(0.0315, 0.0, 0.0)),
        material=dark_housing,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.004, 0.032, 0.090)),
        origin=Origin(xyz=(-0.0195, 0.0, 0.0)),
        material=fence_gray,
        name="left_bushing",
    )
    carriage.visual(
        Box((0.004, 0.032, 0.090)),
        origin=Origin(xyz=(0.0195, 0.0, 0.0)),
        material=fence_gray,
        name="right_bushing",
    )
    carriage.visual(
        Box((0.083, 0.020, 0.110)),
        origin=Origin(xyz=(0.0, 0.0375, 0.0)),
        material=dark_housing,
        name="front_bridge",
    )
    carriage.visual(
        Box((0.048, 0.074, 0.050)),
        origin=Origin(xyz=(0.0, 0.084, 0.0)),
        material=dark_housing,
        name="feed_head",
    )
    carriage.visual(
        Box((0.050, 0.032, 0.026)),
        origin=Origin(xyz=(0.0, 0.116, -0.020)),
        material=dark_housing,
        name="spindle_mount",
    )

    locking_collar = model.part("locking_collar")
    locking_collar.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="clamp_stem",
    )
    locking_collar.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=knob_black,
        name="clamp_knob",
    )
    locking_collar.visual(
        Cylinder(radius=0.003, length=0.028),
        origin=Origin(xyz=(0.023, 0.0, 0.0)),
        material=knob_black,
        name="clamp_bar",
    )

    chuck = model.part("chuck")
    chuck.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=satin_steel,
        name="mount_hub",
    )
    chuck.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=dark_housing,
        name="quick_release_sleeve",
    )
    chuck.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=satin_steel,
        name="chuck_body",
    )
    chuck.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.076)),
        material=satin_steel,
        name="nose",
    )

    fence = model.part("fence")
    fence.visual(
        Box((0.160, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_housing,
        name="slider_foot",
    )
    fence.visual(
        Box((0.150, 0.006, 0.070)),
        origin=Origin(xyz=(0.0, 0.017, 0.045)),
        material=fence_gray,
        name="fence_plate",
    )
    fence.visual(
        Box((0.020, 0.022, 0.032)),
        origin=Origin(xyz=(-0.050, 0.008, 0.021)),
        material=fence_gray,
        name="left_gusset",
    )
    fence.visual(
        Box((0.020, 0.022, 0.032)),
        origin=Origin(xyz=(0.050, 0.008, 0.021)),
        material=fence_gray,
        name="right_gusset",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, column_mount_y, 0.038)),
    )
    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.18,
            lower=0.0,
            upper=0.160,
        ),
    )
    model.articulation(
        "carriage_to_locking_collar",
        ArticulationType.FIXED,
        parent=carriage,
        child=locking_collar,
        origin=Origin(xyz=(0.0415, 0.0, 0.0)),
    )
    model.articulation(
        "carriage_to_chuck",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=chuck,
        origin=Origin(xyz=(0.0, 0.116, -0.033)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=8.0,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "base_to_fence",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fence,
        origin=Origin(xyz=(0.0, 0.055, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.12,
            lower=-0.065,
            upper=0.065,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    locking_collar = object_model.get_part("locking_collar")
    chuck = object_model.get_part("chuck")
    fence = object_model.get_part("fence")

    carriage_slide = object_model.get_articulation("column_to_carriage")
    chuck_spin = object_model.get_articulation("carriage_to_chuck")
    fence_slide = object_model.get_articulation("base_to_fence")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        column,
        base,
        elem_a="column_socket",
        elem_b="column_plinth",
        contact_tol=5e-4,
        name="column socket seats on the base plinth",
    )
    ctx.expect_contact(
        carriage,
        column,
        elem_a="left_bushing",
        elem_b="steel_column",
        contact_tol=5e-4,
        name="left carriage bushing bears on the column",
    )
    ctx.expect_contact(
        carriage,
        column,
        elem_a="right_bushing",
        elem_b="steel_column",
        contact_tol=5e-4,
        name="right carriage bushing bears on the column",
    )
    ctx.expect_contact(
        locking_collar,
        carriage,
        elem_a="clamp_stem",
        elem_b="right_cheek",
        contact_tol=5e-4,
        name="locking collar fastens to the carriage body",
    )
    ctx.expect_contact(
        chuck,
        carriage,
        elem_a="mount_hub",
        elem_b="spindle_mount",
        contact_tol=5e-4,
        name="quick release chuck mounts under the carriage",
    )
    ctx.expect_contact(
        fence,
        base,
        elem_a="slider_foot",
        elem_b="fence_track",
        contact_tol=5e-4,
        name="fence slider rides on the base track",
    )

    ctx.check(
        "carriage slide joint is vertical prismatic motion",
        carriage_slide.articulation_type == ArticulationType.PRISMATIC
        and carriage_slide.axis == (0.0, 0.0, 1.0),
        details=f"type={carriage_slide.articulation_type}, axis={carriage_slide.axis}",
    )
    ctx.check(
        "chuck joint is revolute around the drill axis",
        chuck_spin.articulation_type == ArticulationType.REVOLUTE
        and chuck_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={chuck_spin.articulation_type}, axis={chuck_spin.axis}",
    )
    ctx.check(
        "fence slide joint moves laterally across the base",
        fence_slide.articulation_type == ArticulationType.PRISMATIC
        and fence_slide.axis == (1.0, 0.0, 0.0),
        details=f"type={fence_slide.articulation_type}, axis={fence_slide.axis}",
    )

    carriage_rest = ctx.part_world_position(carriage)
    ctx.expect_gap(
        chuck,
        base,
        axis="z",
        min_gap=0.020,
        max_gap=0.120,
        positive_elem="nose",
        negative_elem="base_plate",
        name="lowest drilling nose still clears the base plate",
    )
    with ctx.pose({carriage_slide: carriage_slide.motion_limits.upper}):
        carriage_raised = ctx.part_world_position(carriage)

    ctx.check(
        "carriage rises on positive travel",
        carriage_rest is not None
        and carriage_raised is not None
        and carriage_raised[2] > carriage_rest[2] + 0.100,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )

    fence_rest = ctx.part_world_position(fence)
    with ctx.pose({fence_slide: fence_slide.motion_limits.lower}):
        ctx.expect_within(
            fence,
            base,
            axes="x",
            inner_elem="slider_foot",
            outer_elem="base_plate",
            margin=0.0,
            name="fence stays on the base at left travel",
        )
    with ctx.pose({fence_slide: fence_slide.motion_limits.upper}):
        fence_right = ctx.part_world_position(fence)
        ctx.expect_within(
            fence,
            base,
            axes="x",
            inner_elem="slider_foot",
            outer_elem="base_plate",
            margin=0.0,
            name="fence stays on the base at right travel",
        )

    ctx.check(
        "fence moves to the right on positive travel",
        fence_rest is not None
        and fence_right is not None
        and fence_right[0] > fence_rest[0] + 0.050,
        details=f"rest={fence_rest}, moved={fence_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
