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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _max_axis_delta(a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> float:
    if a is None or b is None:
        return float("inf")
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knuckle_boom_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.93, 0.76, 0.14, 1.0))
    mast_gray = model.material("mast_gray", rgba=(0.32, 0.35, 0.39, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    hook_black = model.material("hook_black", rgba=(0.10, 0.10, 0.11, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((1.45, 1.45, 0.45)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=mast_gray,
        name="base_pedestal",
    )
    mast.visual(
        Box((0.82, 0.82, 6.00)),
        origin=Origin(xyz=(0.0, 0.0, 3.45)),
        material=mast_gray,
        name="tower_column",
    )
    mast.visual(
        Box((1.00, 1.00, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 6.59)),
        material=steel_dark,
        name="machinery_deck",
    )
    mast.visual(
        Box((0.72, 0.82, 0.46)),
        origin=Origin(xyz=(-0.50, 0.0, 6.67)),
        material=steel_dark,
        name="counterweight_pack",
    )
    mast.visual(
        Box((0.24, 0.08, 0.42)),
        origin=Origin(xyz=(0.60, 0.23, 6.54)),
        material=crane_yellow,
        name="left_hinge_cheek",
    )
    mast.visual(
        Box((0.24, 0.08, 0.42)),
        origin=Origin(xyz=(0.60, -0.23, 6.54)),
        material=crane_yellow,
        name="right_hinge_cheek",
    )
    mast.visual(
        Box((0.16, 0.54, 0.16)),
        origin=Origin(xyz=(0.48, 0.0, 6.72)),
        material=steel_dark,
        name="hinge_crosshead",
    )

    main_boom = model.part("main_boom")
    main_boom.visual(
        Cylinder(radius=0.09, length=0.38),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=crane_yellow,
        name="main_hinge_barrel",
    )
    main_boom.visual(
        Box((0.50, 0.28, 0.30)),
        origin=Origin(xyz=(0.25, 0.0, 0.0)),
        material=crane_yellow,
        name="main_root_saddle",
    )
    main_boom.visual(
        Box((3.20, 0.42, 0.50)),
        origin=Origin(xyz=(1.85, 0.0, 0.0)),
        material=crane_yellow,
        name="main_box_section",
    )
    main_boom.visual(
        Box((1.60, 0.30, 0.36)),
        origin=Origin(xyz=(4.10, 0.0, 0.0)),
        material=crane_yellow,
        name="main_tip_section",
    )
    main_boom.visual(
        Box((0.24, 0.07, 0.32)),
        origin=Origin(xyz=(4.98, 0.16, 0.0)),
        material=crane_yellow,
        name="main_left_tip_fork",
    )
    main_boom.visual(
        Box((0.24, 0.07, 0.32)),
        origin=Origin(xyz=(4.98, -0.16, 0.0)),
        material=crane_yellow,
        name="main_right_tip_fork",
    )

    knuckle_boom = model.part("knuckle_boom")
    knuckle_boom.visual(
        Cylinder(radius=0.07, length=0.25),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=crane_yellow,
        name="knuckle_hinge_barrel",
    )
    knuckle_boom.visual(
        Box((0.44, 0.22, 0.24)),
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        material=crane_yellow,
        name="knuckle_root_saddle",
    )
    knuckle_boom.visual(
        Box((2.20, 0.20, 0.38)),
        origin=Origin(xyz=(1.12, 0.0, 0.0)),
        material=crane_yellow,
        name="knuckle_box_section",
    )
    knuckle_boom.visual(
        Box((1.05, 0.22, 0.28)),
        origin=Origin(xyz=(2.72, 0.0, 0.0)),
        material=crane_yellow,
        name="knuckle_tip_section",
    )
    knuckle_boom.visual(
        Box((0.22, 0.22, 0.18)),
        origin=Origin(xyz=(3.14, 0.0, 0.0)),
        material=crane_yellow,
        name="tip_head",
    )
    knuckle_boom.visual(
        Box((0.08, 0.03, 0.20)),
        origin=Origin(xyz=(3.18, 0.085, -0.10)),
        material=steel_dark,
        name="swivel_left_strap",
    )
    knuckle_boom.visual(
        Box((0.08, 0.03, 0.20)),
        origin=Origin(xyz=(3.18, -0.085, -0.10)),
        material=steel_dark,
        name="swivel_right_strap",
    )
    knuckle_boom.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(xyz=(3.18, 0.0, -0.14)),
        material=steel_dark,
        name="swivel_housing",
    )

    hook_block = model.part("hook_block")
    hook_block.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=steel_dark,
        name="swivel_spindle",
    )
    hook_block.visual(
        Box((0.22, 0.14, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, -0.20)),
        material=hook_black,
        name="upper_block",
    )
    hook_block.visual(
        Box((0.18, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.38)),
        material=hook_black,
        name="lower_block",
    )
    hook_block.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, -0.50)),
        material=steel_dark,
        name="hook_stem",
    )
    hook_block.visual(
        Sphere(radius=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.57)),
        material=steel_dark,
        name="hook_neck",
    )
    hook_profile = tube_from_spline_points(
        [
            (0.0, 0.0, -0.59),
            (0.0, 0.0, -0.72),
            (0.05, 0.0, -0.84),
            (0.14, 0.0, -0.81),
            (0.15, 0.0, -0.69),
            (0.06, 0.0, -0.60),
        ],
        radius=0.022,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    hook_block.visual(
        mesh_from_geometry(hook_profile, "hook_curve"),
        material=hook_black,
        name="hook_curve",
    )

    model.articulation(
        "mast_to_main_boom",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=main_boom,
        origin=Origin(xyz=(0.60, 0.0, 6.54)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=260000.0,
            velocity=0.35,
            lower=-0.20,
            upper=1.10,
        ),
    )
    model.articulation(
        "main_to_knuckle",
        ArticulationType.REVOLUTE,
        parent=main_boom,
        child=knuckle_boom,
        origin=Origin(xyz=(4.98, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160000.0,
            velocity=0.50,
            lower=-1.60,
            upper=1.35,
        ),
    )
    model.articulation(
        "knuckle_to_hook_swivel",
        ArticulationType.REVOLUTE,
        parent=knuckle_boom,
        child=hook_block,
        origin=Origin(xyz=(3.18, 0.0, -0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=2.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    main_boom = object_model.get_part("main_boom")
    knuckle_boom = object_model.get_part("knuckle_boom")
    hook_block = object_model.get_part("hook_block")
    main_joint = object_model.get_articulation("mast_to_main_boom")
    knuckle_joint = object_model.get_articulation("main_to_knuckle")
    hook_joint = object_model.get_articulation("knuckle_to_hook_swivel")

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
        main_boom,
        mast,
        elem_a="main_hinge_barrel",
        elem_b="left_hinge_cheek",
        name="main boom barrel bears on mast clevis",
    )
    ctx.expect_contact(
        knuckle_boom,
        main_boom,
        elem_a="knuckle_hinge_barrel",
        elem_b="main_left_tip_fork",
        name="knuckle barrel bears on main boom fork",
    )
    ctx.expect_contact(
        hook_block,
        knuckle_boom,
        elem_a="swivel_spindle",
        elem_b="swivel_housing",
        name="hook swivel spindle seats under tip housing",
    )

    ctx.check(
        "boom hinges use horizontal pitch axes",
        _max_axis_delta(main_joint.axis, (0.0, -1.0, 0.0)) < 1e-9
        and _max_axis_delta(knuckle_joint.axis, (0.0, -1.0, 0.0)) < 1e-9,
        details=f"main_axis={main_joint.axis}, knuckle_axis={knuckle_joint.axis}",
    )
    ctx.check(
        "hook swivel uses local vertical axis",
        _max_axis_delta(hook_joint.axis, (0.0, 0.0, 1.0)) < 1e-9,
        details=f"hook_axis={hook_joint.axis}",
    )

    knuckle_rest = ctx.part_world_position(knuckle_boom)
    with ctx.pose({main_joint: 0.90}):
        knuckle_raised = ctx.part_world_position(knuckle_boom)
    ctx.check(
        "main boom lifts the knuckle hinge above the mast top",
        knuckle_rest is not None
        and knuckle_raised is not None
        and knuckle_raised[2] > knuckle_rest[2] + 3.0,
        details=f"rest={knuckle_rest}, raised={knuckle_raised}",
    )

    with ctx.pose({main_joint: 0.55, knuckle_joint: 0.0}):
        hook_low = ctx.part_world_position(hook_block)
    with ctx.pose({main_joint: 0.55, knuckle_joint: 0.85}):
        hook_high = ctx.part_world_position(hook_block)
    ctx.check(
        "knuckle joint raises the hook block",
        hook_low is not None and hook_high is not None and hook_high[2] > hook_low[2] + 1.2,
        details=f"low={hook_low}, high={hook_high}",
    )

    with ctx.pose({main_joint: 0.55, knuckle_joint: 0.55, hook_joint: -1.20}):
        swivel_pos_a = ctx.part_world_position(hook_block)
    with ctx.pose({main_joint: 0.55, knuckle_joint: 0.55, hook_joint: 1.20}):
        swivel_pos_b = ctx.part_world_position(hook_block)
    ctx.check(
        "hook swivel rotates in place at the secondary tip",
        _max_axis_delta(swivel_pos_a, swivel_pos_b) < 1e-6,
        details=f"pose_a={swivel_pos_a}, pose_b={swivel_pos_b}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
