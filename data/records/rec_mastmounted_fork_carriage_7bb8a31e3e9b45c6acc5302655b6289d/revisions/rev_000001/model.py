from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def make_channel_rail(height: float, *, open_positive_x: bool):
    web_x = -0.034 if open_positive_x else 0.034
    flange_x = 0.016 if open_positive_x else -0.016
    rail = cq.Workplane("XY").box(0.032, 0.12, height)
    rail = rail.union(
        cq.Workplane("XY").box(0.068, 0.022, height).translate((flange_x, 0.049, 0.0))
    )
    rail = rail.union(
        cq.Workplane("XY").box(0.068, 0.022, height).translate((flange_x, -0.049, 0.0))
    )
    rail = rail.translate((web_x, 0.0, 0.0))
    return rail


def make_fork_tine():
    return (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.36, -0.02),
                (0.22, -0.02),
                (0.32, -0.012),
                (0.36, 0.0),
                (0.32, 0.02),
                (-0.36, 0.02),
            ]
        )
        .close()
        .extrude(0.08, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_bridge_lift_mast")

    frame_paint = model.material("frame_paint", color=(0.22, 0.25, 0.28, 1.0))
    carriage_paint = model.material("carriage_paint", color=(0.70, 0.68, 0.64, 1.0))
    fork_steel = model.material("fork_steel", color=(0.34, 0.34, 0.36, 1.0))
    wear_pad = model.material("wear_pad", color=(0.08, 0.08, 0.08, 1.0))

    mast_frame = model.part("mast_frame")

    add_box(
        mast_frame,
        "base_beam",
        (1.06, 0.22, 0.12),
        (0.0, -0.11, 0.06),
        frame_paint.name,
    )
    mast_frame.visual(
        mesh_from_cadquery(make_channel_rail(2.12, open_positive_x=True), "left_rail"),
        origin=Origin(xyz=(-0.36, -0.03, 1.15)),
        material=frame_paint.name,
        name="left_rail",
    )
    mast_frame.visual(
        mesh_from_cadquery(make_channel_rail(2.12, open_positive_x=False), "right_rail"),
        origin=Origin(xyz=(0.36, -0.03, 1.15)),
        material=frame_paint.name,
        name="right_rail",
    )
    add_box(
        mast_frame,
        "top_tie",
        (0.92, 0.14, 0.12),
        (0.0, -0.03, 2.26),
        frame_paint.name,
    )
    add_box(
        mast_frame,
        "left_guide_strip",
        (0.025, 0.08, 2.04),
        (-0.3025, -0.03, 1.19),
        wear_pad.name,
    )
    add_box(
        mast_frame,
        "right_guide_strip",
        (0.025, 0.08, 2.04),
        (0.3025, -0.03, 1.19),
        wear_pad.name,
    )
    add_box(
        mast_frame,
        "left_base_gusset",
        (0.14, 0.12, 0.28),
        (-0.43, -0.14, 0.20),
        frame_paint.name,
    )
    add_box(
        mast_frame,
        "right_base_gusset",
        (0.14, 0.12, 0.28),
        (0.43, -0.14, 0.20),
        frame_paint.name,
    )

    fork_carriage = model.part("fork_carriage")

    add_box(
        fork_carriage,
        "carriage_lower",
        (0.48, 0.08, 0.08),
        (0.0, -0.02, 0.0),
        carriage_paint.name,
    )
    add_box(
        fork_carriage,
        "carriage_backplate",
        (0.56, 0.04, 0.58),
        (0.0, -0.01, 0.31),
        carriage_paint.name,
    )
    add_box(
        fork_carriage,
        "carriage_top",
        (0.48, 0.08, 0.08),
        (0.0, -0.02, 0.60),
        carriage_paint.name,
    )
    add_box(
        fork_carriage,
        "left_guide_arm",
        (0.05, 0.08, 0.58),
        (-0.24, -0.03, 0.29),
        carriage_paint.name,
    )
    add_box(
        fork_carriage,
        "right_guide_arm",
        (0.05, 0.08, 0.58),
        (0.24, -0.03, 0.29),
        carriage_paint.name,
    )
    add_box(
        fork_carriage,
        "left_pad",
        (0.025, 0.08, 0.58),
        (-0.2775, -0.03, 0.29),
        wear_pad.name,
    )
    add_box(
        fork_carriage,
        "right_pad",
        (0.025, 0.08, 0.58),
        (0.2775, -0.03, 0.29),
        wear_pad.name,
    )
    add_box(
        fork_carriage,
        "left_fork_shank",
        (0.06, 0.08, 0.12),
        (-0.21, 0.05, -0.04),
        fork_steel.name,
    )
    add_box(
        fork_carriage,
        "right_fork_shank",
        (0.06, 0.08, 0.12),
        (0.21, 0.05, -0.04),
        fork_steel.name,
    )
    fork_carriage.visual(
        mesh_from_cadquery(make_fork_tine(), "left_fork_tine"),
        origin=Origin(xyz=(-0.21, 0.42, -0.10)),
        material=fork_steel.name,
        name="left_fork_tine",
    )
    fork_carriage.visual(
        mesh_from_cadquery(make_fork_tine(), "right_fork_tine"),
        origin=Origin(xyz=(0.21, 0.42, -0.10)),
        material=fork_steel.name,
        name="right_fork_tine",
    )

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast_frame,
        child=fork_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.35,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast_frame = object_model.get_part("mast_frame")
    fork_carriage = object_model.get_part("fork_carriage")
    mast_lift = object_model.get_articulation("mast_lift")

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

    axis = tuple(float(v) for v in mast_lift.axis)
    limits = mast_lift.motion_limits
    ctx.check(
        "mast_prismatic_axis_is_vertical",
        axis == (0.0, 0.0, 1.0),
        f"expected axis (0, 0, 1), got {axis}",
    )
    ctx.check(
        "mast_prismatic_limits_are_reasonable",
        limits is not None
        and math.isclose(limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and limits.upper is not None
        and 1.0 <= limits.upper <= 1.4,
        f"unexpected prismatic limits: {limits}",
    )

    with ctx.pose({mast_lift: 0.0}):
        ctx.expect_contact(
            mast_frame,
            fork_carriage,
            elem_a="left_guide_strip",
            elem_b="left_pad",
            name="left_pad_contacts_left_guide_at_rest",
        )
        ctx.expect_contact(
            mast_frame,
            fork_carriage,
            elem_a="right_guide_strip",
            elem_b="right_pad",
            name="right_pad_contacts_right_guide_at_rest",
        )

    with ctx.pose({mast_lift: limits.upper if limits and limits.upper is not None else 1.20}):
        ctx.expect_contact(
            mast_frame,
            fork_carriage,
            elem_a="left_guide_strip",
            elem_b="left_pad",
            name="left_pad_stays_guided_at_full_raise",
        )
        ctx.expect_contact(
            mast_frame,
            fork_carriage,
            elem_a="right_guide_strip",
            elem_b="right_pad",
            name="right_pad_stays_guided_at_full_raise",
        )
        ctx.expect_gap(
            mast_frame,
            fork_carriage,
            axis="z",
            positive_elem="top_tie",
            negative_elem="carriage_top",
            min_gap=0.08,
            max_gap=0.22,
            name="carriage_remains_below_upper_tie",
        )

    with ctx.pose({mast_lift: 0.0}):
        lower_pos = ctx.part_world_position(fork_carriage)
    with ctx.pose({mast_lift: limits.upper if limits and limits.upper is not None else 1.20}):
        upper_pos = ctx.part_world_position(fork_carriage)

    moved_up = (
        lower_pos is not None
        and upper_pos is not None
        and upper_pos[2] - lower_pos[2] > 1.0
        and math.isclose(upper_pos[0], lower_pos[0], abs_tol=1e-6)
        and math.isclose(upper_pos[1], lower_pos[1], abs_tol=1e-6)
    )
    ctx.check(
        "fork_carriage_translates_straight_up_the_mast",
        moved_up,
        f"lower_pos={lower_pos}, upper_pos={upper_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
