from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.14
BASE_THICKNESS = 0.018

SUPPORT_SPAN = 0.22
SUPPORT_LENGTH = 0.055
SUPPORT_FOOT_WIDTH = 0.09
SUPPORT_FOOT_HEIGHT = 0.018
SUPPORT_BODY_WIDTH = 0.05
SUPPORT_HOUSING_RADIUS = 0.032

SPINDLE_AXIS_HEIGHT = 0.082
JOURNAL_RADIUS = 0.016
END_STUB_RADIUS = 0.012
TOOLING_BODY_RADIUS = 0.022
TOOLING_TIP_RADIUS = 0.011

LEFT_SUPPORT_X = -SUPPORT_SPAN / 2.0
RIGHT_SUPPORT_X = SUPPORT_SPAN / 2.0
SUPPORT_INNER_FACE_X = SUPPORT_SPAN / 2.0 - SUPPORT_LENGTH / 2.0
SUPPORT_OUTER_FACE_X = SUPPORT_SPAN / 2.0 + SUPPORT_LENGTH / 2.0


def _make_base_plate() -> cq.Workplane:
    slot_x = BASE_LENGTH * 0.32
    slot_y = BASE_WIDTH * 0.28
    return (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-slot_x, -slot_y),
                (-slot_x, slot_y),
                (slot_x, -slot_y),
                (slot_x, slot_y),
            ]
        )
        .slot2D(0.024, 0.010, angle=0.0)
        .cutThruAll()
    )


def _make_support_body() -> cq.Workplane:
    cheek_thickness = 0.014
    cheek_offset = 0.026
    cheek_height = 0.084
    bridge_height = 0.014
    pedestal_height = 0.026

    foot = cq.Workplane("XY").box(
        SUPPORT_LENGTH,
        SUPPORT_FOOT_WIDTH,
        SUPPORT_FOOT_HEIGHT,
        centered=(True, True, False),
    )
    pedestal = (
        cq.Workplane("XY")
        .box(
            SUPPORT_LENGTH,
            SUPPORT_BODY_WIDTH,
            pedestal_height,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, SUPPORT_FOOT_HEIGHT))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(
            SUPPORT_LENGTH,
            cheek_thickness,
            cheek_height,
            centered=(True, True, False),
        )
        .translate((0.0, -cheek_offset, SUPPORT_FOOT_HEIGHT))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(
            SUPPORT_LENGTH,
            cheek_thickness,
            cheek_height,
            centered=(True, True, False),
        )
        .translate((0.0, cheek_offset, SUPPORT_FOOT_HEIGHT))
    )
    bridge = (
        cq.Workplane("XY")
        .box(
            SUPPORT_LENGTH,
            0.058,
            bridge_height,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, SPINDLE_AXIS_HEIGHT + JOURNAL_RADIUS + 0.004))
    )
    return foot.union(pedestal).union(left_cheek).union(right_cheek).union(bridge)


def _x_cylinder(start_x: float, end_x: float, radius: float) -> cq.Workplane:
    length = end_x - start_x
    mid_x = (start_x + end_x) / 2.0
    return (
        cq.Workplane("YZ")
        .workplane(offset=mid_x)
        .circle(radius)
        .extrude(length / 2.0, both=True)
    )


def _x_loft(start_x: float, start_radius: float, end_x: float, end_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=start_x)
        .circle(start_radius)
        .workplane(offset=end_x - start_x)
        .circle(end_radius)
        .loft(combine=True)
    )


def _make_spindle() -> cq.Workplane:
    points = [
        (-0.175, 0.0),
        (-0.175, END_STUB_RADIUS),
        (-0.1375, END_STUB_RADIUS),
        (-0.1375, JOURNAL_RADIUS),
        (-0.088, JOURNAL_RADIUS),
        (-0.076, 0.0135),
        (-0.024, 0.0135),
        (-0.006, TOOLING_BODY_RADIUS),
        (0.018, TOOLING_BODY_RADIUS),
        (0.062, TOOLING_TIP_RADIUS),
        (0.086, TOOLING_TIP_RADIUS),
        (0.094, JOURNAL_RADIUS),
        (0.1375, JOURNAL_RADIUS),
        (0.1375, END_STUB_RADIUS),
        (0.175, END_STUB_RADIUS),
        (0.175, 0.0),
    ]
    profile = cq.Workplane("XY").polyline(points).close()
    return profile.revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_spindle_assembly")

    model.material("base_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("housing_green", rgba=(0.22, 0.42, 0.30, 1.0))
    model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(_make_base_plate(), "base_plate"),
        material="base_gray",
        name="plate",
    )

    left_support = model.part("left_support")
    left_support.visual(
        mesh_from_cadquery(_make_support_body(), "left_support"),
        material="housing_green",
        name="housing",
    )
    left_support.visual(
        Cylinder(radius=0.006, length=SUPPORT_LENGTH),
        origin=Origin(
            xyz=(0.0, -(JOURNAL_RADIUS + 0.006), SPINDLE_AXIS_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="steel",
        name="left_roller",
    )
    left_support.visual(
        Cylinder(radius=0.006, length=SUPPORT_LENGTH),
        origin=Origin(
            xyz=(0.0, JOURNAL_RADIUS + 0.006, SPINDLE_AXIS_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="steel",
        name="right_roller",
    )

    right_support = model.part("right_support")
    right_support.visual(
        mesh_from_cadquery(_make_support_body(), "right_support"),
        material="housing_green",
        name="housing",
    )
    right_support.visual(
        Cylinder(radius=0.006, length=SUPPORT_LENGTH),
        origin=Origin(
            xyz=(0.0, -(JOURNAL_RADIUS + 0.006), SPINDLE_AXIS_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="steel",
        name="left_roller",
    )
    right_support.visual(
        Cylinder(radius=0.006, length=SUPPORT_LENGTH),
        origin=Origin(
            xyz=(0.0, JOURNAL_RADIUS + 0.006, SPINDLE_AXIS_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="steel",
        name="right_roller",
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_make_spindle(), "spindle"),
        material="steel",
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=SUPPORT_LENGTH),
        origin=Origin(
            xyz=(LEFT_SUPPORT_X, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="steel",
        name="left_journal",
    )
    spindle.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=SUPPORT_LENGTH),
        origin=Origin(
            xyz=(RIGHT_SUPPORT_X, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="steel",
        name="right_journal",
    )

    model.articulation(
        "base_to_left_support",
        ArticulationType.FIXED,
        parent=base_plate,
        child=left_support,
        origin=Origin(xyz=(LEFT_SUPPORT_X, 0.0, BASE_THICKNESS)),
    )
    model.articulation(
        "base_to_right_support",
        ArticulationType.FIXED,
        parent=base_plate,
        child=right_support,
        origin=Origin(xyz=(RIGHT_SUPPORT_X, 0.0, BASE_THICKNESS)),
    )
    model.articulation(
        "base_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + SPINDLE_AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")
    spindle = object_model.get_part("spindle")
    spindle_joint = object_model.get_articulation("base_to_spindle")

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

    ctx.check(
        "spindle_joint_is_continuous_about_x",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spindle_joint.axis) == (1.0, 0.0, 0.0)
        and spindle_joint.motion_limits is not None
        and spindle_joint.motion_limits.lower is None
        and spindle_joint.motion_limits.upper is None,
        details=(
            f"type={spindle_joint.articulation_type}, "
            f"axis={spindle_joint.axis}, "
            f"limits={spindle_joint.motion_limits}"
        ),
    )
    ctx.expect_contact(left_support, base_plate, name="left_support_seats_on_base")
    ctx.expect_contact(right_support, base_plate, name="right_support_seats_on_base")
    ctx.expect_origin_gap(
        right_support,
        left_support,
        axis="x",
        min_gap=SUPPORT_SPAN - 1e-4,
        max_gap=SUPPORT_SPAN + 1e-4,
        name="support_centers_match_bench_span",
    )
    ctx.expect_contact(spindle, left_support, name="spindle_supported_by_left_block")
    ctx.expect_contact(spindle, right_support, name="spindle_supported_by_right_block")
    ctx.expect_gap(
        spindle,
        base_plate,
        axis="z",
        min_gap=0.04,
        name="spindle_clears_base_plate",
    )

    with ctx.pose({spindle_joint: 1.1}):
        ctx.expect_contact(spindle, left_support, name="rotated_spindle_stays_in_left_bearing")
        ctx.expect_contact(spindle, right_support, name="rotated_spindle_stays_in_right_bearing")
        ctx.expect_gap(
            spindle,
            base_plate,
            axis="z",
            min_gap=0.04,
            name="rotated_spindle_still_clears_base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
