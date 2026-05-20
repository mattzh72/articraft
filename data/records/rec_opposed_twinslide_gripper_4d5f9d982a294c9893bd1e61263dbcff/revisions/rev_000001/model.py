from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.18
BODY_D = 0.14
BODY_H = 0.09
REAR_CAP_D = 0.03

RAIL_SPAN = 0.26
RAIL_DEPTH = 0.028
RAIL_THICK = 0.018
RAIL_Z = 0.024

CARRIAGE_W = 0.072
CARRIAGE_D = 0.045
CARRIAGE_H = 0.07
CARRIAGE_CENTER_Y = BODY_D / 2.0 + RAIL_DEPTH / 2.0

OPEN_CENTER_X = 0.082
JAW_STROKE = 0.042

FINGER_BASE_W = 0.026
FINGER_BASE_D = 0.05
FINGER_BASE_H = 0.028
FINGER_TIP_W = 0.016
FINGER_TIP_D = 0.032
FINGER_TIP_H = 0.044


def make_housing_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .edges("|Z")
        .fillet(0.005)
    )

    rear_cap = cq.Workplane("XY").box(BODY_W * 0.62, REAR_CAP_D, BODY_H * 0.72).translate(
        (0.0, -(BODY_D / 2.0 + REAR_CAP_D / 2.0), 0.0)
    )
    top_land = cq.Workplane("XY").box(BODY_W * 0.52, BODY_D * 0.34, 0.016).translate(
        (0.0, -0.006, BODY_H / 2.0 + 0.008)
    )
    left_cheek = cq.Workplane("XY").box(0.026, 0.038, BODY_H * 0.68).translate(
        (-0.082, BODY_D / 2.0 - 0.032, 0.0)
    )
    right_cheek = cq.Workplane("XY").box(0.026, 0.038, BODY_H * 0.68).translate(
        (0.082, BODY_D / 2.0 - 0.032, 0.0)
    )
    top_rail = cq.Workplane("XY").box(RAIL_SPAN, RAIL_DEPTH, RAIL_THICK).translate(
        (0.0, BODY_D / 2.0 + RAIL_DEPTH / 2.0, RAIL_Z)
    )
    bottom_rail = cq.Workplane("XY").box(RAIL_SPAN, RAIL_DEPTH, RAIL_THICK).translate(
        (0.0, BODY_D / 2.0 + RAIL_DEPTH / 2.0, -RAIL_Z)
    )

    return (
        body.union(rear_cap)
        .union(top_land)
        .union(left_cheek)
        .union(right_cheek)
        .union(top_rail)
        .union(bottom_rail)
    )


def make_jaw_shape(hand: int) -> cq.Workplane:
    inward = -hand
    guide_shoe_h = 0.016
    guide_shoe_z = RAIL_Z + RAIL_THICK / 2.0 + guide_shoe_h / 2.0
    bridge_d = 0.032
    bridge_h = 0.09

    top_shoe = cq.Workplane("XY").box(CARRIAGE_W, RAIL_DEPTH, guide_shoe_h).translate(
        (0.0, 0.0, guide_shoe_z)
    )
    bottom_shoe = cq.Workplane("XY").box(CARRIAGE_W, RAIL_DEPTH, guide_shoe_h).translate(
        (0.0, 0.0, -guide_shoe_z)
    )
    front_bridge = cq.Workplane("XY").box(CARRIAGE_W * 0.96, bridge_d, bridge_h).translate(
        (0.0, RAIL_DEPTH / 2.0 + bridge_d / 2.0, 0.0)
    )
    inner_cheek = cq.Workplane("XY").box(0.022, 0.04, 0.082).translate(
        (
            inward * (CARRIAGE_W / 2.0 - 0.011),
            RAIL_DEPTH / 2.0 + 0.02,
            0.0,
        )
    )

    finger_base = cq.Workplane("XY").box(FINGER_BASE_W, FINGER_BASE_D, FINGER_BASE_H).translate(
        (
            inward * (CARRIAGE_W / 2.0 - FINGER_BASE_W / 2.0 - 0.004),
            RAIL_DEPTH / 2.0 + bridge_d + FINGER_BASE_D / 2.0 - 0.004,
            -0.008,
        )
    )
    finger_riser = cq.Workplane("XY").box(0.02, 0.024, 0.052).translate(
        (
            inward * (CARRIAGE_W / 2.0 - 0.01 - 0.004),
            RAIL_DEPTH / 2.0 + bridge_d + 0.018,
            0.0,
        )
    )
    finger_tip = cq.Workplane("XY").box(FINGER_TIP_W, FINGER_TIP_D, FINGER_TIP_H).translate(
        (
            inward * (CARRIAGE_W / 2.0 - FINGER_TIP_W / 2.0 - 0.002),
            RAIL_DEPTH / 2.0 + bridge_d + FINGER_BASE_D + FINGER_TIP_D / 2.0 - 0.012,
            0.0,
        )
    )

    jaw = (
        top_shoe.union(bottom_shoe)
        .union(front_bridge)
        .union(inner_cheek)
        .union(finger_base)
        .union(finger_riser)
        .union(finger_tip)
    )
    return jaw.translate((0.0, RAIL_DEPTH, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_block_gripper")

    housing_color = model.material("housing_color", color=(0.19, 0.21, 0.24, 1.0))
    jaw_color = model.material("jaw_color", color=(0.60, 0.62, 0.64, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(make_housing_shape(), "housing_shell"),
        material=housing_color,
        name="housing_shell",
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_cadquery(make_jaw_shape(-1), "left_jaw_body"),
        material=jaw_color,
        name="left_jaw_body",
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(make_jaw_shape(1), "right_jaw_body"),
        material=jaw_color,
        name="right_jaw_body",
    )

    model.articulation(
        "housing_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_jaw,
        origin=Origin(xyz=(-OPEN_CENTER_X, CARRIAGE_CENTER_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=0.2,
            lower=0.0,
            upper=JAW_STROKE,
        ),
    )
    model.articulation(
        "housing_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_jaw,
        origin=Origin(xyz=(OPEN_CENTER_X, CARRIAGE_CENTER_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=0.2,
            lower=0.0,
            upper=JAW_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("housing_to_left_jaw")
    right_slide = object_model.get_articulation("housing_to_right_jaw")

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
        "left jaw uses inward prismatic slide",
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(left_slide.axis) == (1.0, 0.0, 0.0)
        and left_slide.motion_limits is not None
        and left_slide.motion_limits.lower == 0.0
        and left_slide.motion_limits.upper == JAW_STROKE,
        details=f"axis={left_slide.axis}, limits={left_slide.motion_limits}",
    )
    ctx.check(
        "right jaw uses inward prismatic slide",
        right_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(right_slide.axis) == (-1.0, 0.0, 0.0)
        and right_slide.motion_limits is not None
        and right_slide.motion_limits.lower == 0.0
        and right_slide.motion_limits.upper == JAW_STROKE,
        details=f"axis={right_slide.axis}, limits={right_slide.motion_limits}",
    )

    ctx.expect_contact(left_jaw, housing, name="left jaw carriage bears on housing rails")
    ctx.expect_contact(right_jaw, housing, name="right jaw carriage bears on housing rails")
    ctx.expect_overlap(
        left_jaw,
        housing,
        axes="xz",
        min_overlap=0.06,
        name="left jaw stays in the rail band",
    )
    ctx.expect_overlap(
        right_jaw,
        housing,
        axes="xz",
        min_overlap=0.06,
        name="right jaw stays in the rail band",
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.09,
            max_gap=0.11,
            name="jaws start open with a workpiece gap",
        )

    with ctx.pose({left_slide: JAW_STROKE, right_slide: JAW_STROKE}):
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.006,
            max_gap=0.01,
            name="jaws close symmetrically to a narrow remaining gap",
        )
        left_pos = ctx.part_world_position(left_jaw)
        right_pos = ctx.part_world_position(right_jaw)
        ctx.check(
            "jaw carriage centers remain symmetric in the closed pose",
            left_pos is not None
            and right_pos is not None
            and abs(left_pos[0] + right_pos[0]) < 1e-6
            and abs(left_pos[1] - right_pos[1]) < 1e-6
            and abs(left_pos[2] - right_pos[2]) < 1e-6,
            details=f"left={left_pos}, right={right_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
