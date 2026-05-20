from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BODY_X = 0.160
BODY_Y = 0.080
BODY_BASE_Z = 0.014
BODY_CENTER_X = 0.096
BODY_CENTER_Y = 0.034
BODY_CENTER_Z = 0.018
GUIDE_RAIL_Y = 0.020
GUIDE_RAIL_Z = 0.024
GUIDE_RAIL_OFFSET_Y = 0.029
BOLT_RADIUS = 0.0045
BOLT_HEAD_Z = 0.006
BOLT_OFFSET_X = 0.045
BODY_TOP_Z = BODY_BASE_Z + GUIDE_RAIL_Z

JAW_BLOCK_X = 0.044
JAW_BLOCK_Y = 0.072
JAW_BLOCK_Z = 0.018
JAW_SUPPORT_X = 0.016
JAW_SUPPORT_Y = 0.024
JAW_SUPPORT_Z = 0.012
JAW_SUPPORT_OFFSET_X = 0.016
JAW_STEM_X = 0.012
JAW_STEM_Y = 0.016
JAW_STEM_Z = 0.028
JAW_STEM_OFFSET_X = 0.026
JAW_PAD_X = 0.008
JAW_PAD_Y = 0.024
JAW_PAD_Z = 0.014
JAW_PAD_OFFSET_X = 0.032

JAW_STROKE = 0.014
JAW_OPEN_OFFSET_X = 0.055


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carriage_block_opposed_gripper")

    body_finish = model.material("body_finish", rgba=(0.16, 0.17, 0.19, 1.0))
    rail_finish = model.material("rail_finish", rgba=(0.24, 0.25, 0.28, 1.0))
    fastener_finish = model.material("fastener_finish", rgba=(0.58, 0.61, 0.66, 1.0))
    carriage_finish = model.material("carriage_finish", rgba=(0.64, 0.67, 0.72, 1.0))
    finger_finish = model.material("finger_finish", rgba=(0.34, 0.36, 0.40, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_X, BODY_Y, BODY_BASE_Z)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_Z / 2.0)),
        material=body_finish,
        name="base_plate",
    )
    body.visual(
        Box((BODY_CENTER_X, BODY_CENTER_Y, BODY_CENTER_Z)),
        origin=Origin(
            xyz=(0.0, 0.0, BODY_BASE_Z + BODY_CENTER_Z / 2.0),
        ),
        material=body_finish,
        name="center_core",
    )
    body.visual(
        Box((BODY_X, GUIDE_RAIL_Y, GUIDE_RAIL_Z)),
        origin=Origin(
            xyz=(0.0, GUIDE_RAIL_OFFSET_Y, BODY_BASE_Z + GUIDE_RAIL_Z / 2.0),
        ),
        material=rail_finish,
        name="left_guide_rail",
    )
    body.visual(
        Box((BODY_X, GUIDE_RAIL_Y, GUIDE_RAIL_Z)),
        origin=Origin(
            xyz=(0.0, -GUIDE_RAIL_OFFSET_Y, BODY_BASE_Z + GUIDE_RAIL_Z / 2.0),
        ),
        material=rail_finish,
        name="right_guide_rail",
    )
    body.visual(
        Cylinder(radius=BOLT_RADIUS, length=BOLT_HEAD_Z),
        origin=Origin(
            xyz=(BOLT_OFFSET_X, 0.0, BODY_BASE_Z + BODY_CENTER_Z + BOLT_HEAD_Z / 2.0),
        ),
        material=fastener_finish,
        name="left_bolt_head",
    )
    body.visual(
        Cylinder(radius=BOLT_RADIUS, length=BOLT_HEAD_Z),
        origin=Origin(
            xyz=(-BOLT_OFFSET_X, 0.0, BODY_BASE_Z + BODY_CENTER_Z + BOLT_HEAD_Z / 2.0),
        ),
        material=fastener_finish,
        name="right_bolt_head",
    )

    left_carriage = model.part("left_carriage")
    left_carriage.visual(
        Box((JAW_BLOCK_X, JAW_BLOCK_Y, JAW_BLOCK_Z)),
        origin=Origin(xyz=(0.0, 0.0, JAW_BLOCK_Z / 2.0)),
        material=carriage_finish,
        name="carriage_block",
    )
    left_carriage.visual(
        Box((JAW_SUPPORT_X, JAW_SUPPORT_Y, JAW_SUPPORT_Z)),
        origin=Origin(
            xyz=(
                JAW_SUPPORT_OFFSET_X,
                0.0,
                JAW_BLOCK_Z + JAW_SUPPORT_Z / 2.0,
            ),
        ),
        material=finger_finish,
        name="support_block",
    )
    left_carriage.visual(
        Box((JAW_STEM_X, JAW_STEM_Y, JAW_STEM_Z)),
        origin=Origin(
            xyz=(
                JAW_STEM_OFFSET_X,
                0.0,
                JAW_BLOCK_Z + JAW_STEM_Z / 2.0,
            ),
        ),
        material=finger_finish,
        name="finger_stem",
    )
    left_carriage.visual(
        Box((JAW_PAD_X, JAW_PAD_Y, JAW_PAD_Z)),
        origin=Origin(
            xyz=(
                JAW_PAD_OFFSET_X,
                0.0,
                JAW_BLOCK_Z + JAW_STEM_Z - JAW_PAD_Z / 2.0,
            ),
        ),
        material=finger_finish,
        name="finger_pad",
    )

    right_carriage = model.part("right_carriage")
    right_carriage.visual(
        Box((JAW_BLOCK_X, JAW_BLOCK_Y, JAW_BLOCK_Z)),
        origin=Origin(xyz=(0.0, 0.0, JAW_BLOCK_Z / 2.0)),
        material=carriage_finish,
        name="carriage_block",
    )
    right_carriage.visual(
        Box((JAW_SUPPORT_X, JAW_SUPPORT_Y, JAW_SUPPORT_Z)),
        origin=Origin(
            xyz=(
                -JAW_SUPPORT_OFFSET_X,
                0.0,
                JAW_BLOCK_Z + JAW_SUPPORT_Z / 2.0,
            ),
        ),
        material=finger_finish,
        name="support_block",
    )
    right_carriage.visual(
        Box((JAW_STEM_X, JAW_STEM_Y, JAW_STEM_Z)),
        origin=Origin(
            xyz=(
                -JAW_STEM_OFFSET_X,
                0.0,
                JAW_BLOCK_Z + JAW_STEM_Z / 2.0,
            ),
        ),
        material=finger_finish,
        name="finger_stem",
    )
    right_carriage.visual(
        Box((JAW_PAD_X, JAW_PAD_Y, JAW_PAD_Z)),
        origin=Origin(
            xyz=(
                -JAW_PAD_OFFSET_X,
                0.0,
                JAW_BLOCK_Z + JAW_STEM_Z - JAW_PAD_Z / 2.0,
            ),
        ),
        material=finger_finish,
        name="finger_pad",
    )

    model.articulation(
        "body_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_carriage,
        origin=Origin(xyz=(-JAW_OPEN_OFFSET_X, 0.0, BODY_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_STROKE,
        ),
    )
    model.articulation(
        "body_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_carriage,
        origin=Origin(xyz=(JAW_OPEN_OFFSET_X, 0.0, BODY_TOP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    left_slide = object_model.get_articulation("body_to_left_carriage")
    right_slide = object_model.get_articulation("body_to_right_carriage")

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
        "jaw_carriages_use_prismatic_joints",
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and right_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"left={left_slide.articulation_type}, right={right_slide.articulation_type}"
        ),
    )
    ctx.check(
        "jaw_axes_are_mirrored_on_closing_axis",
        tuple(left_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(right_slide.axis) == (-1.0, 0.0, 0.0),
        details=f"left_axis={left_slide.axis}, right_axis={right_slide.axis}",
    )

    open_left_pos = ctx.part_world_position(left_carriage)
    open_right_pos = ctx.part_world_position(right_carriage)
    if open_left_pos is not None and open_right_pos is not None:
        with ctx.pose({left_slide: JAW_STROKE, right_slide: JAW_STROKE}):
            closed_left_pos = ctx.part_world_position(left_carriage)
            closed_right_pos = ctx.part_world_position(right_carriage)
        ctx.check(
            "positive_prismatic_motion_closes_toward_pickup_zone",
            closed_left_pos is not None
            and closed_right_pos is not None
            and closed_left_pos[0] > open_left_pos[0]
            and closed_right_pos[0] < open_right_pos[0],
            details=(
                f"open_left={open_left_pos}, closed_left={closed_left_pos}, "
                f"open_right={open_right_pos}, closed_right={closed_right_pos}"
            ),
        )
    else:
        ctx.fail(
            "positive_prismatic_motion_closes_toward_pickup_zone",
            "Could not resolve carriage world positions in the default pose.",
        )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_contact(
            left_carriage,
            body,
            elem_a="carriage_block",
            elem_b="left_guide_rail",
            contact_tol=1e-6,
            name="left_carriage_supported_on_body_in_open_pose",
        )
        ctx.expect_contact(
            right_carriage,
            body,
            elem_a="carriage_block",
            elem_b="right_guide_rail",
            contact_tol=1e-6,
            name="right_carriage_supported_on_body_in_open_pose",
        )
        ctx.expect_overlap(
            left_carriage,
            body,
            axes="xy",
            elem_a="carriage_block",
            elem_b="left_guide_rail",
            min_overlap=0.016,
            name="left_carriage_has_real_rail_bearing_area",
        )
        ctx.expect_overlap(
            right_carriage,
            body,
            axes="xy",
            elem_a="carriage_block",
            elem_b="right_guide_rail",
            min_overlap=0.016,
            name="right_carriage_has_real_rail_bearing_area",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem="finger_pad",
            negative_elem="finger_pad",
            min_gap=0.030,
            max_gap=0.045,
            name="open_pose_has_clear_pickup_approach_gap",
        )

    with ctx.pose({left_slide: JAW_STROKE, right_slide: JAW_STROKE}):
        ctx.expect_contact(
            left_carriage,
            body,
            elem_a="carriage_block",
            elem_b="left_guide_rail",
            contact_tol=1e-6,
            name="left_carriage_remains_supported_when_closed",
        )
        ctx.expect_contact(
            right_carriage,
            body,
            elem_a="carriage_block",
            elem_b="right_guide_rail",
            contact_tol=1e-6,
            name="right_carriage_remains_supported_when_closed",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem="finger_pad",
            negative_elem="finger_pad",
            min_gap=0.008,
            max_gap=0.012,
            name="closed_pose_preserves_small_pickup_zone",
        )
        ctx.expect_overlap(
            right_carriage,
            left_carriage,
            axes="yz",
            elem_a="finger_pad",
            elem_b="finger_pad",
            min_overlap=0.014,
            name="finger_pads_face_the_same_pickup_zone",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
