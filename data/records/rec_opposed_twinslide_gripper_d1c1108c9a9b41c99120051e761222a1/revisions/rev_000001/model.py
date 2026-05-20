from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _make_center_block() -> cq.Workplane:
    body_x = 0.120
    body_y = 0.090
    body_z = 0.046

    guide_len = 0.130
    guide_y = 0.034
    guide_z = 0.014
    guide_x = 0.075
    guide_center_y = 0.010

    cap_x = 0.076
    cap_y = 0.052
    cap_z = 0.016

    core = (
        cq.Workplane("XY")
        .box(body_x, body_y, body_z)
        .edges("|Z")
        .fillet(0.006)
    )

    cap = (
        cq.Workplane("XY")
        .box(cap_x, cap_y, cap_z)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, -0.014, body_z / 2 + cap_z / 2 - 0.002))
    )

    left_guide = (
        cq.Workplane("XY")
        .box(guide_len, guide_y, guide_z)
        .edges("|Z")
        .fillet(0.002)
        .translate((-guide_x, guide_center_y, body_z / 2 + guide_z / 2 - 0.001))
    )
    right_guide = (
        cq.Workplane("XY")
        .box(guide_len, guide_y, guide_z)
        .edges("|Z")
        .fillet(0.002)
        .translate((guide_x, guide_center_y, body_z / 2 + guide_z / 2 - 0.001))
    )

    front_pocket = cq.Workplane("XY").box(0.038, 0.040, 0.018).translate(
        (0.0, 0.014, body_z / 2 + 0.003)
    )
    upper_relief = cq.Workplane("XY").box(0.050, 0.026, 0.012).translate(
        (0.0, -0.010, body_z / 2 + 0.008)
    )

    return core.union(cap).union(left_guide).union(right_guide).cut(front_pocket).cut(
        upper_relief
    )


def _make_jaw(side: str) -> cq.Workplane:
    if side not in {"left", "right"}:
        raise ValueError(f"unsupported jaw side: {side}")

    inward = 1.0 if side == "left" else -1.0

    carriage_x = 0.036
    carriage_y = 0.038
    carriage_z = 0.018

    finger_w = 0.018
    finger_l = 0.155
    finger_t = 0.004
    finger_x = inward * 0.008
    finger_z = -0.006
    finger_overlap = 0.004

    slot_w = 0.008
    slot_l = 0.105
    slot_start = 0.046

    carriage = (
        cq.Workplane("XY")
        .box(carriage_x, carriage_y, carriage_z)
        .edges("|Z")
        .fillet(0.003)
    )

    finger = cq.Workplane("XY").box(
        finger_w,
        finger_l,
        finger_t,
        centered=(True, False, True),
    )
    finger = finger.translate((finger_x, carriage_y / 2 - finger_overlap, finger_z))

    fork_slot = cq.Workplane("XY").box(
        slot_w,
        slot_l,
        finger_t + 0.006,
        centered=(True, False, True),
    ).translate(
        (
            finger_x,
            carriage_y / 2 - finger_overlap + slot_start,
            finger_z,
        )
    )

    return carriage.union(finger).cut(fork_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wafer_handling_gripper")

    model.material("body_dark", rgba=(0.24, 0.26, 0.30, 1.0))
    model.material("jaw_metal", rgba=(0.78, 0.80, 0.82, 1.0))

    guide_top_z = 0.046 / 2 + 0.014 - 0.001
    carriage_center_y = 0.010
    carriage_center_z = guide_top_z + 0.018 / 2
    open_offset_x = 0.090
    stroke = 0.055

    body = model.part("center_block")
    body.visual(
        mesh_from_cadquery(_make_center_block(), "center_block"),
        material="body_dark",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(Box((0.280, 0.090, 0.065)), mass=2.8)

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_cadquery(_make_jaw("left"), "left_jaw"),
        material="jaw_metal",
        name="left_jaw_shell",
    )
    left_jaw.inertial = Inertial.from_geometry(Box((0.036, 0.190, 0.020)), mass=0.18)

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(_make_jaw("right"), "right_jaw"),
        material="jaw_metal",
        name="right_jaw_shell",
    )
    right_jaw.inertial = Inertial.from_geometry(Box((0.036, 0.190, 0.020)), mass=0.18)

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(-open_offset_x, carriage_center_y, carriage_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=stroke,
        ),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_jaw,
        origin=Origin(xyz=(open_offset_x, carriage_center_y, carriage_center_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=stroke,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("center_block")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

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

    left_limits = left_slide.motion_limits
    right_limits = right_slide.motion_limits
    left_upper = 0.0 if left_limits is None or left_limits.upper is None else left_limits.upper
    right_upper = 0.0 if right_limits is None or right_limits.upper is None else right_limits.upper

    ctx.check(
        "left_slide_axis",
        tuple(left_slide.axis) == (1.0, 0.0, 0.0),
        f"expected +X prismatic axis, got {left_slide.axis}",
    )
    ctx.check(
        "right_slide_axis",
        tuple(right_slide.axis) == (-1.0, 0.0, 0.0),
        f"expected -X prismatic axis, got {right_slide.axis}",
    )
    ctx.check(
        "matching_slide_strokes",
        abs(left_upper - right_upper) < 1e-9 and left_upper > 0.04,
        f"unexpected prismatic limits: left={left_upper}, right={right_upper}",
    )

    ctx.expect_contact(left_jaw, body, name="left_jaw_supported_on_left_guide")
    ctx.expect_contact(right_jaw, body, name="right_jaw_supported_on_right_guide")
    ctx.expect_overlap(
        left_jaw,
        body,
        axes="xy",
        min_overlap=0.020,
        name="left_carriage_has_real_guide_footprint",
    )
    ctx.expect_overlap(
        right_jaw,
        body,
        axes="xy",
        min_overlap=0.020,
        name="right_carriage_has_real_guide_footprint",
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        open_left_x = ctx.part_world_position(left_jaw)
        open_right_x = ctx.part_world_position(right_jaw)
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.120,
            name="open_jaws_leave_large_center_clearance",
        )

    with ctx.pose({left_slide: left_upper, right_slide: right_upper}):
        closed_left_x = ctx.part_world_position(left_jaw)
        closed_right_x = ctx.part_world_position(right_jaw)
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.015,
            max_gap=0.050,
            name="closed_jaws_remain_separate",
        )

    ctx.check(
        "left_slide_moves_inward",
        open_left_x is not None
        and closed_left_x is not None
        and closed_left_x[0] > open_left_x[0] + 0.040,
        f"left jaw did not move toward center: open={open_left_x}, closed={closed_left_x}",
    )
    ctx.check(
        "right_slide_moves_inward",
        open_right_x is not None
        and closed_right_x is not None
        and closed_right_x[0] < open_right_x[0] - 0.040,
        f"right jaw did not move toward center: open={open_right_x}, closed={closed_right_x}",
    )

    with ctx.pose({left_slide: left_upper, right_slide: 0.0}):
        left_only_left_x = ctx.part_world_position(left_jaw)
        left_only_right_x = ctx.part_world_position(right_jaw)
    ctx.check(
        "left_slide_is_independent",
        left_only_left_x is not None
        and open_left_x is not None
        and left_only_right_x is not None
        and open_right_x is not None
        and left_only_left_x[0] > open_left_x[0] + 0.040
        and abs(left_only_right_x[0] - open_right_x[0]) < 1e-9,
        (
            "left slide should move without dragging right jaw: "
            f"left_only_left={left_only_left_x}, left_only_right={left_only_right_x}, "
            f"open_left={open_left_x}, open_right={open_right_x}"
        ),
    )

    with ctx.pose({left_slide: 0.0, right_slide: right_upper}):
        right_only_left_x = ctx.part_world_position(left_jaw)
        right_only_right_x = ctx.part_world_position(right_jaw)
    ctx.check(
        "right_slide_is_independent",
        right_only_right_x is not None
        and open_right_x is not None
        and right_only_left_x is not None
        and open_left_x is not None
        and right_only_right_x[0] < open_right_x[0] - 0.040
        and abs(right_only_left_x[0] - open_left_x[0]) < 1e-9,
        (
            "right slide should move without dragging left jaw: "
            f"right_only_left={right_only_left_x}, right_only_right={right_only_right_x}, "
            f"open_left={open_left_x}, open_right={open_right_x}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
