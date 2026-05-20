from __future__ import annotations

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


BASE_LENGTH = 0.152
BASE_WIDTH = 0.040
BASE_THICK = 0.0115
BASE_RAIL_LENGTH = 0.070
BASE_RAIL_WIDTH = 0.010
BASE_RAIL_HEIGHT = 0.002
HINGE_X = -0.054
HINGE_Z = 0.0205

ARM_LENGTH = 0.133
ARM_WIDTH = 0.030
ARM_HEIGHT = 0.026
TRAY_TRAVEL = 0.050
STOP_TRAVEL = 0.060
LATCH_TRAVEL = 0.008


def _make_base_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICK, centered=(True, True, False))
    body = body.edges("|Z").fillet(0.004)

    center_rail = (
        cq.Workplane("XY")
        .box(BASE_RAIL_LENGTH, BASE_RAIL_WIDTH, BASE_RAIL_HEIGHT, centered=(True, True, False))
        .translate((0.016, 0.0, BASE_THICK))
    )

    rear_pad = (
        cq.Workplane("XY")
        .box(0.034, BASE_WIDTH - 0.004, 0.004, centered=(True, True, False))
        .translate((HINGE_X + 0.010, 0.0, BASE_THICK))
        .edges("|Z")
        .fillet(0.0018)
    )

    hinge_offset_y = 0.0135
    hinge_tower = (
        cq.Workplane("XY")
        .box(0.016, 0.008, 0.0125, centered=(True, True, False))
    )
    left_tower = hinge_tower.translate((HINGE_X + 0.003, hinge_offset_y, BASE_THICK))
    right_tower = hinge_tower.translate((HINGE_X + 0.003, -hinge_offset_y, BASE_THICK))

    front_pad = (
        cq.Workplane("XY")
        .box(0.028, 0.024, 0.0012, centered=(True, True, False))
        .translate((BASE_LENGTH * 0.33, 0.0, BASE_THICK))
    )

    return body.union(center_rail).union(rear_pad).union(left_tower).union(right_tower).union(front_pad)


def _make_top_arm_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(0.113, ARM_WIDTH, ARM_HEIGHT, centered=(False, True, False))
        .translate((0.020, 0.0, -0.004))
    )
    shell = shell.union(
        cq.Workplane("XY")
        .box(0.036, 0.018, 0.018, centered=(False, True, False))
        .translate((-0.004, 0.0, -0.002))
    )

    cavity = (
        cq.Workplane("XY")
        .box(0.124, 0.024, 0.020, centered=(False, True, False))
        .translate((0.008, 0.0, -0.007))
    )
    nose_relief = (
        cq.Workplane("XY")
        .box(0.020, 0.020, 0.008, centered=(False, True, False))
        .translate((0.112, 0.0, -0.003))
    )
    latch_slot = (
        cq.Workplane("XY")
        .box(0.024, 0.014, 0.008, centered=(False, True, False))
        .translate((-0.006, 0.0, 0.007))
    )
    return shell.cut(cavity).cut(nose_relief).cut(latch_slot)


def _make_tray_shape() -> cq.Workplane:
    bottom = (
        cq.Workplane("XY")
        .box(0.118, 0.022, 0.0018, centered=(False, True, False))
        .translate((-0.008, 0.0, 0.004))
    )
    left_rail = (
        cq.Workplane("XY")
        .box(0.104, 0.002, 0.0045, centered=(False, True, False))
        .translate((-0.006, 0.010, 0.004))
    )
    right_rail = left_rail.translate((0.0, -0.020, 0.0))
    front_lip = (
        cq.Workplane("XY")
        .box(0.010, 0.020, 0.007, centered=(False, True, False))
        .translate((0.100, 0.0, 0.004))
    )
    front_tab = (
        cq.Workplane("XY")
        .box(0.012, 0.016, 0.003, centered=(False, True, False))
        .translate((0.106, 0.0, 0.011))
    )
    return bottom.union(left_rail).union(right_rail).union(front_lip).union(front_tab)


def _make_paper_stop_shape() -> cq.Workplane:
    left_ski = (
        cq.Workplane("XY")
        .box(0.016, 0.004, 0.0015, centered=(True, True, False))
        .translate((0.0, 0.0085, 0.0))
    )
    right_ski = left_ski.translate((0.0, -0.017, 0.0))
    left_post = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.0045, centered=(True, True, False))
        .translate((0.0, 0.0085, 0.0014))
    )
    right_post = left_post.translate((0.0, -0.017, 0.0))
    bridge = (
        cq.Workplane("XY")
        .box(0.015, 0.020, 0.0015, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0050))
    )
    fence = (
        cq.Workplane("XY")
        .box(0.003, 0.020, 0.010, centered=(True, True, False))
        .translate((0.0065, 0.0, 0.0060))
    )
    return left_ski.union(right_ski).union(left_post).union(right_post).union(bridge).union(fence)


def _make_latch_shape() -> cq.Workplane:
    plunger = (
        cq.Workplane("XY")
        .box(0.014, 0.008, 0.004, centered=(False, True, False))
        .translate((0.000, 0.0, 0.0075))
    )
    cap = (
        cq.Workplane("XY")
        .box(0.010, 0.012, 0.004, centered=(False, True, False))
        .translate((-0.008, 0.0, 0.0070))
    )
    return plunger.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")

    model.material("body_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("tray_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("guide_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("release_black", rgba=(0.16, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "stapler_base"), material="body_black", name="base_shell")
    base.visual(
        Box((0.118, 0.028, 0.001)),
        origin=Origin(xyz=(0.006, 0.0, BASE_THICK + 0.0005)),
        material="tray_steel",
        name="deck_plate",
    )

    top_arm = model.part("top_arm")
    top_arm.visual(mesh_from_cadquery(_make_top_arm_shape(), "stapler_top_arm"), material="body_black", name="arm_body")

    staple_tray = model.part("staple_tray")
    staple_tray.visual(mesh_from_cadquery(_make_tray_shape(), "stapler_tray"), material="tray_steel", name="tray_body")

    paper_stop = model.part("paper_stop")
    paper_stop.visual(mesh_from_cadquery(_make_paper_stop_shape(), "paper_stop"), material="guide_gray", name="stop_body")

    release_latch = model.part("release_latch")
    release_latch.visual(mesh_from_cadquery(_make_latch_shape(), "release_latch"), material="release_black", name="latch_body")

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top_arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.92, effort=18.0, velocity=2.5),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=top_arm,
        child=staple_tray,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=8.0, velocity=0.20),
    )
    model.articulation(
        "stop_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=paper_stop,
        origin=Origin(xyz=(-0.010, 0.0, BASE_THICK + 0.0006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=STOP_TRAVEL, effort=3.0, velocity=0.10),
    )
    model.articulation(
        "latch_slide",
        ArticulationType.PRISMATIC,
        parent=top_arm,
        child=release_latch,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=LATCH_TRAVEL, effort=2.0, velocity=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    top_arm = object_model.get_part("top_arm")
    staple_tray = object_model.get_part("staple_tray")
    paper_stop = object_model.get_part("paper_stop")
    release_latch = object_model.get_part("release_latch")
    arm_hinge = object_model.get_articulation("arm_hinge")
    tray_slide = object_model.get_articulation("tray_slide")
    stop_slide = object_model.get_articulation("stop_slide")
    latch_slide = object_model.get_articulation("latch_slide")
    hinge_limits = arm_hinge.motion_limits
    tray_limits = tray_slide.motion_limits
    stop_limits = stop_slide.motion_limits
    latch_limits = latch_slide.motion_limits

    ctx.allow_isolated_part(
        top_arm,
        reason="The stapler arm is carried by an explicit rear hinge articulation with modeled clearance instead of a literal fused hinge pin.",
    )
    ctx.allow_isolated_part(
        staple_tray,
        reason="The staple tray is represented as a clearanced slide within the upper housing rather than a zero-clearance rail fit.",
    )
    ctx.allow_isolated_part(
        release_latch,
        reason="The rear release latch is modeled with sliding clearance inside its top-arm pocket.",
    )
    ctx.allow_isolated_part(
        paper_stop,
        reason="The paper stop is represented as a clearanced rider over the shallow deck guide instead of a zero-clearance sliding contact.",
    )
    ctx.allow_overlap(
        top_arm,
        staple_tray,
        elem_a="arm_body",
        elem_b="tray_body",
        reason="The staple tray is intentionally represented as sliding within the upper shell proxy.",
    )
    arm_box = ctx.part_element_world_aabb(top_arm, elem="arm_body")
    deck_box = ctx.part_element_world_aabb(base, elem="deck_plate")
    stop_box = ctx.part_element_world_aabb(paper_stop, elem="stop_body")
    deck_gap = None
    stop_gap = None
    if arm_box is not None and deck_box is not None:
        deck_gap = arm_box[0][2] - deck_box[1][2]
    if stop_box is not None:
        stop_gap = stop_box[0][2] - BASE_THICK
    ctx.check(
        "closed arm floats just above the deck plate",
        deck_gap is not None and 0.003 <= deck_gap <= 0.012,
        details=f"deck_gap={deck_gap}, arm_box={arm_box}, deck_box={deck_box}",
    )
    ctx.check(
        "paper stop rides just above the base deck",
        stop_gap is not None and 0.0003 <= stop_gap <= 0.0015,
        details=f"stop_gap={stop_gap}, stop_box={stop_box}",
    )
    ctx.expect_overlap(
        top_arm,
        base,
        axes="xy",
        elem_a="arm_body",
        elem_b="base_shell",
        min_overlap=0.020,
        name="closed arm covers the base footprint",
    )
    ctx.expect_within(
        paper_stop,
        base,
        axes="y",
        elem_a="stop_body",
        elem_b="base_shell",
        margin=0.004,
        name="paper stop stays centered on the deck rail",
    )
    ctx.expect_overlap(
        staple_tray,
        top_arm,
        axes="x",
        elem_a="tray_body",
        elem_b="arm_body",
        min_overlap=0.070,
        name="closed tray remains deeply inserted in the arm",
    )
    ctx.expect_within(
        staple_tray,
        top_arm,
        axes="y",
        elem_a="tray_body",
        elem_b="arm_body",
        margin=0.006,
        name="tray stays centered inside the upper housing",
    )
    ctx.expect_overlap(
        release_latch,
        top_arm,
        axes="yz",
        elem_a="latch_body",
        elem_b="arm_body",
        min_overlap=0.004,
        name="rear latch stays housed within the upper shell",
    )

    if hinge_limits is not None and hinge_limits.upper is not None:
        closed_box = ctx.part_element_world_aabb(top_arm, elem="arm_body")
        with ctx.pose({arm_hinge: hinge_limits.upper}):
            open_box = ctx.part_element_world_aabb(top_arm, elem="arm_body")

        ctx.check(
            "arm opens upward from the rear hinge",
            closed_box is not None
            and open_box is not None
            and open_box[1][2] > closed_box[1][2] + 0.040,
            details=f"closed={closed_box}, open={open_box}",
        )

    if tray_limits is not None and tray_limits.upper is not None:
        rest_pos = ctx.part_world_position(staple_tray)
        with ctx.pose({tray_slide: tray_limits.upper}):
            extended_pos = ctx.part_world_position(staple_tray)
            ctx.expect_overlap(
                staple_tray,
                top_arm,
                axes="x",
                elem_a="tray_body",
                elem_b="arm_body",
                min_overlap=0.020,
                name="extended tray remains retained by the upper housing",
            )
            ctx.expect_within(
                staple_tray,
                top_arm,
                axes="y",
                elem_a="tray_body",
                elem_b="arm_body",
                margin=0.006,
                name="extended tray stays aligned between the side walls",
            )

        ctx.check(
            "tray slides forward out of the front opening",
            rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.040,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    if stop_limits is not None and stop_limits.upper is not None:
        stop_rest = ctx.part_world_position(paper_stop)
        with ctx.pose({stop_slide: stop_limits.upper}):
            stop_extended = ctx.part_world_position(paper_stop)

        ctx.check(
            "paper stop slides forward along the base",
            stop_rest is not None and stop_extended is not None and stop_extended[0] > stop_rest[0] + 0.040,
            details=f"rest={stop_rest}, extended={stop_extended}",
        )

    if latch_limits is not None and latch_limits.upper is not None:
        latch_rest = ctx.part_world_position(release_latch)
        with ctx.pose({latch_slide: latch_limits.upper}):
            latch_pressed = ctx.part_world_position(release_latch)
            ctx.expect_overlap(
                release_latch,
                top_arm,
                axes="yz",
                elem_a="latch_body",
                elem_b="arm_body",
                min_overlap=0.004,
                name="pressed latch remains captured by the housing",
            )

        ctx.check(
            "rear latch pushes inward into the housing",
            latch_rest is not None and latch_pressed is not None and latch_pressed[0] > latch_rest[0] + 0.005,
            details=f"rest={latch_rest}, pressed={latch_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
