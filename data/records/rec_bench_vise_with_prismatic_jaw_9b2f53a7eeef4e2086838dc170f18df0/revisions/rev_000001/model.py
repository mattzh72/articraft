from __future__ import annotations

import math

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
)


JAW_TRAVEL = 0.050
LEVER_UPPER = 1.05


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hobby_vise")

    body = model.material("body", rgba=(0.24, 0.28, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    grip = model.material("grip", rgba=(0.14, 0.14, 0.14, 1.0))
    jaw_pad = model.material("jaw_pad", rgba=(0.50, 0.53, 0.58, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.047, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body,
        name="suction_foot",
    )
    base.visual(
        Box((0.088, 0.072, 0.014)),
        origin=Origin(xyz=(-0.002, 0.0, 0.016)),
        material=body,
        name="base_plinth",
    )
    base.visual(
        Box((0.042, 0.050, 0.020)),
        origin=Origin(xyz=(-0.022, 0.0, 0.032)),
        material=body,
        name="pedestal",
    )
    base.visual(
        Box((0.106, 0.054, 0.016)),
        origin=Origin(xyz=(0.004, 0.0, 0.050)),
        material=body,
        name="bed",
    )
    base.visual(
        Box((0.018, 0.060, 0.052)),
        origin=Origin(xyz=(-0.040, 0.0, 0.078)),
        material=body,
        name="fixed_jaw_block",
    )
    base.visual(
        Box((0.004, 0.056, 0.044)),
        origin=Origin(xyz=(-0.029, 0.0, 0.078)),
        material=jaw_pad,
        name="fixed_jaw_face",
    )
    base.visual(
        Box((0.082, 0.024, 0.014)),
        origin=Origin(xyz=(0.046, 0.0, 0.065)),
        material=body,
        name="guide_beam",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.088),
        origin=Origin(xyz=(0.020, 0.0, 0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lead_screw_tube",
    )
    base.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.070, 0.0, 0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_boss",
    )
    base.visual(
        Box((0.020, 0.008, 0.016)),
        origin=Origin(xyz=(-0.014, 0.036, 0.023)),
        material=body,
        name="lever_bracket",
    )

    jaw = model.part("jaw")
    jaw.visual(
        Box((0.016, 0.060, 0.044)),
        origin=Origin(xyz=(-0.022, 0.0, 0.013)),
        material=jaw_pad,
        name="jaw_face",
    )
    jaw.visual(
        Box((0.040, 0.014, 0.022)),
        origin=Origin(xyz=(0.004, -0.0195, 0.008)),
        material=body,
        name="side_cheek_0",
    )
    jaw.visual(
        Box((0.040, 0.014, 0.022)),
        origin=Origin(xyz=(0.004, 0.0195, 0.008)),
        material=body,
        name="side_cheek_1",
    )
    jaw.visual(
        Box((0.040, 0.060, 0.010)),
        origin=Origin(xyz=(0.004, 0.0, 0.022)),
        material=body,
        name="top_bridge",
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handle_hub",
    )
    screw_handle.visual(
        Cylinder(radius=0.0035, length=0.060),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_bar",
    )
    screw_handle.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.006, -0.030, 0.0)),
        material=grip,
        name="handle_knob_0",
    )
    screw_handle.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.006, 0.030, 0.0)),
        material=grip,
        name="handle_knob_1",
    )

    side_lever = model.part("side_lever")
    side_lever.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lever_hub",
    )
    side_lever.visual(
        Box((0.048, 0.012, 0.014)),
        origin=Origin(xyz=(0.024, 0.0, 0.006)),
        material=body,
        name="lever_arm",
    )
    side_lever.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.050, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip,
        name="lever_grip",
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=jaw,
        origin=Origin(xyz=(0.016, 0.0, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.12,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=screw_handle,
        origin=Origin(xyz=(0.076, 0.0, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=side_lever,
        origin=Origin(xyz=(-0.016, 0.045, 0.023)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.55,
            upper=LEVER_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    jaw = object_model.get_part("jaw")
    screw_handle = object_model.get_part("screw_handle")
    side_lever = object_model.get_part("side_lever")

    jaw_slide = object_model.get_articulation("jaw_slide")
    handle_spin = object_model.get_articulation("handle_spin")
    lever_pivot = object_model.get_articulation("lever_pivot")

    ctx.expect_gap(
        jaw,
        base,
        axis="x",
        positive_elem="jaw_face",
        negative_elem="fixed_jaw_face",
        min_gap=0.010,
        max_gap=0.018,
        name="jaw starts with a small clamping gap",
    )
    ctx.expect_gap(
        jaw,
        base,
        axis="z",
        positive_elem="jaw_face",
        negative_elem="lead_screw_tube",
        min_gap=0.001,
        max_gap=0.015,
        name="jaw face clears the lead screw tube",
    )
    ctx.expect_contact(
        screw_handle,
        base,
        elem_a="handle_hub",
        elem_b="front_boss",
        name="handle hub seats on the front boss",
    )
    ctx.expect_gap(
        screw_handle,
        base,
        axis="x",
        positive_elem="handle_bar",
        negative_elem="front_boss",
        min_gap=0.002,
        max_gap=0.012,
        name="handle bar stays ahead of the screw boss",
    )
    ctx.expect_contact(
        side_lever,
        base,
        elem_a="lever_hub",
        elem_b="lever_bracket",
        name="lever hub sits against the side bracket",
    )

    rest_jaw_pos = ctx.part_world_position(jaw)
    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        ctx.expect_gap(
            jaw,
            base,
            axis="x",
            positive_elem="jaw_face",
            negative_elem="fixed_jaw_face",
            min_gap=0.058,
            max_gap=0.070,
            name="jaw opens to a wider clamping span",
        )
        ctx.expect_overlap(
            jaw,
            base,
            axes="x",
            elem_a="side_cheek_0",
            elem_b="guide_beam",
            min_overlap=0.030,
            name="left cheek remains engaged on the guide",
        )
        ctx.expect_overlap(
            jaw,
            base,
            axes="x",
            elem_a="side_cheek_1",
            elem_b="guide_beam",
            min_overlap=0.030,
            name="right cheek remains engaged on the guide",
        )
        extended_jaw_pos = ctx.part_world_position(jaw)

    ctx.check(
        "jaw slides forward along the guide",
        rest_jaw_pos is not None
        and extended_jaw_pos is not None
        and extended_jaw_pos[0] > rest_jaw_pos[0] + 0.040,
        details=f"rest={rest_jaw_pos}, extended={extended_jaw_pos}",
    )

    handle_rest_aabb = ctx.part_world_aabb(screw_handle)
    with ctx.pose({handle_spin: math.pi / 2.0}):
        ctx.expect_contact(
            screw_handle,
            base,
            elem_a="handle_hub",
            elem_b="front_boss",
            name="handle remains seated while spinning",
        )
        handle_spun_aabb = ctx.part_world_aabb(screw_handle)

    ctx.check(
        "handle rotation preserves its front projection",
        handle_rest_aabb is not None
        and handle_spun_aabb is not None
        and handle_spun_aabb[0][0] >= handle_rest_aabb[0][0] - 1e-6
        and handle_spun_aabb[1][0] <= handle_rest_aabb[1][0] + 1e-6,
        details=f"rest={handle_rest_aabb}, spun={handle_spun_aabb}",
    )

    lever_rest_aabb = ctx.part_world_aabb(side_lever)
    with ctx.pose({lever_pivot: LEVER_UPPER}):
        lever_up_aabb = ctx.part_world_aabb(side_lever)

    ctx.check(
        "lever lifts upward from the base",
        lever_rest_aabb is not None
        and lever_up_aabb is not None
        and lever_up_aabb[1][2] > lever_rest_aabb[1][2] + 0.020,
        details=f"rest={lever_rest_aabb}, raised={lever_up_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
