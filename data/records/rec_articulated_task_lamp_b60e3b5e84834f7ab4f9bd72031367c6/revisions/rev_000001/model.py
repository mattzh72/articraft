from __future__ import annotations

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
    model = ArticulatedObject(name="desk_task_lamp")

    base_finish = model.material("base_finish", rgba=(0.20, 0.20, 0.22, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.12, 0.12, 0.13, 1.0))
    head_finish = model.material("head_finish", rgba=(0.16, 0.16, 0.17, 1.0))
    lens_finish = model.material("lens_finish", rgba=(0.88, 0.89, 0.82, 0.95))
    button_finish = model.material("button_finish", rgba=(0.86, 0.28, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.18, 0.11, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_finish,
        name="base_plate",
    )
    base.visual(
        Box((0.024, 0.016, 0.044)),
        origin=Origin(xyz=(-0.062, -0.019, 0.040)),
        material=base_finish,
        name="base_cheek_0",
    )
    base.visual(
        Box((0.024, 0.016, 0.044)),
        origin=Origin(xyz=(-0.062, 0.019, 0.040)),
        material=base_finish,
        name="base_cheek_1",
    )

    upright = model.part("upright")
    upright.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="upright_barrel",
    )
    upright.visual(
        Box((0.016, 0.018, 0.102)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=frame_finish,
        name="upright_stem",
    )
    upright.visual(
        Box((0.018, 0.022, 0.012)),
        origin=Origin(xyz=(0.003, 0.010, 0.105)),
        material=frame_finish,
        name="upright_saddle",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="arm_barrel",
    )
    arm.visual(
        Box((0.122, 0.014, 0.014)),
        origin=Origin(xyz=(0.061, 0.0, 0.0)),
        material=frame_finish,
        name="arm_beam",
    )
    arm.visual(
        Box((0.010, 0.016, 0.010)),
        origin=Origin(xyz=(0.122, 0.0, -0.011)),
        material=frame_finish,
        name="arm_tip_saddle",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=head_finish,
        name="head_barrel",
    )
    head.visual(
        Box((0.020, 0.016, 0.006)),
        origin=Origin(xyz=(0.001, 0.0, -0.003)),
        material=head_finish,
        name="head_mount",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.062),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=head_finish,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(0.059, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=head_finish,
        name="head_bezel",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(0.063, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_finish,
        name="lens",
    )
    head.visual(
        Box((0.018, 0.010, 0.004)),
        origin=Origin(xyz=(0.018, 0.0245, 0.008)),
        material=head_finish,
        name="button_guide_upper",
    )
    head.visual(
        Box((0.018, 0.010, 0.004)),
        origin=Origin(xyz=(0.018, 0.0245, -0.008)),
        material=head_finish,
        name="button_guide_lower",
    )

    button = model.part("button")
    button.visual(
        Box((0.014, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=button_finish,
        name="button_cap",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upright,
        origin=Origin(xyz=(-0.062, 0.0, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.95,
        ),
    )
    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=upright,
        child=arm,
        origin=Origin(xyz=(0.003, 0.010, 0.117)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=-0.75,
            upper=1.05,
        ),
    )
    model.articulation(
        "head_hinge",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.136, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=-1.10,
            upper=0.65,
        ),
    )
    model.articulation(
        "button_slide",
        ArticulationType.PRISMATIC,
        parent=head,
        child=button,
        origin=Origin(xyz=(0.018, 0.026, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.05,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head = object_model.get_part("head")
    button = object_model.get_part("button")
    base_hinge = object_model.get_articulation("base_hinge")
    arm_hinge = object_model.get_articulation("arm_hinge")
    head_hinge = object_model.get_articulation("head_hinge")
    button_slide = object_model.get_articulation("button_slide")

    ctx.expect_gap(
        button,
        head,
        axis="y",
        min_gap=0.003,
        max_gap=0.005,
        positive_elem="button_cap",
        negative_elem="head_shell",
        name="button cap sits proud of the cylindrical shell",
    )
    ctx.expect_within(
        button,
        head,
        axes="xz",
        margin=0.0,
        inner_elem="button_cap",
        name="button stays aligned within the head envelope",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({base_hinge: base_hinge.motion_limits.upper}):
        forward_head_pos = ctx.part_world_position(head)
    ctx.check(
        "upright hinge pitches the lamp forward",
        rest_head_pos is not None
        and forward_head_pos is not None
        and forward_head_pos[0] > rest_head_pos[0] + 0.035,
        details=f"rest={rest_head_pos}, forward={forward_head_pos}",
    )

    with ctx.pose({arm_hinge: arm_hinge.motion_limits.upper}):
        raised_head_pos = ctx.part_world_position(head)
    ctx.check(
        "arm hinge raises the head",
        rest_head_pos is not None
        and raised_head_pos is not None
        and raised_head_pos[2] > rest_head_pos[2] + 0.04,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(head, elem="lens")
    with ctx.pose({head_hinge: head_hinge.motion_limits.lower}):
        lowered_lens_aabb = ctx.part_element_world_aabb(head, elem="lens")
    rest_lens_z = None
    lowered_lens_z = None
    if rest_lens_aabb is not None:
        rest_lens_z = 0.5 * (rest_lens_aabb[0][2] + rest_lens_aabb[1][2])
    if lowered_lens_aabb is not None:
        lowered_lens_z = 0.5 * (lowered_lens_aabb[0][2] + lowered_lens_aabb[1][2])
    ctx.check(
        "head hinge tilts the beam downward",
        rest_lens_z is not None
        and lowered_lens_z is not None
        and lowered_lens_z < rest_lens_z - 0.02,
        details=f"rest_lens_z={rest_lens_z}, lowered_lens_z={lowered_lens_z}",
    )

    rest_button_pos = ctx.part_world_position(button)
    with ctx.pose({button_slide: button_slide.motion_limits.upper}):
        pressed_button_pos = ctx.part_world_position(button)
    ctx.check(
        "side button depresses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] < rest_button_pos[1] - 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
