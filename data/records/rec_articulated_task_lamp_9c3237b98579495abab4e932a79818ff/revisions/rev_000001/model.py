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


Y_AXIS_ROT = (pi / 2.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_task_lamp")

    base_black = model.material("base_black", rgba=(0.15, 0.15, 0.16, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.69, 0.72, 0.76, 1.0))
    knob_rubber = model.material("knob_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    diffuser = model.material("diffuser", rgba=(0.92, 0.92, 0.88, 0.68))

    base_plate_size = (0.260, 0.170, 0.018)
    sleeve_outer = (0.046, 0.036, 0.145)
    sleeve_inner = (0.032, 0.022)
    sleeve_wall_x = (sleeve_outer[0] - sleeve_inner[0]) / 2.0
    sleeve_wall_y = (sleeve_outer[1] - sleeve_inner[1]) / 2.0
    post_x = -0.078
    sleeve_center_z = base_plate_size[2] + sleeve_outer[2] / 2.0
    sleeve_top_z = base_plate_size[2] + sleeve_outer[2]

    base = model.part("base")
    base.visual(
        Box(base_plate_size),
        origin=Origin(xyz=(0.0, 0.0, base_plate_size[2] / 2.0)),
        material=base_black,
        name="base_plate",
    )
    base.visual(
        Box((sleeve_wall_x, sleeve_outer[1], sleeve_outer[2])),
        origin=Origin(
            xyz=(
                post_x + sleeve_inner[0] / 2.0 + sleeve_wall_x / 2.0,
                0.0,
                sleeve_center_z,
            )
        ),
        material=base_black,
        name="sleeve_front",
    )
    base.visual(
        Box((sleeve_wall_x, sleeve_outer[1], sleeve_outer[2])),
        origin=Origin(
            xyz=(
                post_x - sleeve_inner[0] / 2.0 - sleeve_wall_x / 2.0,
                0.0,
                sleeve_center_z,
            )
        ),
        material=base_black,
        name="sleeve_rear",
    )
    base.visual(
        Box((sleeve_inner[0], sleeve_wall_y, sleeve_outer[2])),
        origin=Origin(
            xyz=(
                post_x,
                sleeve_inner[1] / 2.0 + sleeve_wall_y / 2.0,
                sleeve_center_z,
            )
        ),
        material=base_black,
        name="sleeve_side_pos",
    )
    base.visual(
        Box((sleeve_inner[0], sleeve_wall_y, sleeve_outer[2])),
        origin=Origin(
            xyz=(
                post_x,
                -sleeve_inner[1] / 2.0 - sleeve_wall_y / 2.0,
                sleeve_center_z,
            )
        ),
        material=base_black,
        name="sleeve_side_neg",
    )
    base.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(
            xyz=(post_x, sleeve_outer[1] / 2.0 + 0.008, 0.110),
            rpy=Y_AXIS_ROT,
        ),
        material=brushed_steel,
        name="clamp_stem",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(
            xyz=(post_x, sleeve_outer[1] / 2.0 + 0.023, 0.110),
            rpy=Y_AXIS_ROT,
        ),
        material=knob_rubber,
        name="clamp_knob",
    )

    upright = model.part("upright")
    upright.visual(
        Box((0.028, 0.018, 0.445)),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=brushed_steel,
        name="mast",
    )
    upright.visual(
        Box((0.020, 0.004, 0.130)),
        origin=Origin(xyz=(0.0, 0.009, -0.055)),
        material=satin_black,
        name="guide_pos",
    )
    upright.visual(
        Box((0.020, 0.004, 0.130)),
        origin=Origin(xyz=(0.0, -0.009, -0.055)),
        material=satin_black,
        name="guide_neg",
    )
    for y_pos, name in ((0.011, "shoulder_strap_pos"), (-0.011, "shoulder_strap_neg")):
        upright.visual(
            Box((0.026, 0.008, 0.030)),
            origin=Origin(xyz=(0.010, y_pos, 0.323)),
            material=satin_black,
            name=name,
        )
    for y_pos, name in ((0.011, "shoulder_knuckle_pos"), (-0.011, "shoulder_knuckle_neg")):
        upright.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.020, y_pos, 0.338), rpy=Y_AXIS_ROT),
            material=satin_black,
            name=name,
        )

    model.articulation(
        "base_to_upright",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upright,
        origin=Origin(xyz=(post_x, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.080,
        ),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(rpy=Y_AXIS_ROT),
        material=satin_black,
        name="root_hub",
    )
    arm.visual(
        Box((0.170, 0.012, 0.012)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=satin_black,
        name="beam",
    )
    for y_pos, name in ((0.010, "tip_strap_pos"), (-0.010, "tip_strap_neg")):
        arm.visual(
            Box((0.020, 0.008, 0.018)),
            origin=Origin(xyz=(0.180, y_pos, 0.0)),
            material=satin_black,
            name=name,
        )
    for y_pos, name in ((0.010, "tip_knuckle_pos"), (-0.010, "tip_knuckle_neg")):
        arm.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(0.190, y_pos, 0.0), rpy=Y_AXIS_ROT),
            material=satin_black,
            name=name,
        )

    model.articulation(
        "upright_to_arm",
        ArticulationType.REVOLUTE,
        parent=upright,
        child=arm,
        origin=Origin(xyz=(0.020, 0.0, 0.338)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.6,
            lower=-0.35,
            upper=1.05,
        ),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(rpy=Y_AXIS_ROT),
        material=satin_black,
        name="tilt_hub",
    )
    shade.visual(
        Box((0.020, 0.012, 0.016)),
        origin=Origin(xyz=(0.010, 0.0, -0.008)),
        material=satin_black,
        name="spine",
    )
    shade.visual(
        Box((0.170, 0.062, 0.004)),
        origin=Origin(xyz=(0.097, 0.0, -0.002)),
        material=satin_black,
        name="top_panel",
    )
    shade.visual(
        Box((0.156, 0.004, 0.020)),
        origin=Origin(xyz=(0.101, 0.029, -0.010)),
        material=satin_black,
        name="side_wall_pos",
    )
    shade.visual(
        Box((0.156, 0.004, 0.020)),
        origin=Origin(xyz=(0.101, -0.029, -0.010)),
        material=satin_black,
        name="side_wall_neg",
    )
    shade.visual(
        Box((0.006, 0.054, 0.018)),
        origin=Origin(xyz=(0.179, 0.0, -0.009)),
        material=satin_black,
        name="front_wall",
    )
    shade.visual(
        Box((0.160, 0.058, 0.002)),
        origin=Origin(xyz=(0.097, 0.0, -0.019)),
        material=diffuser,
        name="diffuser",
    )

    model.articulation(
        "arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.50,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upright = object_model.get_part("upright")
    arm = object_model.get_part("arm")
    shade = object_model.get_part("shade")

    lift_joint = object_model.get_articulation("base_to_upright")
    shoulder_joint = object_model.get_articulation("upright_to_arm")
    shade_joint = object_model.get_articulation("arm_to_shade")

    ctx.expect_gap(
        base,
        upright,
        axis="x",
        positive_elem="sleeve_front",
        negative_elem="mast",
        min_gap=0.0015,
        max_gap=0.0030,
        name="mast clears the forward sleeve wall",
    )
    ctx.expect_gap(
        upright,
        base,
        axis="x",
        positive_elem="mast",
        negative_elem="sleeve_rear",
        min_gap=0.0015,
        max_gap=0.0030,
        name="mast clears the rear sleeve wall",
    )
    ctx.expect_contact(
        upright,
        base,
        elem_a="guide_pos",
        elem_b="sleeve_side_pos",
        name="upper guide bears on the positive sleeve wall",
    )
    ctx.expect_contact(
        upright,
        base,
        elem_a="guide_neg",
        elem_b="sleeve_side_neg",
        name="upper guide bears on the negative sleeve wall",
    )
    ctx.expect_overlap(
        upright,
        base,
        axes="z",
        elem_a="mast",
        elem_b="sleeve_front",
        min_overlap=0.115,
        name="collapsed mast remains deeply inserted in the sleeve",
    )

    rest_upright_pos = ctx.part_world_position(upright)
    with ctx.pose({lift_joint: 0.080}):
        ctx.expect_overlap(
            upright,
            base,
            axes="z",
            elem_a="mast",
            elem_b="sleeve_front",
            min_overlap=0.038,
            name="extended mast still retains insertion in the sleeve",
        )
        extended_upright_pos = ctx.part_world_position(upright)

    ctx.check(
        "upright extends upward",
        rest_upright_pos is not None
        and extended_upright_pos is not None
        and extended_upright_pos[2] > rest_upright_pos[2] + 0.075,
        details=f"rest={rest_upright_pos}, extended={extended_upright_pos}",
    )

    rest_arm_aabb = ctx.part_element_world_aabb(arm, elem="beam")
    with ctx.pose({shoulder_joint: 1.05}):
        raised_arm_aabb = ctx.part_element_world_aabb(arm, elem="beam")

    ctx.check(
        "arm pitches upward at the shoulder",
        rest_arm_aabb is not None
        and raised_arm_aabb is not None
        and raised_arm_aabb[1][2] > rest_arm_aabb[1][2] + 0.120,
        details=f"rest={rest_arm_aabb}, raised={raised_arm_aabb}",
    )
    ctx.expect_contact(
        arm,
        upright,
        elem_a="root_hub",
        elem_b="shoulder_knuckle_pos",
        name="arm hub stays mounted in the shoulder clevis",
    )

    with ctx.pose({shoulder_joint: 0.45}):
        neutral_shade_aabb = ctx.part_element_world_aabb(shade, elem="front_wall")
    with ctx.pose({shoulder_joint: 0.45, shade_joint: 0.55}):
        tilted_shade_aabb = ctx.part_element_world_aabb(shade, elem="front_wall")

    ctx.check(
        "shade tilts downward at the arm tip",
        neutral_shade_aabb is not None
        and tilted_shade_aabb is not None
        and tilted_shade_aabb[0][2] < neutral_shade_aabb[0][2] - 0.050,
        details=f"neutral={neutral_shade_aabb}, tilted={tilted_shade_aabb}",
    )
    ctx.expect_contact(
        shade,
        arm,
        elem_a="tilt_hub",
        elem_b="tip_knuckle_pos",
        name="shade hub stays mounted in the arm-tip clevis",
    )

    return ctx.report()


object_model = build_object_model()
