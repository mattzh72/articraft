from __future__ import annotations

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overbed_workstation_table")

    steel = model.material("steel", rgba=(0.27, 0.29, 0.31, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    top_laminate = model.material("top_laminate", rgba=(0.80, 0.77, 0.70, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.40, 0.04, 0.022)),
        origin=Origin(xyz=(0.08, 0.0, 0.079)),
        material=steel,
        name="spine",
    )
    base.visual(
        Box((0.04, 0.24, 0.022)),
        origin=Origin(xyz=(-0.08, 0.0, 0.079)),
        material=steel,
        name="rear_crossbar",
    )
    base.visual(
        Box((0.04, 0.20, 0.022)),
        origin=Origin(xyz=(0.26, 0.0, 0.079)),
        material=steel,
        name="front_crossbar",
    )
    base.visual(
        Box((0.08, 0.09, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=steel,
        name="pedestal",
    )

    sleeve_height = 0.34
    sleeve_z = 0.110 + sleeve_height / 2.0
    base.visual(
        Box((0.004, 0.070, sleeve_height)),
        origin=Origin(xyz=(0.023, 0.0, sleeve_z)),
        material=dark_trim,
        name="sleeve_front",
    )
    base.visual(
        Box((0.004, 0.070, sleeve_height)),
        origin=Origin(xyz=(-0.023, 0.0, sleeve_z)),
        material=dark_trim,
        name="sleeve_back",
    )
    base.visual(
        Box((0.042, 0.004, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.033, sleeve_z)),
        material=dark_trim,
        name="sleeve_right",
    )
    base.visual(
        Box((0.042, 0.004, sleeve_height)),
        origin=Origin(xyz=(0.0, -0.033, sleeve_z)),
        material=dark_trim,
        name="sleeve_left",
    )

    caster_positions = {
        "front_left_wheel": (0.26, -0.09),
        "front_right_wheel": (0.26, 0.09),
        "rear_left_wheel": (-0.08, -0.11),
        "rear_right_wheel": (-0.08, 0.11),
    }
    for wheel_name, (x_pos, y_pos) in caster_positions.items():
        base.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(x_pos, y_pos, 0.062)),
            material=dark_trim,
            name=f"{wheel_name}_stem",
        )
        base.visual(
            Box((0.022, 0.018, 0.008)),
            origin=Origin(xyz=(x_pos, y_pos, 0.052)),
            material=dark_trim,
            name=f"{wheel_name}_bridge",
        )
        for suffix, y_offset in (("inner_arm", -0.007), ("outer_arm", 0.007)):
            base.visual(
                Box((0.006, 0.002, 0.036)),
                origin=Origin(xyz=(x_pos, y_pos + y_offset, 0.030)),
                material=dark_trim,
                name=f"{wheel_name}_{suffix}",
            )

    column = model.part("column")
    column.visual(
        Box((0.042, 0.062, 0.520)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_trim,
        name="mast",
    )
    column.visual(
        Box((0.100, 0.090, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.307)),
        material=steel,
        name="head_plate",
    )
    column.visual(
        Box((0.050, 0.090, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.314)),
        material=steel,
        name="hinge_saddle",
    )

    top = model.part("top")
    top.visual(
        Box((0.44, 0.34, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=top_laminate,
        name="tray",
    )
    top.visual(
        Cylinder(radius=0.004, length=0.080),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    top.visual(
        Box((0.020, 0.080, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel,
        name="hinge_web",
    )
    top.visual(
        Box((0.018, 0.33, 0.018)),
        origin=Origin(xyz=(0.211, 0.0, 0.032)),
        material=top_laminate,
        name="lip",
    )
    top.visual(
        Box((0.16, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.185, -0.002)),
        material=steel,
        name="wing_bracket",
    )

    wing = model.part("wing")
    wing.visual(
        Box((0.26, 0.14, 0.016)),
        origin=Origin(xyz=(0.0, 0.070, 0.010)),
        material=top_laminate,
        name="panel",
    )
    wing.visual(
        Box((0.10, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.009, -0.002)),
        material=steel,
        name="hinge_leaf",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.18),
    )
    model.articulation(
        "column_to_top",
        ArticulationType.REVOLUTE,
        parent=column,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.323)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=-0.25, upper=0.95),
    )
    model.articulation(
        "top_to_wing",
        ArticulationType.REVOLUTE,
        parent=top,
        child=wing,
        origin=Origin(xyz=(0.0, 0.200, 0.005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=1.30),
    )

    for wheel_name, (x_pos, y_pos) in caster_positions.items():
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        model.articulation(
            f"base_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=(x_pos, y_pos, 0.024)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    top = object_model.get_part("top")
    wing = object_model.get_part("wing")

    lift = object_model.get_articulation("base_to_column")
    tilt = object_model.get_articulation("column_to_top")
    wing_hinge = object_model.get_articulation("top_to_wing")

    with ctx.pose({lift: 0.0}):
        ctx.expect_gap(
            base,
            column,
            axis="x",
            positive_elem="sleeve_front",
            negative_elem="mast",
            max_gap=0.0005,
            max_penetration=0.00001,
            name="mast stays guided by sleeve front wall",
        )
        ctx.expect_gap(
            column,
            base,
            axis="x",
            positive_elem="mast",
            negative_elem="sleeve_back",
            max_gap=0.0005,
            max_penetration=0.00001,
            name="mast stays guided by sleeve back wall",
        )
        ctx.expect_gap(
            base,
            column,
            axis="y",
            positive_elem="sleeve_right",
            negative_elem="mast",
            max_gap=0.0005,
            max_penetration=0.00001,
            name="mast stays guided by sleeve right wall",
        )
        ctx.expect_gap(
            column,
            base,
            axis="y",
            positive_elem="mast",
            negative_elem="sleeve_left",
            max_gap=0.0005,
            max_penetration=0.00001,
            name="mast stays guided by sleeve left wall",
        )
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="mast",
            elem_b="sleeve_front",
            min_overlap=0.20,
            name="collapsed mast remains deep in sleeve",
        )
        ctx.expect_overlap(
            wing,
            top,
            axes="x",
            elem_a="panel",
            elem_b="tray",
            min_overlap=0.24,
            name="wing aligns with tray depth",
        )
        ctx.expect_gap(
            wing,
            top,
            axis="y",
            positive_elem="hinge_leaf",
            negative_elem="wing_bracket",
            min_gap=0.0,
            max_gap=0.002,
            name="wing hinge sits on side bracket",
        )

    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({lift: 0.18}):
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="mast",
            elem_b="sleeve_front",
            min_overlap=0.035,
            name="extended mast stays retained in sleeve",
        )
        extended_column_pos = ctx.part_world_position(column)

    ctx.check(
        "column extends upward",
        rest_column_pos is not None
        and extended_column_pos is not None
        and extended_column_pos[2] > rest_column_pos[2] + 0.15,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(top, elem="lip")
    with ctx.pose({tilt: 0.95}):
        tilted_lip_aabb = ctx.part_element_world_aabb(top, elem="lip")

    ctx.check(
        "top front edge tilts upward",
        rest_lip_aabb is not None
        and tilted_lip_aabb is not None
        and tilted_lip_aabb[1][2] > rest_lip_aabb[1][2] + 0.10,
        details=f"rest={rest_lip_aabb}, tilted={tilted_lip_aabb}",
    )

    rest_wing_aabb = ctx.part_element_world_aabb(wing, elem="panel")
    with ctx.pose({wing_hinge: 1.30}):
        raised_wing_aabb = ctx.part_element_world_aabb(wing, elem="panel")

    ctx.check(
        "reading wing raises beside the tray",
        rest_wing_aabb is not None
        and raised_wing_aabb is not None
        and raised_wing_aabb[1][2] > rest_wing_aabb[1][2] + 0.08,
        details=f"rest={rest_wing_aabb}, raised={raised_wing_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
