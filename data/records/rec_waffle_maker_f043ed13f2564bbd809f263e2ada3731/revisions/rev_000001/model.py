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
    model = ArticulatedObject(name="square_clamshell_waffle_maker")

    housing = model.material("housing", rgba=(0.16, 0.17, 0.18, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.22, 0.23, 0.24, 1.0))
    iron = model.material("iron", rgba=(0.11, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.70, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.30, 0.27, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=housing,
        name="body",
    )
    base.visual(
        Box((0.06, 0.27, 0.018)),
        origin=Origin(xyz=(-0.12, 0.0, 0.049)),
        material=housing,
        name="rear_rail",
    )
    base.visual(
        Box((0.02, 0.27, 0.018)),
        origin=Origin(xyz=(0.14, 0.0, 0.049)),
        material=housing,
        name="front_rail",
    )
    base.visual(
        Box((0.22, 0.025, 0.018)),
        origin=Origin(xyz=(0.02, -0.1225, 0.049)),
        material=housing,
        name="side_rail_0",
    )
    base.visual(
        Box((0.22, 0.025, 0.018)),
        origin=Origin(xyz=(0.02, 0.1225, 0.049)),
        material=housing,
        name="side_rail_1",
    )
    base.visual(
        Box((0.028, 0.25, 0.032)),
        origin=Origin(xyz=(-0.136, 0.0, 0.056)),
        material=housing,
        name="hinge_block",
    )
    base.visual(
        Box((0.22, 0.22, 0.018)),
        origin=Origin(xyz=(0.02, 0.0, 0.049)),
        material=iron,
        name="lower_plate",
    )

    for idx, (x_pos, y_pos) in enumerate(((-0.11, -0.095), (-0.11, 0.095), (0.11, -0.095), (0.11, 0.095))):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, -0.003)),
            material=rubber,
            name=f"foot_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.26, 0.268, 0.044)),
        origin=Origin(xyz=(0.14, 0.0, 0.016)),
        material=lid_finish,
        name="lid_shell",
    )
    lid.visual(
        Box((0.19, 0.19, 0.010)),
        origin=Origin(xyz=(0.145, 0.0, 0.039)),
        material=lid_finish,
        name="lid_crown",
    )
    lid.visual(
        Box((0.018, 0.24, 0.020)),
        origin=Origin(xyz=(0.009, 0.0, 0.002)),
        material=lid_finish,
        name="rear_skirt",
    )
    lid.visual(
        Box((0.22, 0.22, 0.018)),
        origin=Origin(xyz=(0.14, 0.0, 0.0005)),
        material=iron,
        name="upper_plate",
    )
    lid.visual(
        Box((0.018, 0.02, 0.020)),
        origin=Origin(xyz=(0.277, -0.045, -0.004)),
        material=lid_finish,
        name="handle_post_0",
    )
    lid.visual(
        Box((0.018, 0.02, 0.020)),
        origin=Origin(xyz=(0.277, 0.045, -0.004)),
        material=lid_finish,
        name="handle_post_1",
    )
    lid.visual(
        Box((0.05, 0.12, 0.018)),
        origin=Origin(xyz=(0.300, 0.0, -0.004)),
        material=rubber,
        name="front_handle",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="dial_body",
    )
    dial.visual(
        Box((0.010, 0.003, 0.014)),
        origin=Origin(xyz=(0.010, 0.017, 0.0)),
        material=iron,
        name="indicator",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.12, 0.0, 0.068)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=0.0, upper=1.55),
    )
    model.articulation(
        "dial_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.085, 0.135, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=4.71),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    lid_hinge = object_model.get_articulation("lid_hinge")
    dial_joint = object_model.get_articulation("dial_joint")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.001,
            max_gap=0.004,
            name="closed plates nearly meet",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.20,
            name="upper and lower plates align",
        )
        ctx.expect_gap(
            dial,
            base,
            axis="y",
            positive_elem="dial_body",
            negative_elem="body",
            min_gap=0.0,
            max_gap=0.001,
            name="temperature dial seats against lower housing side",
        )

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            ctx.expect_gap(
                lid,
                base,
                axis="z",
                positive_elem="upper_plate",
                negative_elem="lower_plate",
                min_gap=0.035,
                name="lid opens clearly above the lower plate",
            )

    dial_limits = dial_joint.motion_limits
    if dial_limits is not None and dial_limits.lower is not None and dial_limits.upper is not None:
        with ctx.pose({dial_joint: dial_limits.lower}):
            low_center = _aabb_center(ctx.part_element_world_aabb(dial, elem="indicator"))
        with ctx.pose({dial_joint: dial_limits.upper}):
            high_center = _aabb_center(ctx.part_element_world_aabb(dial, elem="indicator"))
        ctx.check(
            "dial indicator rotates through its range",
            low_center is not None
            and high_center is not None
            and high_center[2] > low_center[2] + 0.008,
            details=f"low_center={low_center}, high_center={high_center}",
        )

    return ctx.report()


object_model = build_object_model()
