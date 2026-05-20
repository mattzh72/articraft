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


BODY_DEPTH = 0.34
BODY_WIDTH = 0.27
BODY_HALF_DEPTH = BODY_DEPTH / 2.0
BODY_HALF_WIDTH = BODY_WIDTH / 2.0

PLATE_DEPTH = 0.262
PLATE_WIDTH = 0.190
PLATE_THICKNESS = 0.014
RIDGE_THICKNESS = 0.004
RIDGE_HEIGHT = 0.002


def _add_plate_grid(
    part,
    *,
    x_center: float,
    z_plane: float,
    material,
    underside: bool = False,
) -> None:
    ridge_center_z = z_plane + (RIDGE_HEIGHT / 2.0) * (-1.0 if underside else 1.0)
    ridge_depth = PLATE_DEPTH - 0.016
    ridge_width = PLATE_WIDTH - 0.016

    for y_pos in (-0.060, -0.020, 0.020, 0.060):
        part.visual(
            Box((ridge_depth, RIDGE_THICKNESS, RIDGE_HEIGHT)),
            origin=Origin(xyz=(x_center, y_pos, ridge_center_z)),
            material=material,
            name=None,
        )

    for x_offset in (-0.090, -0.030, 0.030, 0.090):
        part.visual(
            Box((RIDGE_THICKNESS, ridge_width, RIDGE_HEIGHT)),
            origin=Origin(xyz=(x_center + x_offset, 0.0, ridge_center_z)),
            material=material,
            name=None,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_waffle_iron")

    shell = model.material("shell", rgba=(0.10, 0.10, 0.11, 1.0))
    shell_trim = model.material("shell_trim", rgba=(0.16, 0.16, 0.17, 1.0))
    plate_metal = model.material("plate_metal", rgba=(0.22, 0.23, 0.24, 1.0))
    handle_trim = model.material("handle_trim", rgba=(0.17, 0.18, 0.19, 1.0))
    dial_metal = model.material("dial_metal", rgba=(0.74, 0.75, 0.77, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.94, 0.24, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((BODY_DEPTH, BODY_WIDTH, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=shell,
        name="floor",
    )
    base.visual(
        Box((0.020, BODY_WIDTH, 0.036)),
        origin=Origin(xyz=(0.160, 0.0, 0.028)),
        material=shell,
        name="front_wall",
    )
    base.visual(
        Box((0.026, BODY_WIDTH, 0.054)),
        origin=Origin(xyz=(-0.157, 0.0, 0.037)),
        material=shell,
        name="rear_wall",
    )
    base.visual(
        Box((0.294, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.126, 0.030)),
        material=shell,
        name="left_wall",
    )
    base.visual(
        Box((0.294, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.126, 0.030)),
        material=shell,
        name="right_wall",
    )
    lower_plate = model.part("lower_plate")
    lower_plate.visual(
        Box((PLATE_DEPTH, PLATE_WIDTH, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=plate_metal,
        name="plate",
    )
    lower_plate.visual(
        Box((0.118, 0.090, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=plate_metal,
        name="mount",
    )
    _add_plate_grid(
        lower_plate,
        x_center=0.0,
        z_plane=0.050,
        material=plate_metal,
    )

    model.articulation(
        "base_to_lower_plate",
        ArticulationType.FIXED,
        parent=base,
        child=lower_plate,
        origin=Origin(),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.316, BODY_WIDTH, 0.010)),
        origin=Origin(xyz=(0.158, 0.0, 0.042)),
        material=shell,
        name="top_panel",
    )
    lid.visual(
        Box((0.024, BODY_WIDTH, 0.037)),
        origin=Origin(xyz=(0.012, 0.0, 0.0185)),
        material=shell,
        name="rear_spine",
    )
    lid.visual(
        Box((0.018, BODY_WIDTH, 0.032)),
        origin=Origin(xyz=(0.307, 0.0, 0.021)),
        material=shell,
        name="front_skirt",
    )
    lid.visual(
        Box((0.292, 0.018, 0.032)),
        origin=Origin(xyz=(0.158, -0.126, 0.021)),
        material=shell,
        name="left_skirt",
    )
    lid.visual(
        Box((0.292, 0.018, 0.032)),
        origin=Origin(xyz=(0.158, 0.126, 0.021)),
        material=shell,
        name="right_skirt",
    )
    lid.visual(
        Box((0.020, 0.018, 0.018)),
        origin=Origin(xyz=(0.321, -0.036, 0.016)),
        material=handle_trim,
        name="grip_post_0",
    )
    lid.visual(
        Box((0.020, 0.018, 0.018)),
        origin=Origin(xyz=(0.321, 0.036, 0.016)),
        material=handle_trim,
        name="grip_post_1",
    )
    lid.visual(
        Box((0.020, 0.090, 0.018)),
        origin=Origin(xyz=(0.331, 0.0, 0.016)),
        material=handle_trim,
        name="front_grip",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.158, 0.0, 0.064)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.25,
        ),
    )

    upper_plate = model.part("upper_plate")
    upper_plate.visual(
        Box((PLATE_DEPTH, PLATE_WIDTH, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.158, 0.0, 0.003)),
        material=plate_metal,
        name="plate",
    )
    upper_plate.visual(
        Box((0.116, 0.096, 0.027)),
        origin=Origin(xyz=(0.158, 0.0, 0.0235)),
        material=plate_metal,
        name="mount",
    )
    _add_plate_grid(
        upper_plate,
        x_center=0.158,
        z_plane=-0.004,
        material=plate_metal,
        underside=True,
    )

    model.articulation(
        "lid_to_upper_plate",
        ArticulationType.FIXED,
        parent=lid,
        child=upper_plate,
        origin=Origin(),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_metal,
        name="shaft",
    )
    dial.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_metal,
        name="knob",
    )
    dial.visual(
        Box((0.022, 0.003, 0.003)),
        origin=Origin(xyz=(0.006, 0.0245, 0.010)),
        material=dial_mark,
        name="pointer",
    )

    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.055, 0.135, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lower_plate = object_model.get_part("lower_plate")
    upper_plate = object_model.get_part("upper_plate")
    dial = object_model.get_part("dial")
    lid_hinge = object_model.get_articulation("base_to_lid")
    dial_joint = object_model.get_articulation("base_to_dial")
    lid_limits = lid_hinge.motion_limits

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            base,
            lower_plate,
            axis="x",
            positive_elem="front_wall",
            negative_elem="plate",
            min_gap=0.012,
            max_gap=0.028,
            name="base front shell break surrounds lower plate",
        )
        ctx.expect_gap(
            base,
            lower_plate,
            axis="y",
            positive_elem="right_wall",
            negative_elem="plate",
            min_gap=0.020,
            max_gap=0.040,
            name="base side shell break surrounds lower plate",
        )
        ctx.expect_gap(
            lid,
            upper_plate,
            axis="x",
            positive_elem="front_skirt",
            negative_elem="plate",
            min_gap=0.008,
            max_gap=0.018,
            name="lid front shell break surrounds upper plate",
        )
        ctx.expect_gap(
            lid,
            upper_plate,
            axis="y",
            positive_elem="right_skirt",
            negative_elem="plate",
            min_gap=0.020,
            max_gap=0.040,
            name="lid side shell break surrounds upper plate",
        )
        ctx.expect_overlap(
            upper_plate,
            lower_plate,
            axes="xy",
            elem_a="plate",
            elem_b="plate",
            min_overlap=0.180,
            name="upper and lower cooking plates stay aligned when closed",
        )
        ctx.expect_gap(
            upper_plate,
            lower_plate,
            axis="z",
            positive_elem="plate",
            negative_elem="plate",
            min_gap=0.008,
            max_gap=0.012,
            name="closed plates keep a narrow working gap",
        )
    ctx.expect_contact(
        dial,
        base,
        elem_a="shaft",
        elem_b="right_wall",
        name="dial shaft mounts directly to the side wall",
    )
    ctx.expect_origin_gap(
        dial,
        base,
        axis="y",
        min_gap=0.130,
        max_gap=0.140,
        name="dial sits on the right housing side",
    )
    with ctx.pose({dial_joint: 1.6}):
        ctx.expect_contact(
            dial,
            base,
            elem_a="shaft",
            elem_b="right_wall",
            name="dial stays seated while rotating",
        )

    if lid_limits is not None and lid_limits.upper is not None:
        closed_grip = ctx.part_element_world_aabb(lid, elem="front_grip")
        with ctx.pose({lid_hinge: lid_limits.upper}):
            ctx.expect_gap(
                upper_plate,
                base,
                axis="z",
                min_gap=0.022,
                name="opened lid lifts the upper plate clear of the base",
            )
            open_grip = ctx.part_element_world_aabb(lid, elem="front_grip")

        grip_rises = (
            closed_grip is not None
            and open_grip is not None
            and open_grip[0][2] > closed_grip[0][2] + 0.12
        )
        ctx.check(
            "front grip rises when the lid opens",
            grip_rises,
            details=f"closed_grip={closed_grip}, open_grip={open_grip}",
        )

    return ctx.report()


object_model = build_object_model()
