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
    model = ArticulatedObject(name="yawing_telescoping_spindle_arm")

    cast_iron = model.material("cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.02, 0.02, 0.025, 1.0))
    blue_paint = model.material("blue_paint", rgba=(0.08, 0.22, 0.58, 1.0))
    dark_rail = model.material("dark_rail", rgba=(0.16, 0.17, 0.18, 1.0))
    slide_orange = model.material("slide_orange", rgba=(0.93, 0.46, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.01, 0.01, 0.012, 1.0))
    red_mark = model.material("red_mark", rgba=(0.95, 0.05, 0.03, 1.0))

    # Fixed floor pedestal.  The upper black ring is the stationary bearing race
    # that the yawing turntable sits on.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.34, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.24, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
        material=bearing_black,
        name="bearing_race",
    )

    # Yawing base and outer sleeve.  The straight main beam is built as an open
    # rectangular tube so the center stage can slide through it without relying
    # on overlap allowances.
    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.22, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blue_paint,
        name="rotating_disc",
    )
    turntable.visual(
        Cylinder(radius=0.16, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=dark_rail,
        name="top_bearing_cap",
    )
    turntable.visual(
        Cylinder(radius=0.07, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=blue_paint,
        name="riser_column",
    )
    turntable.visual(
        Box((0.24, 0.18, 0.05)),
        origin=Origin(xyz=(0.10, 0.0, 0.2625)),
        material=blue_paint,
        name="beam_saddle",
    )

    beam_x_center = 0.61
    beam_length = 0.90
    beam_z = 0.35
    turntable.visual(
        Box((beam_length, 0.19, 0.025)),
        origin=Origin(xyz=(beam_x_center, 0.0, beam_z + 0.0625)),
        material=dark_rail,
        name="beam_top_rail",
    )
    turntable.visual(
        Box((beam_length, 0.19, 0.025)),
        origin=Origin(xyz=(beam_x_center, 0.0, beam_z - 0.0625)),
        material=dark_rail,
        name="beam_bottom_rail",
    )
    turntable.visual(
        Box((beam_length, 0.025, 0.125)),
        origin=Origin(xyz=(beam_x_center, 0.0875, beam_z)),
        material=dark_rail,
        name="beam_side_pos",
    )
    turntable.visual(
        Box((beam_length, 0.025, 0.125)),
        origin=Origin(xyz=(beam_x_center, -0.0875, beam_z)),
        material=dark_rail,
        name="beam_side_neg",
    )
    # Stiffening collars at the sleeve mouth and root, shaped as four plates
    # rather than a solid block so the sliding stage remains clear.
    for x, suffix in ((0.17, "root"), (1.05, "mouth")):
        turntable.visual(
            Box((0.035, 0.205, 0.025)),
            origin=Origin(xyz=(x, 0.0, beam_z + 0.071)),
            material=blue_paint,
            name=f"{suffix}_collar_top",
        )
        turntable.visual(
            Box((0.035, 0.205, 0.025)),
            origin=Origin(xyz=(x, 0.0, beam_z - 0.071)),
            material=blue_paint,
            name=f"{suffix}_collar_bottom",
        )
        turntable.visual(
            Box((0.035, 0.025, 0.145)),
            origin=Origin(xyz=(x, 0.101, beam_z)),
            material=blue_paint,
            name=f"{suffix}_collar_side_pos",
        )
        turntable.visual(
            Box((0.035, 0.025, 0.145)),
            origin=Origin(xyz=(x, -0.101, beam_z)),
            material=blue_paint,
            name=f"{suffix}_collar_side_neg",
        )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )

    # Sliding forearm stage.  Its part frame sits at the sleeve centerline entry,
    # so positive prismatic motion extends the orange stage along the beam.
    forearm = model.part("forearm")
    forearm.visual(
        Box((0.82, 0.11, 0.07)),
        origin=Origin(xyz=(0.43, 0.0, 0.0)),
        material=slide_orange,
        name="center_stage",
    )
    forearm.visual(
        Box((0.08, 0.13, 0.09)),
        origin=Origin(xyz=(0.80, 0.0, 0.0)),
        material=slide_orange,
        name="tip_block",
    )
    forearm.visual(
        Box((0.50, 0.030, 0.017)),
        origin=Origin(xyz=(0.42, 0.0, 0.0415)),
        material=rubber_black,
        name="top_glide",
    )
    forearm.visual(
        Box((0.50, 0.030, 0.017)),
        origin=Origin(xyz=(0.42, 0.0, -0.0415)),
        material=rubber_black,
        name="bottom_glide",
    )
    forearm.visual(
        Box((0.50, 0.022, 0.030)),
        origin=Origin(xyz=(0.42, 0.064, 0.0)),
        material=rubber_black,
        name="side_glide_pos",
    )
    forearm.visual(
        Box((0.50, 0.022, 0.030)),
        origin=Origin(xyz=(0.42, -0.064, 0.0)),
        material=rubber_black,
        name="side_glide_neg",
    )
    forearm.visual(
        Box((0.012, 0.116, 0.076)),
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
        material=brushed_steel,
        name="extension_mark_0",
    )
    forearm.visual(
        Box((0.012, 0.116, 0.076)),
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        material=brushed_steel,
        name="extension_mark_1",
    )

    model.articulation(
        "extension",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=forearm,
        origin=Origin(xyz=(0.24, 0.0, beam_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.32),
    )

    # Rotary nose/spindle at the sliding stage tip.  Cylinders are rotated so
    # their local Z axes become the tool axis along local +X.
    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="spindle_housing",
    )
    spindle.visual(
        Cylinder(radius=0.061, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="nose_ring",
    )
    spindle.visual(
        Cylinder(radius=0.026, length=0.07),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tool_collet",
    )
    spindle.visual(
        Cylinder(radius=0.012, length=0.09),
        origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tool_bit",
    )
    spindle.visual(
        Box((0.06, 0.012, 0.008)),
        origin=Origin(xyz=(0.065, 0.0, 0.058)),
        material=red_mark,
        name="rotation_stripe",
    )

    model.articulation(
        "spindle_spin",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=spindle,
        origin=Origin(xyz=(0.84, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turntable = object_model.get_part("turntable")
    forearm = object_model.get_part("forearm")
    spindle = object_model.get_part("spindle")
    yaw = object_model.get_articulation("yaw")
    extension = object_model.get_articulation("extension")
    spindle_spin = object_model.get_articulation("spindle_spin")

    ctx.check(
        "three distinct motion layers",
        {yaw.name, extension.name, spindle_spin.name} == {"yaw", "extension", "spindle_spin"},
        details="Expected yaw turntable, prismatic center stage, and rotating spindle joints.",
    )
    ctx.check(
        "yaw is vertical revolute",
        yaw.articulation_type == ArticulationType.REVOLUTE and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "forearm slides along beam",
        extension.articulation_type == ArticulationType.PRISMATIC
        and tuple(extension.axis) == (1.0, 0.0, 0.0)
        and extension.motion_limits is not None
        and extension.motion_limits.upper is not None
        and extension.motion_limits.upper > 0.25,
        details=f"type={extension.articulation_type}, axis={extension.axis}, limits={extension.motion_limits}",
    )
    ctx.check(
        "spindle rotates on tool axis",
        spindle_spin.articulation_type == ArticulationType.REVOLUTE
        and tuple(spindle_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spindle_spin.articulation_type}, axis={spindle_spin.axis}",
    )

    ctx.expect_contact(
        turntable,
        pedestal,
        elem_a="rotating_disc",
        elem_b="bearing_race",
        contact_tol=0.001,
        name="turntable is seated on bearing race",
    )
    ctx.expect_gap(
        turntable,
        forearm,
        axis="z",
        positive_elem="beam_top_rail",
        negative_elem="center_stage",
        min_gap=0.010,
        max_gap=0.025,
        name="center stage clears upper rail",
    )
    ctx.expect_gap(
        forearm,
        turntable,
        axis="z",
        positive_elem="center_stage",
        negative_elem="beam_bottom_rail",
        min_gap=0.010,
        max_gap=0.025,
        name="center stage clears lower rail",
    )
    ctx.expect_gap(
        turntable,
        forearm,
        axis="y",
        positive_elem="beam_side_pos",
        negative_elem="center_stage",
        min_gap=0.015,
        max_gap=0.030,
        name="center stage clears positive side rail",
    )
    ctx.expect_gap(
        forearm,
        turntable,
        axis="y",
        positive_elem="center_stage",
        negative_elem="beam_side_neg",
        min_gap=0.015,
        max_gap=0.030,
        name="center stage clears negative side rail",
    )
    ctx.expect_overlap(
        forearm,
        turntable,
        axes="x",
        elem_a="center_stage",
        elem_b="beam_top_rail",
        min_overlap=0.75,
        name="collapsed forearm remains captured in main beam",
    )
    ctx.expect_contact(
        spindle,
        forearm,
        elem_a="spindle_housing",
        elem_b="tip_block",
        contact_tol=0.001,
        name="spindle nose mounts to sliding forearm tip",
    )

    rest_pos = ctx.part_world_position(forearm)
    with ctx.pose({extension: 0.32}):
        ctx.expect_overlap(
            forearm,
            turntable,
            axes="x",
            elem_a="center_stage",
            elem_b="beam_top_rail",
            min_overlap=0.45,
            name="extended forearm still retains sleeve insertion",
        )
        extended_pos = ctx.part_world_position(forearm)
    ctx.check(
        "forearm extension moves outward along beam",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    def _aabb_center_z(box):
        if box is None:
            return None
        return (box[0][2] + box[1][2]) / 2.0

    stripe_rest_z = _aabb_center_z(ctx.part_element_world_aabb(spindle, elem="rotation_stripe"))
    with ctx.pose({spindle_spin: math.pi / 2.0}):
        stripe_rotated_z = _aabb_center_z(ctx.part_element_world_aabb(spindle, elem="rotation_stripe"))
    ctx.check(
        "spindle stripe visibly rotates about nose axis",
        stripe_rest_z is not None and stripe_rotated_z is not None and stripe_rest_z - stripe_rotated_z > 0.04,
        details=f"rest_z={stripe_rest_z}, rotated_z={stripe_rotated_z}",
    )

    return ctx.report()


object_model = build_object_model()
