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


def x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Visual origin for a URDF cylinder whose length should run along +X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yawing_telescoping_spindle_arm")

    model.material("base_casting", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("bearing_black", rgba=(0.02, 0.022, 0.024, 1.0))
    model.material("turntable_blue", rgba=(0.19, 0.27, 0.34, 1.0))
    model.material("beam_oxide", rgba=(0.27, 0.34, 0.31, 1.0))
    model.material("slide_steel", rgba=(0.67, 0.69, 0.66, 1.0))
    model.material("wear_strip", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("spindle_orange", rgba=(0.86, 0.42, 0.12, 1.0))
    model.material("tool_dark", rgba=(0.03, 0.035, 0.038, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.78, 0.52, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material="base_casting",
        name="floor_housing",
    )
    base.visual(
        Box((0.48, 0.36, 0.15)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material="base_casting",
        name="upper_housing",
    )
    base.visual(
        Cylinder(radius=0.215, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material="bearing_black",
        name="yaw_bearing",
    )
    base.visual(
        Cylinder(radius=0.115, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material="bearing_black",
        name="center_boss",
    )

    beam = model.part("beam")
    beam.visual(
        Cylinder(radius=0.245, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material="turntable_blue",
        name="turntable_disk",
    )
    beam.visual(
        Box((0.30, 0.30, 0.19)),
        origin=Origin(xyz=(0.20, 0.0, 0.145)),
        material="turntable_blue",
        name="saddle_column",
    )
    beam.visual(
        Box((0.42, 0.022, 0.19)),
        origin=Origin(xyz=(0.34, 0.135, 0.190)),
        material="turntable_blue",
        name="side_gusset_0",
    )
    beam.visual(
        Box((0.42, 0.022, 0.19)),
        origin=Origin(xyz=(0.34, -0.135, 0.190)),
        material="turntable_blue",
        name="side_gusset_1",
    )
    # Straight hollow guide beam: four long sleeve walls with a central clearance
    # tunnel for the sliding forearm.
    beam.visual(
        Box((1.02, 0.22, 0.035)),
        origin=Origin(xyz=(0.59, 0.0, 0.3825)),
        material="beam_oxide",
        name="guide_top",
    )
    beam.visual(
        Box((1.02, 0.22, 0.035)),
        origin=Origin(xyz=(0.59, 0.0, 0.2575)),
        material="beam_oxide",
        name="guide_bottom",
    )
    beam.visual(
        Box((1.02, 0.040, 0.160)),
        origin=Origin(xyz=(0.59, 0.090, 0.320)),
        material="beam_oxide",
        name="guide_side_pos",
    )
    beam.visual(
        Box((1.02, 0.040, 0.160)),
        origin=Origin(xyz=(0.59, -0.090, 0.320)),
        material="beam_oxide",
        name="guide_side_neg",
    )
    beam.visual(
        Box((0.075, 0.24, 0.18)),
        origin=Origin(xyz=(0.065, 0.0, 0.320)),
        material="beam_oxide",
        name="rear_stop_block",
    )
    beam.visual(
        Box((0.080, 0.24, 0.036)),
        origin=Origin(xyz=(1.105, 0.0, 0.390)),
        material="beam_oxide",
        name="front_collar_top",
    )
    beam.visual(
        Box((0.080, 0.24, 0.036)),
        origin=Origin(xyz=(1.105, 0.0, 0.250)),
        material="beam_oxide",
        name="front_collar_bottom",
    )
    beam.visual(
        Box((0.080, 0.044, 0.180)),
        origin=Origin(xyz=(1.105, 0.098, 0.320)),
        material="beam_oxide",
        name="front_collar_side_pos",
    )
    beam.visual(
        Box((0.080, 0.044, 0.180)),
        origin=Origin(xyz=(1.105, -0.098, 0.320)),
        material="beam_oxide",
        name="front_collar_side_neg",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.96, 0.108, 0.060)),
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
        material="slide_steel",
        name="inner_member",
    )
    forearm.visual(
        Box((0.82, 0.022, 0.066)),
        origin=Origin(xyz=(0.16, 0.059, 0.0)),
        material="wear_strip",
        name="slide_strip_pos",
    )
    forearm.visual(
        Box((0.82, 0.022, 0.066)),
        origin=Origin(xyz=(0.16, -0.059, 0.0)),
        material="wear_strip",
        name="slide_strip_neg",
    )
    forearm.visual(
        Box((0.120, 0.160, 0.120)),
        origin=Origin(xyz=(0.640, 0.0, 0.0)),
        material="slide_steel",
        name="end_carriage",
    )
    forearm.visual(
        Cylinder(radius=0.074, length=0.060),
        origin=x_cylinder_origin(0.710, 0.0, 0.0),
        material="bearing_black",
        name="bearing_shell",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.052, length=0.082),
        origin=x_cylinder_origin(0.041, 0.0, 0.0),
        material="spindle_orange",
        name="rear_spindle",
    )
    spindle.visual(
        Cylinder(radius=0.039, length=0.066),
        origin=x_cylinder_origin(0.113, 0.0, 0.0),
        material="spindle_orange",
        name="front_cap",
    )
    spindle.visual(
        Cylinder(radius=0.023, length=0.074),
        origin=x_cylinder_origin(0.181, 0.0, 0.0),
        material="tool_dark",
        name="collet",
    )
    spindle.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=x_cylinder_origin(0.239, 0.0, 0.0),
        material="tool_dark",
        name="tool_stub",
    )
    spindle.visual(
        Box((0.020, 0.014, 0.012)),
        origin=Origin(xyz=(0.075, 0.0, 0.049)),
        material="tool_dark",
        name="index_mark",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "forearm_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=forearm,
        origin=Origin(xyz=(0.580, 0.0, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.30, lower=0.0, upper=0.340),
    )
    model.articulation(
        "spindle_spin",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=spindle,
        origin=Origin(xyz=(0.740, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=12.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    forearm = object_model.get_part("forearm")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("forearm_slide")
    spin = object_model.get_articulation("spindle_spin")

    ctx.expect_contact(
        base,
        beam,
        elem_a="yaw_bearing",
        elem_b="turntable_disk",
        contact_tol=1e-5,
        name="turntable is seated on yaw bearing",
    )

    for pose_name, extension in (("retracted", 0.0), ("extended", 0.340)):
        with ctx.pose({slide: extension}):
            ctx.expect_gap(
                beam,
                forearm,
                axis="y",
                positive_elem="guide_side_pos",
                negative_elem="inner_member",
                min_gap=0.010,
                name=f"{pose_name} positive side guide clearance",
            )
            ctx.expect_gap(
                forearm,
                beam,
                axis="y",
                positive_elem="inner_member",
                negative_elem="guide_side_neg",
                min_gap=0.010,
                name=f"{pose_name} negative side guide clearance",
            )
            ctx.expect_gap(
                beam,
                forearm,
                axis="z",
                positive_elem="guide_top",
                negative_elem="inner_member",
                min_gap=0.010,
                name=f"{pose_name} top guide clearance",
            )
            ctx.expect_gap(
                forearm,
                beam,
                axis="z",
                positive_elem="inner_member",
                negative_elem="guide_bottom",
                min_gap=0.010,
                name=f"{pose_name} bottom guide clearance",
            )
            ctx.expect_overlap(
                forearm,
                beam,
                axes="x",
                elem_a="inner_member",
                elem_b="guide_top",
                min_overlap=0.45,
                name=f"{pose_name} forearm remains captured in beam",
            )

    ctx.expect_contact(
        forearm,
        spindle,
        elem_a="bearing_shell",
        elem_b="rear_spindle",
        contact_tol=1e-5,
        name="spindle nose is supported by bearing shell",
    )
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_gap(
            spindle,
            forearm,
            axis="x",
            positive_elem="rear_spindle",
            negative_elem="bearing_shell",
            max_penetration=1e-5,
            max_gap=1e-5,
            name="spindle rotation stays in front of bearing",
        )

    return ctx.report()


object_model = build_object_model()
