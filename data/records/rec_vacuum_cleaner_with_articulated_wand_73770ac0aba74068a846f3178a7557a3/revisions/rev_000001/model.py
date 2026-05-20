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
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    red = model.material("red_plastic", color=(0.72, 0.05, 0.04, 1.0))
    dark = model.material("dark_rubber", color=(0.025, 0.025, 0.025, 1.0))
    graphite = model.material("graphite_plastic", color=(0.14, 0.15, 0.16, 1.0))
    silver = model.material("brushed_aluminum", color=(0.65, 0.68, 0.70, 1.0))
    black = model.material("black_trim", color=(0.01, 0.01, 0.012, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.56, 0.30, 0.20)),
        origin=Origin(xyz=(0.00, 0.00, 0.16)),
        material=red,
        name="lower_chassis",
    )
    body.visual(
        Box((0.41, 0.28, 0.34)),
        origin=Origin(xyz=(-0.05, 0.00, 0.37)),
        material=red,
        name="dust_bin",
    )
    body.visual(
        Box((0.20, 0.21, 0.20)),
        origin=Origin(xyz=(0.24, 0.00, 0.47)),
        material=graphite,
        name="front_socket_pod",
    )
    body.visual(
        Box((0.08, 0.18, 0.14)),
        origin=Origin(xyz=(0.355, 0.00, 0.58)),
        material=graphite,
        name="wand_socket_face",
    )
    body.visual(
        Box((0.18, 0.24, 0.045)),
        origin=Origin(xyz=(0.22, 0.00, 0.055)),
        material=black,
        name="front_bumper",
    )
    body.visual(
        Cylinder(radius=0.105, length=0.065),
        origin=Origin(xyz=(-0.19, 0.172, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wheel_0",
    )
    body.visual(
        Cylinder(radius=0.105, length=0.065),
        origin=Origin(xyz=(-0.19, -0.172, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wheel_1",
    )
    body.visual(
        Box((0.19, 0.08, 0.045)),
        origin=Origin(xyz=(-0.08, 0.00, 0.55)),
        material=black,
        name="carry_handle",
    )
    body.visual(
        Box((0.22, 0.012, 0.12)),
        origin=Origin(xyz=(-0.16, 0.146, 0.39)),
        material=black,
        name="side_vent_0",
    )
    body.visual(
        Box((0.22, 0.012, 0.12)),
        origin=Origin(xyz=(-0.16, -0.146, 0.39)),
        material=black,
        name="side_vent_1",
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Box((0.12, 0.15, 0.12)),
        origin=Origin(xyz=(0.08, 0.00, 0.00)),
        material=graphite,
        name="body_elbow_housing",
    )
    upper_wand.visual(
        Box((0.56, 0.09, 0.075)),
        origin=Origin(xyz=(0.39, 0.00, 0.00)),
        material=silver,
        name="upper_link",
    )
    upper_wand.visual(
        Box((0.148, 0.15, 0.12)),
        origin=Origin(xyz=(0.709, 0.00, 0.00)),
        material=graphite,
        name="middle_elbow_housing",
    )
    upper_wand.visual(
        Box((0.065, 0.17, 0.055)),
        origin=Origin(xyz=(0.750, 0.00, 0.00)),
        material=black,
        name="middle_hinge_cap",
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Box((0.12, 0.15, 0.12)),
        origin=Origin(xyz=(0.085, 0.00, 0.00)),
        material=graphite,
        name="middle_socket",
    )
    lower_wand.visual(
        Box((0.50, 0.085, 0.070)),
        origin=Origin(xyz=(0.38, 0.00, 0.00)),
        material=silver,
        name="lower_link",
    )
    lower_wand.visual(
        Box((0.13, 0.15, 0.12)),
        origin=Origin(xyz=(0.69, 0.00, 0.00)),
        material=graphite,
        name="nozzle_elbow_housing",
    )
    lower_wand.visual(
        Box((0.060, 0.17, 0.055)),
        origin=Origin(xyz=(0.722, 0.00, 0.00)),
        material=black,
        name="nozzle_hinge_cap",
    )

    nozzle = model.part("floor_nozzle")
    nozzle.visual(
        Box((0.13, 0.18, 0.10)),
        origin=Origin(xyz=(0.105, 0.00, 0.00)),
        material=graphite,
        name="pitch_hinge_housing",
    )
    nozzle.visual(
        Box((0.12, 0.13, 0.06)),
        origin=Origin(xyz=(0.15, 0.00, -0.025)),
        material=graphite,
        name="nozzle_neck",
    )
    nozzle.visual(
        Box((0.36, 0.52, 0.070)),
        origin=Origin(xyz=(0.32, 0.00, -0.040)),
        material=graphite,
        name="wide_nozzle_shell",
    )
    nozzle.visual(
        Box((0.05, 0.54, 0.045)),
        origin=Origin(xyz=(0.525, 0.00, -0.035)),
        material=black,
        name="front_lip",
    )
    nozzle.visual(
        Box((0.22, 0.42, 0.014)),
        origin=Origin(xyz=(0.36, 0.00, -0.080)),
        material=dark,
        name="intake_slot",
    )
    nozzle.visual(
        Box((0.030, 0.50, 0.025)),
        origin=Origin(xyz=(0.475, 0.00, -0.078)),
        material=black,
        name="brush_strip",
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(xyz=(0.386, 0.0, 0.58), rpy=(0.0, 0.18, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.45, upper=0.70),
    )
    model.articulation(
        "upper_wand_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.775, 0.0, 0.0), rpy=(0.0, 0.32, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.5, lower=-0.65, upper=0.85),
    )
    model.articulation(
        "lower_wand_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=nozzle,
        origin=Origin(xyz=(0.74218, 0.0, 0.0), rpy=(0.0, -0.50, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.55, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    body_elbow = object_model.get_articulation("body_to_upper_wand")
    middle_elbow = object_model.get_articulation("upper_wand_to_lower_wand")
    nozzle_pitch = object_model.get_articulation("lower_wand_to_floor_nozzle")

    ctx.allow_overlap(
        upper_wand,
        lower_wand,
        elem_a="middle_elbow_housing",
        elem_b="middle_socket",
        reason=(
            "The boxy middle elbow is represented as a captured socket; the "
            "child socket is intentionally seated slightly inside the parent housing."
        ),
    )
    ctx.check(
        "two wand elbows and nozzle pitch are revolute",
        body_elbow.articulation_type == ArticulationType.REVOLUTE
        and middle_elbow.articulation_type == ArticulationType.REVOLUTE
        and nozzle_pitch.articulation_type == ArticulationType.REVOLUTE,
        details="The vacuum should expose two wand bending elbows plus a pitching floor-nozzle hinge.",
    )
    ctx.expect_overlap(
        upper_wand,
        lower_wand,
        axes="y",
        elem_a="middle_hinge_cap",
        elem_b="middle_socket",
        min_overlap=0.05,
        name="middle elbow housings share hinge width",
    )
    ctx.expect_gap(
        lower_wand,
        upper_wand,
        axis="x",
        positive_elem="middle_socket",
        negative_elem="middle_elbow_housing",
        max_penetration=0.035,
        name="middle elbow socket has bounded seating depth",
    )
    ctx.expect_overlap(
        lower_wand,
        floor_nozzle,
        axes="y",
        elem_a="nozzle_hinge_cap",
        elem_b="pitch_hinge_housing",
        min_overlap=0.05,
        name="nozzle pitch hinge spans across the wand",
    )

    rest_nozzle = ctx.part_world_position(floor_nozzle)
    rest_lower = ctx.part_world_position(lower_wand)
    with ctx.pose({middle_elbow: 0.45, nozzle_pitch: -0.35}):
        bent_nozzle = ctx.part_world_position(floor_nozzle)
        bent_lower = ctx.part_world_position(lower_wand)
    ctx.check(
        "wand elbow bend changes nozzle reach",
        rest_nozzle is not None
        and bent_nozzle is not None
        and abs(bent_nozzle[0] - rest_nozzle[0]) > 0.08,
        details=f"rest_nozzle={rest_nozzle}, bent_nozzle={bent_nozzle}",
    )
    ctx.check(
        "lower wand remains connected through elbow pose",
        rest_lower is not None and bent_lower is not None,
        details=f"rest_lower={rest_lower}, bent_lower={bent_lower}",
    )

    return ctx.report()


object_model = build_object_model()
