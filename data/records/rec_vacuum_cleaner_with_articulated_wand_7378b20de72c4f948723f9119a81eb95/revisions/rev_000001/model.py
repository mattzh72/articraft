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
    model = ArticulatedObject(name="vacuum_with_articulated_wand")

    body_blue = model.material("body_blue", rgba=(0.06, 0.18, 0.40, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.66, 0.68, 1.0))
    accent = model.material("accent", rgba=(0.04, 0.55, 0.85, 1.0))

    wand_pitch = 0.35
    upper_len = 0.64
    lower_len = 0.68
    socket_x = 0.47
    socket_z = 0.57

    main_body = model.part("main_body")
    main_body.visual(
        Box((0.58, 0.32, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=body_blue,
        name="motor_housing",
    )
    main_body.visual(
        Box((0.38, 0.13, 0.075)),
        origin=Origin(xyz=(-0.04, 0.0, 0.705)),
        material=dark_plastic,
        name="top_grip",
    )
    main_body.visual(
        Box((0.44, 0.014, 0.09)),
        origin=Origin(xyz=(0.00, 0.163, 0.48)),
        material=accent,
        name="side_panel",
    )
    main_body.visual(
        Cylinder(radius=0.080, length=0.36),
        origin=Origin(xyz=(-0.18, 0.0, 0.19), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_wheel",
    )
    main_body.visual(
        Cylinder(radius=0.075, length=0.19),
        origin=Origin(xyz=(0.375, 0.0, socket_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="wand_socket",
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Box((upper_len - 0.044, 0.074, 0.054)),
        origin=Origin(xyz=(upper_len / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="upper_tube",
    )
    upper_wand.visual(
        Box((0.090, 0.105, 0.084)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=dark_plastic,
        name="proximal_collar",
    )
    upper_wand.visual(
        Cylinder(radius=0.052, length=0.140),
        origin=Origin(xyz=(upper_len - 0.052, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="elbow_knuckle",
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Box((lower_len - 0.044, 0.074, 0.054)),
        origin=Origin(xyz=(lower_len / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="lower_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.052, length=0.140),
        origin=Origin(xyz=(0.051, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="elbow_socket",
    )
    lower_wand.visual(
        Box((0.105, 0.112, 0.086)),
        origin=Origin(xyz=(lower_len - 0.0525, 0.0, 0.0)),
        material=dark_plastic,
        name="nozzle_collar",
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Cylinder(radius=0.040, length=0.220),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="pitch_hinge",
    )
    floor_nozzle.visual(
        Box((0.520, 0.360, 0.080)),
        origin=Origin(xyz=(0.260, 0.0, -0.075)),
        material=dark_plastic,
        name="nozzle_shell",
    )
    floor_nozzle.visual(
        Box((0.420, 0.295, 0.026)),
        origin=Origin(xyz=(0.330, 0.0, -0.124)),
        material=rubber,
        name="suction_lip",
    )
    floor_nozzle.visual(
        Box((0.110, 0.392, 0.040)),
        origin=Origin(xyz=(0.480, 0.0, -0.070)),
        material=accent,
        name="front_bumper",
    )

    model.articulation(
        "body_elbow",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=upper_wand,
        origin=Origin(xyz=(socket_x, 0.0, socket_z), rpy=(0.0, wand_pitch, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.45, upper=0.70),
    )
    model.articulation(
        "wand_elbow",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(upper_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.5, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "nozzle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(lower_len, 0.0, 0.0), rpy=(0.0, -wand_pitch, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.55, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_body = object_model.get_part("main_body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    body_elbow = object_model.get_articulation("body_elbow")
    wand_elbow = object_model.get_articulation("wand_elbow")
    nozzle_pitch = object_model.get_articulation("nozzle_pitch")

    ctx.allow_overlap(
        main_body,
        upper_wand,
        elem_a="wand_socket",
        elem_b="proximal_collar",
        reason="The upper wand collar is intentionally captured in the simplified body socket hinge.",
    )
    ctx.allow_overlap(
        lower_wand,
        upper_wand,
        elem_a="elbow_socket",
        elem_b="elbow_knuckle",
        reason="The two elbow barrels are intentionally seated together as a broad captured revolute hinge.",
    )
    ctx.allow_overlap(
        floor_nozzle,
        lower_wand,
        elem_a="pitch_hinge",
        elem_b="nozzle_collar",
        reason="The nozzle pitch barrel is intentionally seated in the wand collar as a captured hinge.",
    )

    ctx.check(
        "vacuum has articulated wand and pitching nozzle",
        len(object_model.parts) == 4 and len(object_model.articulations) == 3,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )

    ctx.expect_contact(
        main_body,
        upper_wand,
        elem_a="wand_socket",
        elem_b="proximal_collar",
        contact_tol=0.003,
        name="body socket carries the upper wand",
    )
    ctx.expect_contact(
        upper_wand,
        lower_wand,
        elem_a="elbow_knuckle",
        elem_b="elbow_socket",
        contact_tol=0.003,
        name="wand elbow knuckles meet",
    )
    ctx.expect_contact(
        lower_wand,
        floor_nozzle,
        elem_a="nozzle_collar",
        elem_b="pitch_hinge",
        contact_tol=0.003,
        name="floor nozzle hinge is mounted to wand",
    )

    rest_nozzle = ctx.part_world_position(floor_nozzle)
    with ctx.pose({body_elbow: 0.35, wand_elbow: -0.40}):
        bent_nozzle = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "elbow joints bend the wand chain",
        rest_nozzle is not None
        and bent_nozzle is not None
        and abs(bent_nozzle[2] - rest_nozzle[2]) > 0.06,
        details=f"rest={rest_nozzle}, bent={bent_nozzle}",
    )

    rest_shell = ctx.part_element_world_aabb(floor_nozzle, elem="nozzle_shell")
    with ctx.pose({nozzle_pitch: -0.40}):
        pitched_shell = ctx.part_element_world_aabb(floor_nozzle, elem="nozzle_shell")
    rest_center_z = None if rest_shell is None else (rest_shell[0][2] + rest_shell[1][2]) / 2.0
    pitched_center_z = None if pitched_shell is None else (pitched_shell[0][2] + pitched_shell[1][2]) / 2.0
    ctx.check(
        "floor nozzle pitches on a horizontal hinge",
        rest_center_z is not None
        and pitched_center_z is not None
        and pitched_center_z > rest_center_z + 0.03,
        details=f"rest_z={rest_center_z}, pitched_z={pitched_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
