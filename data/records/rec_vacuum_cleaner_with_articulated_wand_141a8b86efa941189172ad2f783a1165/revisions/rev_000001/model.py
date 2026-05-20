from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


WAND_0_LENGTH = 0.18
WAND_1_LENGTH = 0.18
BODY_WAND_ANGLE = 0.30
WAND_ELBOW_SET_ANGLE = -0.18
NOZZLE_LEVEL_SET_ANGLE = -(BODY_WAND_ANGLE + WAND_ELBOW_SET_ANGLE)


def _rounded_box_mesh(name: str, size: tuple[float, float, float], fillet: float):
    """Small CadQuery helper for appliance housings with softened molded edges."""
    shape = cq.Workplane("XY").box(*size).edges().fillet(fillet)
    return mesh_from_cadquery(shape, name, tolerance=0.0007, angular_tolerance=0.08)


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    samples_per_segment: int = 14,
):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_vacuum_with_articulated_wand")

    shell_red = model.material("shell_red", rgba=(0.70, 0.08, 0.06, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.058, 0.062, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.72, 0.75, 0.77, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.31, 0.32, 0.33, 1.0))
    brush_strip = model.material("brush_strip", rgba=(0.02, 0.018, 0.015, 1.0))

    main_body = model.part("main_body")
    main_body.visual(
        _rounded_box_mesh("main_body_shell", (0.30, 0.18, 0.13), 0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=shell_red,
        name="body_shell",
    )
    main_body.visual(
        Box((0.19, 0.035, 0.018)),
        origin=Origin(xyz=(-0.015, 0.0, 0.151)),
        material=dark_plastic,
        name="top_inset",
    )
    main_body.visual(
        _tube_mesh(
            "carry_handle",
            [
                (-0.098, -0.045, 0.136),
                (-0.066, -0.053, 0.190),
                (0.020, -0.053, 0.205),
                (0.088, -0.045, 0.138),
            ],
            radius=0.007,
            samples_per_segment=18,
        ),
        material=dark_plastic,
        name="carry_handle",
    )

    # Side wheel pods and vent slats are embedded slightly into the shell so the
    # whole root part reads as a connected molded vacuum body.
    for side, y in (("side_0", -0.098), ("side_1", 0.098)):
        main_body.visual(
            Cylinder(radius=0.034, length=0.026),
            origin=Origin(xyz=(-0.092, y, 0.043), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name=f"{side}_wheel",
        )
    for idx, x in enumerate((-0.045, -0.015, 0.015, 0.045)):
        main_body.visual(
            Box((0.018, 0.006, 0.055)),
            origin=Origin(xyz=(x, -0.092, 0.085)),
            material=dark_plastic,
            name=f"vent_slit_{idx}",
        )

    connector_dir = (
        math.cos(BODY_WAND_ANGLE),
        0.0,
        -math.sin(BODY_WAND_ANGLE),
    )
    connector_joint = (0.155, 0.0, 0.128)
    connector_length = 0.064
    connector_center = (
        connector_joint[0] - connector_dir[0] * connector_length * 0.5,
        connector_joint[1],
        connector_joint[2] - connector_dir[2] * connector_length * 0.5,
    )
    main_body.visual(
        Cylinder(radius=0.026, length=connector_length),
        origin=Origin(
            xyz=connector_center,
            rpy=(0.0, math.pi / 2.0 + BODY_WAND_ANGLE, 0.0),
        ),
        material=dark_plastic,
        name="wand_socket",
    )

    wand_0 = model.part("wand_0")
    wand_0.visual(
        Cylinder(radius=0.015, length=WAND_0_LENGTH - 0.018),
        origin=Origin(xyz=((WAND_0_LENGTH - 0.018) * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wand_metal,
        name="tube",
    )
    wand_0.visual(
        Box((0.030, 0.048, 0.040)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=hinge_gray,
        name="proximal_collar",
    )
    wand_0.visual(
        Box((0.030, 0.052, 0.042)),
        origin=Origin(xyz=(WAND_0_LENGTH - 0.015, 0.0, 0.0)),
        material=hinge_gray,
        name="distal_collar",
    )

    wand_1 = model.part("wand_1")
    wand_1.visual(
        Cylinder(radius=0.0135, length=WAND_1_LENGTH),
        origin=Origin(xyz=(WAND_1_LENGTH * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wand_metal,
        name="tube",
    )
    wand_1.visual(
        Box((0.022, 0.046, 0.038)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=hinge_gray,
        name="proximal_collar",
    )
    wand_1.visual(
        Box((0.030, 0.052, 0.040)),
        origin=Origin(xyz=(WAND_1_LENGTH - 0.015, 0.0, 0.0)),
        material=hinge_gray,
        name="distal_collar",
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Cylinder(radius=0.018, length=0.086),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_gray,
        name="pitch_barrel",
    )
    floor_nozzle.visual(
        Box((0.062, 0.076, 0.034)),
        origin=Origin(xyz=(0.036, 0.0, -0.020)),
        material=dark_plastic,
        name="neck_block",
    )
    floor_nozzle.visual(
        _rounded_box_mesh("floor_nozzle_head", (0.120, 0.300, 0.035), 0.010),
        origin=Origin(xyz=(0.090, 0.0, -0.044)),
        material=dark_plastic,
        name="nozzle_head",
    )
    floor_nozzle.visual(
        Box((0.095, 0.270, 0.006)),
        origin=Origin(xyz=(0.098, 0.0, -0.064)),
        material=brush_strip,
        name="brush_strip",
    )
    floor_nozzle.visual(
        Box((0.105, 0.018, 0.012)),
        origin=Origin(xyz=(0.092, -0.155, -0.030)),
        material=black_rubber,
        name="side_bumper_0",
    )
    floor_nozzle.visual(
        Box((0.105, 0.018, 0.012)),
        origin=Origin(xyz=(0.092, 0.155, -0.030)),
        material=black_rubber,
        name="side_bumper_1",
    )

    model.articulation(
        "body_elbow",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=wand_0,
        origin=Origin(xyz=connector_joint, rpy=(0.0, BODY_WAND_ANGLE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.55, upper=0.65),
    )
    model.articulation(
        "wand_elbow",
        ArticulationType.REVOLUTE,
        parent=wand_0,
        child=wand_1,
        origin=Origin(xyz=(WAND_0_LENGTH, 0.0, 0.0), rpy=(0.0, WAND_ELBOW_SET_ANGLE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.2, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "nozzle_pitch",
        ArticulationType.REVOLUTE,
        parent=wand_1,
        child=floor_nozzle,
        origin=Origin(xyz=(WAND_1_LENGTH, 0.0, 0.0), rpy=(0.0, NOZZLE_LEVEL_SET_ANGLE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=-0.60, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    main_body = object_model.get_part("main_body")
    wand_0 = object_model.get_part("wand_0")
    wand_1 = object_model.get_part("wand_1")
    floor_nozzle = object_model.get_part("floor_nozzle")
    body_elbow = object_model.get_articulation("body_elbow")
    wand_elbow = object_model.get_articulation("wand_elbow")
    nozzle_pitch = object_model.get_articulation("nozzle_pitch")

    ctx.allow_overlap(
        wand_0,
        wand_1,
        elem_a="distal_collar",
        elem_b="tube",
        reason=(
            "The child wand tube is intentionally captured a few millimeters "
            "inside the simplified elbow collar, representing the hidden hinge sleeve."
        ),
    )

    ctx.check(
        "requested revolute joints are present",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (body_elbow, wand_elbow, nozzle_pitch)
        ),
        details="The vacuum needs two wand elbows plus a floor-nozzle pitch hinge.",
    )
    ctx.expect_contact(
        main_body,
        wand_0,
        elem_a="wand_socket",
        elem_b="proximal_collar",
        contact_tol=0.002,
        name="body socket meets first wand",
    )
    ctx.expect_contact(
        wand_0,
        wand_1,
        elem_a="distal_collar",
        elem_b="proximal_collar",
        contact_tol=0.012,
        name="wand collars meet at elbow",
    )
    ctx.expect_overlap(
        wand_0,
        wand_1,
        axes="x",
        elem_a="distal_collar",
        elem_b="tube",
        min_overlap=0.004,
        name="elbow sleeve retains child tube",
    )
    ctx.expect_contact(
        wand_1,
        floor_nozzle,
        elem_a="distal_collar",
        elem_b="pitch_barrel",
        contact_tol=0.004,
        name="wand end meets nozzle hinge",
    )

    rest_wand_1 = ctx.part_world_position(wand_1)
    with ctx.pose({body_elbow: 0.40, wand_elbow: -0.35}):
        bent_wand_1 = ctx.part_world_position(wand_1)
    ctx.check(
        "wand elbows change the chain pose",
        rest_wand_1 is not None
        and bent_wand_1 is not None
        and abs(bent_wand_1[2] - rest_wand_1[2]) > 0.015,
        details=f"rest={rest_wand_1}, bent={bent_wand_1}",
    )

    rest_head_aabb = ctx.part_element_world_aabb(floor_nozzle, elem="nozzle_head")
    with ctx.pose({nozzle_pitch: nozzle_pitch.motion_limits.upper}):
        pitched_head_aabb = ctx.part_element_world_aabb(floor_nozzle, elem="nozzle_head")
    rest_center_z = None if rest_head_aabb is None else (rest_head_aabb[0][2] + rest_head_aabb[1][2]) * 0.5
    pitched_center_z = None if pitched_head_aabb is None else (pitched_head_aabb[0][2] + pitched_head_aabb[1][2]) * 0.5
    ctx.check(
        "nozzle pitches about horizontal hinge",
        rest_center_z is not None
        and pitched_center_z is not None
        and pitched_center_z < rest_center_z - 0.012,
        details=f"rest_center_z={rest_center_z}, pitched_center_z={pitched_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
