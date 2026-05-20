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
    model = ArticulatedObject(name="bridge_capped_roll_module")

    saddle_paint = model.material("saddle_paint", rgba=(0.18, 0.27, 0.33, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    output_metal = model.material("output_metal", rgba=(0.78, 0.80, 0.78, 1.0))
    witness_red = model.material("witness_red", rgba=(0.78, 0.08, 0.05, 1.0))

    lower_saddle = model.part("lower_saddle")
    lower_saddle.visual(
        Box((0.34, 0.20, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=saddle_paint,
        name="base_plate",
    )
    lower_saddle.visual(
        Box((0.22, 0.105, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=saddle_paint,
        name="cradle_pad",
    )
    for y, name in ((-0.086, "side_wall_0"), (0.086, "side_wall_1")):
        lower_saddle.visual(
            Box((0.22, 0.028, 0.120)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material=saddle_paint,
            name=name,
        )
    lower_saddle.visual(
        Box((0.22, 0.20, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=saddle_paint,
        name="bridge_cap",
    )

    lower_saddle.visual(
        Cylinder(radius=0.040, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="cartridge_shell",
    )
    lower_saddle.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(xyz=(0.111, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="front_bearing",
    )
    lower_saddle.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(xyz=(-0.111, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="rear_bearing",
    )

    for index, (x, y) in enumerate(
        ((-0.070, -0.062), (-0.070, 0.062), (0.070, -0.062), (0.070, 0.062))
    ):
        lower_saddle.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(x, y, 0.181)),
            material=bright_steel,
            name=f"bridge_bolt_{index}",
        )

    output_face = model.part("output_face")
    output_face.visual(
        Cylinder(radius=0.054, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=output_metal,
        name="face_disc",
    )
    output_face.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="center_boss",
    )
    output_face.visual(
        Box((0.005, 0.014, 0.032)),
        origin=Origin(xyz=(0.031, 0.0, 0.026)),
        material=witness_red,
        name="rotation_mark",
    )

    model.articulation(
        "face_spin",
        ArticulationType.REVOLUTE,
        parent=lower_saddle,
        child=output_face,
        origin=Origin(xyz=(0.120, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=12.0,
            lower=-math.pi,
            upper=math.pi,
        ),
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

    lower_saddle = object_model.get_part("lower_saddle")
    output_face = object_model.get_part("output_face")
    face_spin = object_model.get_articulation("face_spin")

    ctx.check(
        "single revolute output joint",
        len(object_model.articulations) == 1
        and face_spin.articulation_type == ArticulationType.REVOLUTE
        and tuple(face_spin.axis) == (1.0, 0.0, 0.0),
        details=f"joint_count={len(object_model.articulations)}, type={face_spin.articulation_type}, axis={face_spin.axis}",
    )
    ctx.expect_gap(
        output_face,
        lower_saddle,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="face_disc",
        negative_elem="front_bearing",
        name="output face seats on front bearing",
    )
    ctx.expect_overlap(
        output_face,
        lower_saddle,
        axes="yz",
        elem_a="face_disc",
        elem_b="front_bearing",
        min_overlap=0.080,
        name="output face is concentric with supported roll axis",
    )

    def _center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_mark = ctx.part_element_world_aabb(output_face, elem="rotation_mark")
    rest_center = _center(rest_mark) if rest_mark is not None else None
    with ctx.pose({face_spin: math.pi / 2.0}):
        turned_mark = ctx.part_element_world_aabb(output_face, elem="rotation_mark")
        turned_center = _center(turned_mark) if turned_mark is not None else None

    ctx.check(
        "witness mark follows face spin",
        rest_center is not None
        and turned_center is not None
        and rest_center[2] > 0.125
        and abs(rest_center[1]) < 0.010
        and turned_center[1] < -0.020
        and abs(turned_center[2] - 0.105) < 0.012,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
