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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_transfer_axis")

    painted = model.material("painted_warm_gray", rgba=(0.42, 0.44, 0.45, 1.0))
    dark = model.material("black_oxide_rail", rgba=(0.04, 0.045, 0.05, 1.0))
    anodized = model.material("blue_anodized_carriage", rgba=(0.08, 0.22, 0.46, 1.0))
    face_mat = model.material("machined_square_face", rgba=(0.72, 0.75, 0.76, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    fastener = model.material("dark_socket_screws", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.45, 0.42, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=painted,
        name="base_plate",
    )
    base.visual(
        Box((1.28, 0.045, 0.026)),
        origin=Origin(xyz=(0.0, -0.165, 0.046)),
        material=dark,
        name="base_rail_0",
    )
    base.visual(
        Box((1.28, 0.045, 0.026)),
        origin=Origin(xyz=(0.0, 0.165, 0.046)),
        material=dark,
        name="base_rail_1",
    )
    for i, x in enumerate((-0.67, 0.67)):
        base.visual(
            Box((0.050, 0.35, 0.047)),
            origin=Origin(xyz=(x, 0.0, 0.0465)),
            material=painted,
            name=f"end_stop_{i}",
        )
    for ix, x in enumerate((-0.56, 0.56)):
        for iy, y in enumerate((-0.155, 0.155)):
            base.visual(
                Box((0.12, 0.065, 0.014)),
                origin=Origin(xyz=(x, y, 0.007)),
                material=rubber,
                name=f"foot_{ix}_{iy}",
            )

    first_slide = model.part("first_slide")
    first_slide.visual(
        Box((1.08, 0.32, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=painted,
        name="first_deck",
    )
    first_slide.visual(
        Box((1.00, 0.035, 0.020)),
        origin=Origin(xyz=(0.0, -0.105, 0.040)),
        material=dark,
        name="first_rail_0",
    )
    first_slide.visual(
        Box((1.00, 0.035, 0.020)),
        origin=Origin(xyz=(0.0, 0.105, 0.040)),
        material=dark,
        name="first_rail_1",
    )
    first_slide.visual(
        Box((0.075, 0.26, 0.044)),
        origin=Origin(xyz=(0.49, 0.0, 0.034)),
        material=painted,
        name="first_end_block",
    )
    first_slide.visual(
        Box((0.080, 0.24, 0.040)),
        origin=Origin(xyz=(-0.49, 0.0, 0.032)),
        material=painted,
        name="first_rear_block",
    )

    second_slide = model.part("second_slide")
    second_slide.visual(
        Box((0.70, 0.24, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=anodized,
        name="second_deck",
    )
    second_slide.visual(
        Box((0.58, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, -0.074, 0.035)),
        material=dark,
        name="second_rail_0",
    )
    second_slide.visual(
        Box((0.58, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.074, 0.035)),
        material=dark,
        name="second_rail_1",
    )
    second_slide.visual(
        Box((0.070, 0.19, 0.038)),
        origin=Origin(xyz=(0.315, 0.0, 0.030)),
        material=anodized,
        name="second_end_block",
    )
    second_slide.visual(
        Box((0.065, 0.18, 0.034)),
        origin=Origin(xyz=(-0.315, 0.0, 0.028)),
        material=anodized,
        name="second_rear_block",
    )

    terminal_stage = model.part("terminal_stage")
    terminal_stage.visual(
        Box((0.26, 0.15, 0.024)),
        origin=Origin(xyz=(-0.030, 0.0, 0.012)),
        material=painted,
        name="terminal_foot",
    )
    terminal_stage.visual(
        Box((0.110, 0.095, 0.060)),
        origin=Origin(xyz=(0.060, 0.0, 0.042)),
        material=painted,
        name="terminal_neck",
    )
    terminal_stage.visual(
        Box((0.028, 0.120, 0.120)),
        origin=Origin(xyz=(0.122, 0.0, 0.071)),
        material=face_mat,
        name="square_face",
    )
    for i, (y, z) in enumerate(((-0.040, 0.035), (0.040, 0.035), (-0.040, 0.107), (0.040, 0.107))):
        terminal_stage.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.141, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=fastener,
            name=f"face_screw_{i}",
        )

    model.articulation(
        "base_to_first_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first_slide,
        origin=Origin(xyz=(-0.09, 0.0, 0.059)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.55, lower=0.0, upper=0.30),
    )
    model.articulation(
        "first_to_second_slide",
        ArticulationType.PRISMATIC,
        parent=first_slide,
        child=second_slide,
        origin=Origin(xyz=(-0.04, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.50, lower=0.0, upper=0.22),
    )
    model.articulation(
        "second_to_terminal_stage",
        ArticulationType.PRISMATIC,
        parent=second_slide,
        child=terminal_stage,
        origin=Origin(xyz=(-0.02, 0.0, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=0.0, upper=0.12),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    first = object_model.get_part("first_slide")
    second = object_model.get_part("second_slide")
    terminal = object_model.get_part("terminal_stage")
    j1 = object_model.get_articulation("base_to_first_slide")
    j2 = object_model.get_articulation("first_to_second_slide")
    j3 = object_model.get_articulation("second_to_terminal_stage")

    joints = (j1, j2, j3)
    ctx.check(
        "three serial prismatic joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in joints)
        and j1.parent == "base"
        and j1.child == "first_slide"
        and j2.parent == "first_slide"
        and j2.child == "second_slide"
        and j3.parent == "second_slide"
        and j3.child == "terminal_stage",
        details=f"articulations={object_model.articulations}",
    )

    ctx.expect_contact(
        first,
        base,
        elem_a="first_deck",
        elem_b="base_rail_0",
        name="first slide sits on base rail",
    )
    ctx.expect_contact(
        second,
        first,
        elem_a="second_deck",
        elem_b="first_rail_0",
        name="second slide sits on first rail",
    )
    ctx.expect_contact(
        terminal,
        second,
        elem_a="terminal_foot",
        elem_b="second_rail_0",
        name="terminal stage sits on second rail",
    )

    ctx.expect_overlap(first, base, axes="x", elem_a="first_deck", elem_b="base_rail_0", min_overlap=0.80)
    ctx.expect_overlap(second, first, axes="x", elem_a="second_deck", elem_b="first_rail_0", min_overlap=0.45)
    ctx.expect_overlap(
        terminal,
        second,
        axes="x",
        elem_a="terminal_foot",
        elem_b="second_rail_0",
        min_overlap=0.18,
    )

    face_aabb = ctx.part_element_world_aabb(terminal, elem="square_face")
    face_is_square = False
    if face_aabb is not None:
        lo, hi = face_aabb
        dy = hi[1] - lo[1]
        dz = hi[2] - lo[2]
        face_is_square = abs(dy - dz) < 1e-6 and dy >= 0.11
    ctx.check("terminal stage has square face", face_is_square, details=f"face_aabb={face_aabb}")

    rest_x = ctx.part_world_position(terminal)[0]
    with ctx.pose({j1: j1.motion_limits.upper, j2: j2.motion_limits.upper, j3: j3.motion_limits.upper}):
        ctx.expect_overlap(
            first,
            base,
            axes="x",
            elem_a="first_deck",
            elem_b="base_rail_0",
            min_overlap=0.50,
            name="first slide remains supported at full travel",
        )
        ctx.expect_overlap(
            second,
            first,
            axes="x",
            elem_a="second_deck",
            elem_b="first_rail_0",
            min_overlap=0.28,
            name="second slide remains supported at full travel",
        )
        ctx.expect_overlap(
            terminal,
            second,
            axes="x",
            elem_a="terminal_foot",
            elem_b="second_rail_0",
            min_overlap=0.08,
            name="terminal stage remains supported at full travel",
        )
        extended_x = ctx.part_world_position(terminal)[0]

    ctx.check(
        "terminal stage extends along transfer axis",
        extended_x > rest_x + 0.60,
        details=f"rest_x={rest_x}, extended_x={extended_x}",
    )

    return ctx.report()


object_model = build_object_model()
