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

BASE_TOP_Z = 0.032
JAW_FACE_Z = 0.104
JAW_CLOSED_X = 0.020
JAW_TRAVEL = 0.120


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machinist_vise")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.27, 0.29, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.76, 0.78, 0.81, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.14, 0.15, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.280, 0.170, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=cast_iron,
        name="swivel_ring",
    )
    for x_pos in (-0.102, 0.102):
        for y_pos in (-0.054, 0.054):
            base.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(x_pos, y_pos, 0.018)),
                material=black_oxide,
                name=f"clamp_boss_{int((x_pos + 0.102) * 1000)}_{int((y_pos + 0.054) * 1000)}",
            )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.090, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cast_iron,
        name="swivel_table",
    )
    body.visual(
        Box((0.160, 0.120, 0.020)),
        origin=Origin(xyz=(-0.030, 0.0, 0.022)),
        material=cast_iron,
        name="pedestal",
    )
    body.visual(
        Box((0.340, 0.042, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, 0.039)),
        material=cast_iron,
        name="guide_floor",
    )
    for rail_index, y_pos in enumerate((-0.036, 0.036)):
        body.visual(
            Box((0.340, 0.038, 0.022)),
            origin=Origin(xyz=(-0.010, y_pos, 0.057)),
            material=cast_iron,
            name=f"guide_rail_{rail_index}",
        )
    body.visual(
        Box((0.042, 0.152, 0.070)),
        origin=Origin(xyz=(-0.021, 0.0, JAW_FACE_Z)),
        material=cast_iron,
        name="fixed_jaw",
    )
    body.visual(
        Box((0.090, 0.110, 0.032)),
        origin=Origin(xyz=(-0.055, 0.0, 0.084)),
        material=cast_iron,
        name="rear_saddle",
    )
    body.visual(
        Box((0.082, 0.118, 0.010)),
        origin=Origin(xyz=(-0.040, 0.0, 0.141)),
        material=cast_iron,
        name="anvil",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=Origin(xyz=(0.145, 0.0, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="screw_nose",
    )

    sliding_jaw = model.part("sliding_jaw")
    sliding_jaw.visual(
        Box((0.178, 0.030, 0.022)),
        origin=Origin(xyz=(-0.092, 0.0, -0.046)),
        material=cast_iron,
        name="guide_tail",
    )
    sliding_jaw.visual(
        Box((0.080, 0.050, 0.030)),
        origin=Origin(xyz=(0.035, 0.0, -0.020)),
        material=cast_iron,
        name="support_web",
    )
    sliding_jaw.visual(
        Box((0.052, 0.150, 0.070)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=cast_iron,
        name="jaw_block",
    )
    sliding_jaw.visual(
        Box((0.072, 0.120, 0.010)),
        origin=Origin(xyz=(0.026, 0.0, 0.040)),
        material=cast_iron,
        name="top_cap",
    )
    sliding_jaw.visual(
        Cylinder(radius=0.015, length=0.034),
        origin=Origin(xyz=(0.086, 0.0, -0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="screw_boss",
    )

    rear_plate = model.part("rear_plate")
    rear_plate.visual(
        Box((0.008, 0.145, 0.050)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=machined_steel,
        name="plate",
    )
    rear_plate.visual(
        Box((0.002, 0.140, 0.036)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=machined_steel,
        name="tooth_band",
    )
    for index, z_pos in enumerate((-0.015, 0.015)):
        rear_plate.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(0.0055, 0.0, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_oxide,
            name=f"screw_{index}",
        )

    front_plate = model.part("front_plate")
    front_plate.visual(
        Box((0.008, 0.145, 0.050)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
        material=machined_steel,
        name="plate",
    )
    front_plate.visual(
        Box((0.002, 0.140, 0.036)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=machined_steel,
        name="tooth_band",
    )
    for index, z_pos in enumerate((-0.015, 0.015)):
        front_plate.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(-0.0055, 0.0, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_oxide,
            name=f"screw_{index}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="hub",
    )
    handle.visual(
        Cylinder(radius=0.0052, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="bar",
    )
    for index, y_pos in enumerate((-0.089, 0.089)):
        handle.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_oxide,
            name=f"knob_{index}",
        )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.0,
            lower=-math.radians(55.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=sliding_jaw,
        origin=Origin(xyz=(JAW_CLOSED_X, 0.0, JAW_FACE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.08,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=sliding_jaw,
        child=handle,
        origin=Origin(xyz=(0.086, 0.0, -0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )
    model.articulation(
        "body_to_rear_plate",
        ArticulationType.FIXED,
        parent=body,
        child=rear_plate,
        origin=Origin(xyz=(0.0, 0.0, JAW_FACE_Z)),
    )
    model.articulation(
        "jaw_to_front_plate",
        ArticulationType.FIXED,
        parent=sliding_jaw,
        child=front_plate,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    front_plate = object_model.get_part("front_plate")
    rear_plate = object_model.get_part("rear_plate")
    handle = object_model.get_part("handle")

    base_swivel = object_model.get_articulation("base_swivel")
    jaw_slide = object_model.get_articulation("jaw_slide")
    handle_spin = object_model.get_articulation("handle_spin")

    ctx.allow_overlap(
        handle,
        "sliding_jaw",
        reason="The T-handle hub is intentionally simplified as rotating inside the sliding jaw screw boss.",
    )

    def _span(aabb, axis: int) -> float | None:
        if aabb is None:
            return None
        lower, upper = aabb
        return upper[axis] - lower[axis]

    with ctx.pose({jaw_slide: 0.0}):
        ctx.expect_gap(
            front_plate,
            rear_plate,
            axis="x",
            min_gap=0.003,
            max_gap=0.005,
            name="jaw plates begin nearly closed",
        )
        ctx.expect_overlap(
            front_plate,
            rear_plate,
            axes="yz",
            min_overlap=0.040,
            name="jaw plates stay aligned for clamping",
        )
        ctx.expect_gap(
            body,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="swivel table rests on the base ring",
        )

    front_plate_rest = ctx.part_world_position(front_plate)
    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        ctx.expect_gap(
            front_plate,
            rear_plate,
            axis="x",
            min_gap=0.123,
            max_gap=0.125,
            name="sliding jaw opens to a realistic bench-vise span",
        )
        ctx.expect_overlap(
            front_plate,
            rear_plate,
            axes="yz",
            min_overlap=0.040,
            name="jaw plates remain square when opened",
        )
        front_plate_open = ctx.part_world_position(front_plate)

    ctx.check(
        "sliding jaw moves forward on the guide",
        front_plate_rest is not None
        and front_plate_open is not None
        and front_plate_open[0] > front_plate_rest[0] + 0.10,
        details=f"rest={front_plate_rest}, open={front_plate_open}",
    )

    handle_rest = ctx.part_world_position(handle)
    with ctx.pose({base_swivel: math.radians(35.0)}):
        handle_swiveled = ctx.part_world_position(handle)
    ctx.check(
        "body yaws on the swivel base",
        handle_rest is not None
        and handle_swiveled is not None
        and abs(handle_swiveled[1] - handle_rest[1]) > 0.05,
        details=f"rest={handle_rest}, swiveled={handle_swiveled}",
    )

    handle_rest_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_spin: math.pi / 2.0}):
        handle_turned_aabb = ctx.part_world_aabb(handle)
    rest_y = _span(handle_rest_aabb, 1)
    turn_y = _span(handle_turned_aabb, 1)
    rest_z = _span(handle_rest_aabb, 2)
    turn_z = _span(handle_turned_aabb, 2)
    ctx.check(
        "front handle rotates about the screw axis",
        rest_y is not None
        and turn_y is not None
        and rest_z is not None
        and turn_z is not None
        and rest_y > turn_y + 0.10
        and turn_z > rest_z + 0.10,
        details=(
            f"rest_y={rest_y}, turn_y={turn_y}, "
            f"rest_z={rest_z}, turn_z={turn_z}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
