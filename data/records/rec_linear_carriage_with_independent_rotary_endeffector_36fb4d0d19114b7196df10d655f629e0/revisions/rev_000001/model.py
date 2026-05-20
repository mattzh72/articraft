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
)


def _x_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    """Origin for an SDK cylinder whose local Z axis should lie along +X."""
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _build_spindle_sleeve():
    """A short annular bearing housing with a real clearance bore."""
    return (
        cq.Workplane("XY")
        .circle(0.060)
        .circle(0.032)
        .extrude(0.130)
        .translate((0.0, 0.0, -0.065))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_rotary_shuttle")

    dark_casting = model.material("dark_casting", rgba=(0.16, 0.17, 0.18, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    ground_rail = model.material("ground_rail", rgba=(0.78, 0.79, 0.77, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.10, 0.19, 0.34, 1.0))
    spindle_black = model.material("spindle_black", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    warning_mark = model.material("warning_mark", rgba=(0.95, 0.70, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.900, 0.340, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_casting,
        name="base_plate",
    )
    for y, name in ((-0.070, "guide_rail_0"), (0.070, "guide_rail_1")):
        base.visual(
            Box((0.740, 0.032, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0625)),
            material=ground_rail,
            name=name,
        )
    for y, name in ((-0.153, "side_lip_0"), (0.153, "side_lip_1")):
        base.visual(
            Box((0.860, 0.024, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.065)),
            material=dark_casting,
            name=name,
        )
    for x, name in ((-0.405, "end_stop_0"), (0.405, "end_stop_1")):
        base.visual(
            Box((0.030, 0.250, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material=dark_casting,
            name=name,
        )
    for x in (-0.300, 0.300):
        for y in (-0.125, 0.125):
            base.visual(
                Cylinder(radius=0.016, length=0.004),
                origin=Origin(xyz=(x, y, 0.047)),
                material=rubber,
                name=f"recessed_bolt_{x:+.1f}_{y:+.1f}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.340, 0.220, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=carriage_blue,
        name="carriage_deck",
    )
    for x in (-0.110, 0.110):
        for y, rail_index in ((-0.070, 0), (0.070, 1)):
            pad_name = f"bearing_pad_{0 if x < 0 else 1}_{rail_index}"
            carriage.visual(
                Box((0.095, 0.038, 0.020)),
                origin=Origin(xyz=(x, y, 0.010)),
                material=machined_steel,
                name=pad_name,
            )
    carriage.visual(
        Box((0.270, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=machined_steel,
        name="center_way_cover",
    )
    sleeve_mesh = mesh_from_cadquery(
        _build_spindle_sleeve(),
        "spindle_sleeve",
        tolerance=0.0007,
        angular_tolerance=0.08,
    )
    carriage.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.105, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_black,
        name="spindle_sleeve",
    )
    carriage.visual(
        Box((0.115, 0.025, 0.016)),
        origin=Origin(xyz=(0.105, 0.0, 0.168)),
        material=spindle_black,
        name="top_cooling_rib",
    )
    for y, support_index in ((-0.055, 0), (0.055, 1)):
        carriage.visual(
            Box((0.018, 0.035, 0.090)),
            origin=Origin(xyz=(0.035, y, 0.100)),
            material=spindle_black,
            name=f"rear_side_support_{support_index}",
        )
    carriage.visual(
        Box((0.055, 0.170, 0.020)),
        origin=Origin(xyz=(0.105, 0.0, 0.056)),
        material=spindle_black,
        name="spindle_saddle",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.018, length=0.235),
        origin=_x_cylinder_origin((-0.020, 0.0, 0.0)),
        material=machined_steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=_x_cylinder_origin((0.006, 0.0, 0.0)),
        material=machined_steel,
        name="front_bearing_collar",
    )
    spindle.visual(
        Cylinder(radius=0.027, length=0.056),
        origin=_x_cylinder_origin((0.039, 0.0, 0.0)),
        material=machined_steel,
        name="spindle_nose",
    )
    spindle.visual(
        Cylinder(radius=0.022, length=0.048),
        origin=_x_cylinder_origin((0.091, 0.0, 0.0)),
        material=spindle_black,
        name="tool_collet",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=_x_cylinder_origin((0.150, 0.0, 0.0)),
        material=machined_steel,
        name="tool_stub",
    )
    spindle.visual(
        Box((0.052, 0.007, 0.007)),
        origin=Origin(xyz=(0.041, 0.0, 0.029)),
        material=warning_mark,
        name="rotation_mark",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.180, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.360),
    )
    model.articulation(
        "spindle_spin",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.170, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=25.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("carriage_slide")
    spin = object_model.get_articulation("spindle_spin")

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="bearing_pad_0_0",
        negative_elem="guide_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="front bearing pad sits on guide rail",
    )
    ctx.expect_gap(
        spindle,
        carriage,
        axis="x",
        positive_elem="front_bearing_collar",
        negative_elem="spindle_sleeve",
        max_gap=0.001,
        max_penetration=0.0,
        name="spindle collar seats against sleeve face",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.360}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="bearing_pad_1_1",
            elem_b="guide_rail_1",
            min_overlap=0.080,
            name="extended carriage remains captured on rail",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along the shuttle axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    mark_rest = ctx.part_element_world_aabb(spindle, elem="rotation_mark")
    with ctx.pose({spin: math.pi / 2.0}):
        mark_rotated = ctx.part_element_world_aabb(spindle, elem="rotation_mark")
    ctx.check(
        "spindle rotates its off-axis mark about X",
        mark_rest is not None
        and mark_rotated is not None
        and abs(mark_rotated[0][2] - mark_rest[0][2]) > 0.020,
        details=f"rest={mark_rest}, rotated={mark_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
