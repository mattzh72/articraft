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


WHEEL_RADIUS = 0.68
WHEEL_WIDTH = 0.18
SHAFT_Z = 0.80
PLATE_TRAVEL = 0.14


def _annular_disk(radius_outer: float, radius_inner: float, width: float) -> cq.Workplane:
    """Annular cylinder centered on the origin, with its axis along local Y."""
    return (
        cq.Workplane("XY")
        .circle(radius_outer)
        .circle(radius_inner)
        .extrude(width, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90)
    )


def _rotated_box(
    sx: float,
    sy: float,
    sz: float,
    center: tuple[float, float, float],
    angle_deg: float,
) -> cq.Workplane:
    """Box first placed at +X, then rotated around the waterwheel shaft axis."""
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz)
        .translate(center)
        .rotate((0, 0, 0), (0, 1, 0), angle_deg)
    )


def _make_wheel_mesh() -> cq.Workplane:
    """Narrow overshot wheel: twin side rims, spokes, and many rim buckets."""
    side_y = WHEEL_WIDTH * 0.5 - 0.018
    wheel = _annular_disk(WHEEL_RADIUS, WHEEL_RADIUS - 0.045, 0.028).translate(
        (0, -side_y, 0)
    )
    wheel = wheel.union(
        _annular_disk(WHEEL_RADIUS, WHEEL_RADIUS - 0.045, 0.028).translate(
            (0, side_y, 0)
        )
    )

    spoke_len = WHEEL_RADIUS - 0.08
    spoke_mid = 0.08 + spoke_len * 0.5
    for i in range(12):
        angle = i * 30.0
        wheel = wheel.union(_rotated_box(spoke_len, 0.025, 0.030, (spoke_mid, -side_y, 0), angle))
        wheel = wheel.union(_rotated_box(spoke_len, 0.025, 0.030, (spoke_mid, side_y, 0), angle))

    # Cross-wheel buckets are shallow trough-like slats set around the outside.
    # They intentionally overlap the wooden rims to read as bolted-on paddles.
    for i in range(18):
        angle = i * 20.0 + 4.0
        bucket = _rotated_box(0.050, WHEEL_WIDTH, 0.145, (WHEEL_RADIUS - 0.024, 0, 0), angle)
        wheel = wheel.union(bucket)

    return wheel


def _make_bearing_block(y_center: float) -> cq.Workplane:
    """Pillow-block bearing housing with a clear cylindrical bore for the shaft."""
    block = cq.Workplane("XY").box(0.18, 0.055, 0.15).translate((0, y_center, SHAFT_Z))
    bore = (
        cq.Workplane("XZ")
        .circle(0.048)
        .extrude(0.16, both=True)
        .translate((0, y_center, SHAFT_Z))
    )
    return block.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_overshot_waterwheel")

    wood = model.material("warm_oiled_wood", rgba=(0.55, 0.32, 0.15, 1.0))
    dark_wood = model.material("dark_wet_wood", rgba=(0.28, 0.16, 0.08, 1.0))
    iron = model.material("dark_burnished_iron", rgba=(0.12, 0.12, 0.12, 1.0))
    guide_iron = model.material("worn_guide_iron", rgba=(0.22, 0.22, 0.20, 1.0))
    shadow = model.material("deep_outlet_shadow", rgba=(0.03, 0.025, 0.02, 1.0))

    frame = model.part("frame")

    # Timber base and side frame, with two connected side rails and cross ties.
    for y, name in ((-0.16, "rail_0"), (0.16, "rail_1")):
        frame.visual(
            Box((1.35, 0.060, 0.080)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material=wood,
            name=name,
        )
        frame.visual(
            Box((0.12, 0.060, 0.70)),
            origin=Origin(xyz=(0.0, y, 0.390)),
            material=wood,
            name=f"bearing_post_{0 if y < 0 else 1}",
        )
        frame.visual(
            Box((0.065, 0.065, 1.80)),
            origin=Origin(xyz=(-0.56, y, 0.900)),
            material=wood,
            name=f"tall_post_{0 if y < 0 else 1}",
        )
        frame.visual(
            Box((0.58, 0.055, 0.065)),
            origin=Origin(xyz=(-0.30, y, 1.875)),
            material=wood,
            name=f"top_side_beam_{0 if y < 0 else 1}",
        )

    for x, name in ((-0.58, "base_tie_0"), (0.58, "base_tie_1")):
        frame.visual(
            Box((0.08, 0.42, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=wood,
            name=name,
        )

    for x, name in ((-0.56, "top_tie_0"), (-0.04, "top_tie_1")):
        frame.visual(
            Box((0.075, 0.42, 0.070)),
            origin=Origin(xyz=(x, 0.0, 1.875)),
            material=wood,
            name=name,
        )

    frame.visual(
        mesh_from_cadquery(_make_bearing_block(-0.16), "bearing_0"),
        material=iron,
        name="bearing_0",
    )
    frame.visual(
        mesh_from_cadquery(_make_bearing_block(0.16), "bearing_1"),
        material=iron,
        name="bearing_1",
    )
    frame.visual(
        Cylinder(radius=0.052, length=0.080),
        origin=Origin(xyz=(0.0, -0.16, SHAFT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="bearing_bushing_0",
    )
    frame.visual(
        Cylinder(radius=0.052, length=0.080),
        origin=Origin(xyz=(0.0, 0.16, SHAFT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="bearing_bushing_1",
    )

    # Suspended open-top inlet box above the overshot rim.
    inlet_x = -0.23
    inlet_z = 1.65
    frame.visual(
        Box((0.50, 0.25, 0.026)),
        origin=Origin(xyz=(-0.30, 0.0, 1.553)),
        material=dark_wood,
        name="inlet_floor_rear",
    )
    frame.visual(
        Box((0.050, 0.25, 0.026)),
        origin=Origin(xyz=(0.065, 0.0, 1.553)),
        material=dark_wood,
        name="outlet_lip",
    )
    for y, name in ((-0.14, "inlet_side_0"), (0.14, "inlet_side_1")):
        frame.visual(
            Box((0.64, 0.030, 0.22)),
            origin=Origin(xyz=(inlet_x, y, inlet_z)),
            material=dark_wood,
            name=name,
        )
    for x, name in ((-0.55, "inlet_end_0"), (0.09, "inlet_end_1")):
        frame.visual(
            Box((0.030, 0.28, 0.22)),
            origin=Origin(xyz=(x, 0.0, inlet_z)),
            material=dark_wood,
            name=name,
        )
    frame.visual(
        Box((0.11, 0.18, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, 1.546)),
        material=shadow,
        name="outlet_slot",
    )

    # Short iron guide rails under the outlet slot for the sliding regulator.
    frame.visual(
        Box((0.40, 0.024, 0.030)),
        origin=Origin(xyz=(0.005, -0.092, 1.535)),
        material=guide_iron,
        name="regulator_guide_0",
    )
    frame.visual(
        Box((0.40, 0.024, 0.030)),
        origin=Origin(xyz=(0.005, 0.092, 1.535)),
        material=guide_iron,
        name="regulator_guide_1",
    )

    # Four hanger straps make the inlet box visibly suspended from the overhead beam.
    for x in (-0.49, 0.02):
        for y in (-0.14, 0.14):
            frame.visual(
                Box((0.026, 0.026, 0.120)),
                origin=Origin(xyz=(x, y, 1.810)),
                material=iron,
                name=f"hanger_{len(frame.visuals)}",
            )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_make_wheel_mesh(), "wheel_structure", tolerance=0.002),
        material=wood,
        name="wheel_structure",
    )
    wheel.visual(
        Cylinder(radius=0.032, length=0.46),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="shaft",
    )
    wheel.visual(
        Cylinder(radius=0.090, length=0.22),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub",
    )

    regulator = model.part("regulator")
    regulator.visual(
        Box((0.24, 0.17, 0.016)),
        origin=Origin(),
        material=guide_iron,
        name="plate_leaf",
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0),
    )
    model.articulation(
        "regulator_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=regulator,
        origin=Origin(xyz=(-0.035, 0.0, 1.516)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.25, lower=0.0, upper=PLATE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    regulator = object_model.get_part("regulator")
    spin = object_model.get_articulation("wheel_spin")
    slide = object_model.get_articulation("regulator_slide")

    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="bearing_bushing_0",
        elem_b="shaft",
        reason="The shaft is intentionally captured inside the simplified first bearing bushing.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="bearing_bushing_1",
        elem_b="shaft",
        reason="The shaft is intentionally captured inside the simplified second bearing bushing.",
    )

    ctx.check(
        "wheel has continuous horizontal shaft joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "regulator has short prismatic travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and slide.motion_limits.upper == PLATE_TRAVEL,
        details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
    )

    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="outlet_lip",
        negative_elem="wheel_structure",
        min_gap=0.010,
        name="inlet clears top of wheel rim",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="shaft",
        elem_b="bearing_bushing_0",
        min_overlap=0.025,
        name="shaft passes through first side bearing",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="shaft",
        outer_elem="bearing_bushing_0",
        margin=0.002,
        name="shaft is centered in first bearing",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="shaft",
        elem_b="bearing_bushing_1",
        min_overlap=0.025,
        name="shaft passes through second side bearing",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="shaft",
        outer_elem="bearing_bushing_1",
        margin=0.002,
        name="shaft is centered in second bearing",
    )
    ctx.expect_overlap(
        regulator,
        frame,
        axes="x",
        elem_a="plate_leaf",
        elem_b="regulator_guide_0",
        min_overlap=0.18,
        name="closed regulator remains in first guide",
    )
    with ctx.pose({slide: PLATE_TRAVEL}):
        ctx.expect_overlap(
            regulator,
            frame,
            axes="x",
            elem_a="plate_leaf",
            elem_b="regulator_guide_1",
            min_overlap=0.14,
            name="open regulator remains in second guide",
        )

    rest_pos = ctx.part_world_position(regulator)
    with ctx.pose({slide: PLATE_TRAVEL}):
        open_pos = ctx.part_world_position(regulator)
    ctx.check(
        "regulator slides downstream",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 0.10,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
