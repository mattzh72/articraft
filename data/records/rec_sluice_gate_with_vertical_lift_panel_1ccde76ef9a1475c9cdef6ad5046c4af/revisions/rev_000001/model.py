from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _xz_annulus(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """Annular plate centered on the local Y axle, authored in the XZ plane."""
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness / 2.0, both=True)
    )


def _xz_rect(cx: float, cz: float, sx: float, sz: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(cx, cz)
        .rect(sx, sz)
        .extrude(thickness / 2.0, both=True)
    )


def _build_handwheel() -> cq.Workplane:
    ring = _xz_annulus(0.160, 0.128, 0.040)
    hub = _xz_annulus(0.050, 0.024, 0.048)
    horizontal_spoke = _xz_rect(0.0, 0.0, 0.260, 0.022, 0.034)
    vertical_spoke = _xz_rect(0.0, 0.0, 0.022, 0.260, 0.034)
    bore = cq.Workplane("XZ").circle(0.013).extrude(0.070, both=True)
    return ring.union(hub).union(horizontal_spoke).union(vertical_spoke).cut(bore)


def _build_pawl() -> cq.Workplane:
    hub = _xz_annulus(0.034, 0.019, 0.034)
    lever = _xz_rect(-0.112, 0.0, 0.205, 0.025, 0.030)
    tooth = (
        cq.Workplane("XZ")
        .polyline([(-0.215, -0.012), (-0.168, -0.012), (-0.190, -0.055)])
        .close()
        .extrude(0.015, both=True)
    )
    pin_bore = cq.Workplane("XZ").circle(0.012).extrude(0.050, both=True)
    return hub.union(lever).union(tooth).cut(pin_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sluice_mechanism")

    galvanized = Material("galvanized_steel", color=(0.48, 0.52, 0.54, 1.0))
    dark_steel = Material("dark_burnished_steel", color=(0.08, 0.09, 0.10, 1.0))
    gate_blue = Material("painted_gate_plate", color=(0.05, 0.22, 0.40, 1.0))
    brass = Material("worn_brass_pawl", color=(0.63, 0.46, 0.20, 1.0))
    rubber = Material("black_grip", color=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("frame")
    # Rigid portal frame: two posts, split top crossmember with a center slot,
    # bottom sill, close-fitting side guide channels, and the top actuator bracket.
    frame.visual(
        Box((0.09, 0.14, 1.05)),
        origin=Origin(xyz=(-0.45, 0.0, 0.525)),
        material=galvanized,
        name="post_0",
    )
    frame.visual(
        Box((0.09, 0.14, 1.05)),
        origin=Origin(xyz=(0.45, 0.0, 0.525)),
        material=galvanized,
        name="post_1",
    )
    frame.visual(
        Box((1.02, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=galvanized,
        name="sill",
    )
    frame.visual(
        Box((1.02, 0.045, 0.09)),
        origin=Origin(xyz=(0.0, -0.055, 1.04)),
        material=galvanized,
        name="front_top_beam",
    )
    frame.visual(
        Box((1.02, 0.045, 0.09)),
        origin=Origin(xyz=(0.0, 0.055, 1.04)),
        material=galvanized,
        name="rear_top_beam",
    )
    frame.visual(
        Box((0.04, 0.08, 0.84)),
        origin=Origin(xyz=(-0.385, 0.0, 0.50)),
        material=galvanized,
        name="left_guide_web",
    )
    frame.visual(
        Box((0.095, 0.020, 0.84)),
        origin=Origin(xyz=(-0.371525, -0.040, 0.50)),
        material=galvanized,
        name="left_back_lip",
    )
    frame.visual(
        Box((0.095, 0.020, 0.84)),
        origin=Origin(xyz=(-0.371525, 0.040, 0.50)),
        material=galvanized,
        name="left_front_lip",
    )
    frame.visual(
        Box((0.04, 0.08, 0.84)),
        origin=Origin(xyz=(0.385, 0.0, 0.50)),
        material=galvanized,
        name="right_guide_web",
    )
    frame.visual(
        Box((0.095, 0.020, 0.84)),
        origin=Origin(xyz=(0.371525, -0.040, 0.50)),
        material=galvanized,
        name="right_back_lip",
    )
    frame.visual(
        Box((0.095, 0.020, 0.84)),
        origin=Origin(xyz=(0.371525, 0.040, 0.50)),
        material=galvanized,
        name="right_front_lip",
    )

    frame.visual(
        Box((0.070, 0.105, 0.080)),
        origin=Origin(xyz=(-0.115, 0.0, 1.11)),
        material=galvanized,
        name="bracket_foot_0",
    )
    frame.visual(
        Box((0.070, 0.105, 0.080)),
        origin=Origin(xyz=(-0.245, 0.0, 1.11)),
        material=galvanized,
        name="bracket_foot_1",
    )
    frame.visual(
        Box((0.315, 0.025, 0.250)),
        origin=Origin(xyz=(-0.180, -0.020, 1.22)),
        material=galvanized,
        name="actuator_plate",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.160),
        origin=Origin(xyz=(-0.170, -0.110, 1.22), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_axle",
    )
    frame.visual(
        Box((0.160, 0.035, 0.050)),
        origin=Origin(xyz=(0.000, -0.100, 1.44)),
        material=galvanized,
        name="pawl_support",
    )
    frame.visual(
        Box((0.040, 0.090, 0.040)),
        origin=Origin(xyz=(-0.060, -0.060, 1.44)),
        material=galvanized,
        name="pawl_bridge",
    )
    frame.visual(
        Box((0.045, 0.100, 0.120)),
        origin=Origin(xyz=(0.000, -0.065, 1.385)),
        material=galvanized,
        name="pawl_upright",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.060, -0.135, 1.44), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pawl_pin",
    )

    gate = model.part("gate")
    gate.visual(
        Box((0.680, 0.035, 0.580)),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=gate_blue,
        name="plate",
    )
    gate.visual(
        Box((0.045, 0.025, 0.370)),
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        material=dark_steel,
        name="lift_stem",
    )
    gate.visual(
        Box((0.210, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.596)),
        material=gate_blue,
        name="top_clamp",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_build_handwheel(), "handwheel"),
        material=dark_steel,
        name="rim_spokes",
    )
    for i, (x, z) in enumerate(((0.0, 0.160), (0.160, 0.0), (0.0, -0.160), (-0.160, 0.0))):
        wheel.visual(
            Cylinder(radius=0.012, length=0.030),
            origin=Origin(xyz=(x, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{i}",
        )

    pawl = model.part("pawl")
    pawl.visual(
        mesh_from_cadquery(_build_pawl(), "locking_pawl"),
        material=brass,
        name="lever_tooth",
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.20, lower=0.0, upper=0.32),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(-0.170, -0.160, 1.22)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0),
    )
    model.articulation(
        "pawl_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=pawl,
        origin=Origin(xyz=(0.060, -0.160, 1.44)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.35, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate")
    wheel = object_model.get_part("wheel")
    pawl = object_model.get_part("pawl")
    gate_slide = object_model.get_articulation("gate_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")
    pawl_pivot = object_model.get_articulation("pawl_pivot")

    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="wheel_axle",
        elem_b="rim_spokes",
        reason="The wheel hub is represented with a captured axle seated through its central bearing.",
    )
    ctx.allow_overlap(
        frame,
        pawl,
        elem_a="pawl_pin",
        elem_b="lever_tooth",
        reason="The locking pawl pivots around a captured pin represented inside the pawl hub.",
    )

    ctx.check(
        "gate uses vertical prismatic slide",
        gate_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(gate_slide.axis) == (0.0, 0.0, 1.0),
    )
    ctx.check(
        "wheel is continuous on its axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
    )
    ctx.check(
        "pawl has limited revolute travel",
        pawl_pivot.articulation_type == ArticulationType.REVOLUTE
        and pawl_pivot.motion_limits is not None
        and pawl_pivot.motion_limits.lower < 0.0
        and pawl_pivot.motion_limits.upper > 0.0,
    )

    ctx.expect_gap(
        frame,
        gate,
        axis="x",
        positive_elem="right_guide_web",
        negative_elem="plate",
        min_gap=0.015,
        max_gap=0.050,
        name="right guide clears gate edge",
    )
    ctx.expect_gap(
        gate,
        frame,
        axis="x",
        positive_elem="plate",
        negative_elem="left_guide_web",
        min_gap=0.015,
        max_gap=0.050,
        name="left guide clears gate edge",
    )
    ctx.expect_gap(
        frame,
        gate,
        axis="y",
        positive_elem="right_front_lip",
        negative_elem="plate",
        min_gap=0.005,
        max_gap=0.030,
        name="front guide lip clears plate face",
    )
    ctx.expect_gap(
        gate,
        frame,
        axis="y",
        positive_elem="plate",
        negative_elem="right_back_lip",
        min_gap=0.005,
        max_gap=0.030,
        name="back guide lip clears plate face",
    )
    ctx.expect_gap(
        gate,
        frame,
        axis="z",
        positive_elem="plate",
        negative_elem="sill",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed gate rests on sill",
    )
    ctx.expect_overlap(
        gate,
        frame,
        axes="z",
        elem_a="plate",
        elem_b="right_guide_web",
        min_overlap=0.40,
        name="closed gate remains in vertical guides",
    )

    closed_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: 0.32}):
        ctx.expect_overlap(
            gate,
            frame,
            axes="z",
            elem_a="plate",
            elem_b="right_guide_web",
            min_overlap=0.30,
            name="raised gate retains guide engagement",
        )
        raised_pos = ctx.part_world_position(gate)
    ctx.check(
        "gate raises along z",
        closed_pos is not None and raised_pos is not None and raised_pos[2] > closed_pos[2] + 0.30,
        details=f"closed={closed_pos}, raised={raised_pos}",
    )

    ctx.expect_within(
        frame,
        wheel,
        axes="xz",
        inner_elem="wheel_axle",
        outer_elem="rim_spokes",
        margin=0.001,
        name="axle is captured inside wheel hub",
    )
    ctx.expect_overlap(
        frame,
        wheel,
        axes="y",
        elem_a="wheel_axle",
        elem_b="rim_spokes",
        min_overlap=0.040,
        name="wheel hub surrounds axle through its thickness",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="yz",
        inner_elem="rim_spokes",
        outer_elem="wheel_axle",
        margin=0.17,
        name="wheel is centered on axle line",
    )
    ctx.expect_within(
        frame,
        pawl,
        axes="xz",
        inner_elem="pawl_pin",
        outer_elem="lever_tooth",
        margin=0.001,
        name="pawl pin is captured in pawl hub",
    )
    ctx.expect_overlap(
        frame,
        pawl,
        axes="y",
        elem_a="pawl_pin",
        elem_b="lever_tooth",
        min_overlap=0.030,
        name="pawl hub surrounds pivot pin",
    )

    resting_pawl = ctx.part_element_world_aabb(pawl, elem="lever_tooth")
    with ctx.pose({pawl_pivot: 0.35}):
        swung_pawl = ctx.part_element_world_aabb(pawl, elem="lever_tooth")
    ctx.check(
        "pawl visibly pivots beside wheel",
        resting_pawl is not None
        and swung_pawl is not None
        and abs(swung_pawl[1][2] - resting_pawl[1][2]) > 0.040,
        details=f"rest={resting_pawl}, swung={swung_pawl}",
    )

    return ctx.report()


object_model = build_object_model()
