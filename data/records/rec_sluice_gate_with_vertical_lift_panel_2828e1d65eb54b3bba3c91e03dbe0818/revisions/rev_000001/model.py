from __future__ import annotations

import math

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
    TorusGeometry,
    mesh_from_geometry,
)


STEEL = Material("weathered_galvanized_steel", rgba=(0.47, 0.51, 0.50, 1.0))
RAIL_DARK = Material("dark_wet_steel", rgba=(0.20, 0.23, 0.23, 1.0))
PANEL_BLUE = Material("painted_lift_panel", rgba=(0.16, 0.29, 0.38, 1.0))
RUBBER = Material("black_rubber_grip", rgba=(0.03, 0.03, 0.025, 1.0))
BRASS = Material("worn_brass_tags", rgba=(0.75, 0.58, 0.25, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_irrigation_sluice_gate")

    frame = model.part("frame")

    # Fixed steel sill and paired side guide rails.  The front/rear lips make
    # each guide read as a simple channel while leaving a clear slot for the
    # sliding plate.
    frame.visual(
        Box((0.82, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=RAIL_DARK,
        name="bottom_sill",
    )
    frame.visual(
        Box((0.065, 0.18, 0.90)),
        origin=Origin(xyz=(-0.335, 0.0, 0.49)),
        material=STEEL,
        name="left_web",
    )
    frame.visual(
        Box((0.035, 0.035, 0.86)),
        origin=Origin(xyz=(-0.282, -0.095, 0.50)),
        material=STEEL,
        name="left_front_lip",
    )
    frame.visual(
        Box((0.035, 0.035, 0.86)),
        origin=Origin(xyz=(-0.282, 0.095, 0.50)),
        material=STEEL,
        name="left_rear_lip",
    )
    frame.visual(
        Box((0.12, 0.035, 0.045)),
        origin=Origin(xyz=(-0.335, -0.095, 0.095)),
        material=RAIL_DARK,
        name="left_foot_clamp",
    )
    frame.visual(
        Box((0.065, 0.18, 0.90)),
        origin=Origin(xyz=(0.335, 0.0, 0.49)),
        material=STEEL,
        name="right_web",
    )
    frame.visual(
        Box((0.035, 0.035, 0.86)),
        origin=Origin(xyz=(0.282, -0.095, 0.50)),
        material=STEEL,
        name="right_front_lip",
    )
    frame.visual(
        Box((0.035, 0.035, 0.86)),
        origin=Origin(xyz=(0.282, 0.095, 0.50)),
        material=STEEL,
        name="right_rear_lip",
    )
    frame.visual(
        Box((0.12, 0.035, 0.045)),
        origin=Origin(xyz=(0.335, -0.095, 0.095)),
        material=RAIL_DARK,
        name="right_foot_clamp",
    )

    # Top crossmembers are split front/back so the lift panel can rise through
    # the middle slot.  A front bracket plate carries the crank wheel and latch.
    for name, y in (("front_crossbar", -0.095), ("rear_crossbar", 0.095)):
        frame.visual(
            Box((0.82, 0.035, 0.08)),
            origin=Origin(xyz=(0.0, y, 0.96)),
            material=STEEL,
            name=name,
        )
    frame.visual(
        Box((0.24, 0.055, 0.25)),
        origin=Origin(xyz=(0.0, -0.105, 1.105)),
        material=STEEL,
        name="top_bracket_plate",
    )
    frame.visual(
        Box((0.10, 0.035, 0.18)),
        origin=Origin(xyz=(-0.12, -0.095, 1.055), rpy=(0.0, math.radians(18.0), 0.0)),
        material=STEEL,
        name="left_gusset",
    )
    frame.visual(
        Box((0.10, 0.035, 0.18)),
        origin=Origin(xyz=(0.12, -0.095, 1.055), rpy=(0.0, math.radians(-18.0), 0.0)),
        material=STEEL,
        name="right_gusset",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.025),
        origin=Origin(xyz=(0.0, -0.1425, 1.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RAIL_DARK,
        name="crank_bearing",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.025),
        origin=Origin(xyz=(0.18, -0.125, 0.91), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RAIL_DARK,
        name="latch_boss",
    )

    panel = model.part("lift_panel")
    panel.visual(
        Box((0.56, 0.035, 0.68)),
        origin=Origin(),
        material=PANEL_BLUE,
        name="panel_plate",
    )
    panel.visual(
        Box((0.49, 0.014, 0.035)),
        origin=Origin(xyz=(0.0, -0.02425, 0.18)),
        material=STEEL,
        name="lower_stiffener",
    )
    panel.visual(
        Box((0.49, 0.014, 0.035)),
        origin=Origin(xyz=(0.0, -0.02425, -0.16)),
        material=STEEL,
        name="upper_stiffener",
    )
    panel.visual(
        Box((0.045, 0.014, 0.58)),
        origin=Origin(xyz=(0.0, -0.02425, 0.0)),
        material=STEEL,
        name="center_stiffener",
    )
    panel.visual(
        Box((0.055, 0.010, 0.54)),
        origin=Origin(xyz=(0.18, -0.02225, 0.0)),
        material=RAIL_DARK,
        name="rack_strip",
    )
    for i, z in enumerate((-0.23, -0.13, -0.03, 0.07, 0.17, 0.27)):
        panel.visual(
            Box((0.070, 0.004, 0.018)),
            origin=Origin(xyz=(0.18, -0.02925, z)),
            material=BRASS,
            name=f"height_mark_{i}",
        )

    crank_wheel = model.part("crank_wheel")
    crank_wheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.140, tube=0.010), "crank_wheel_rim"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RAIL_DARK,
        name="rim",
    )
    crank_wheel.visual(
        Cylinder(radius=0.034, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RAIL_DARK,
        name="hub",
    )
    for i in range(6):
        crank_wheel.visual(
            Box((0.270, 0.016, 0.016)),
            origin=Origin(rpy=(0.0, i * math.pi / 6.0, 0.0)),
            material=STEEL,
            name=f"spoke_{i}",
        )
    crank_wheel.visual(
        Cylinder(radius=0.010, length=0.045),
        origin=Origin(xyz=(0.10, -0.025, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="crank_pin",
    )
    crank_wheel.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(0.10, -0.080, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RUBBER,
        name="hand_grip",
    )

    latch_arm = model.part("latch_arm")
    latch_arm.visual(
        Cylinder(radius=0.026, length=0.025),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RAIL_DARK,
        name="pivot_sleeve",
    )
    latch_arm.visual(
        Box((0.026, 0.014, 0.245)),
        origin=Origin(xyz=(0.0, 0.0, -0.1225)),
        material=STEEL,
        name="drop_arm",
    )
    latch_arm.visual(
        Cylinder(radius=0.012, length=0.117),
        origin=Origin(xyz=(0.0, 0.0585, -0.225), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RAIL_DARK,
        name="locking_pin",
    )

    model.articulation(
        "frame_to_lift_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.22, effort=500.0, velocity=0.15),
    )
    model.articulation(
        "frame_to_crank_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank_wheel,
        origin=Origin(xyz=(0.0, -0.18, 1.12)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0),
    )
    model.articulation(
        "frame_to_latch_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=latch_arm,
        origin=Origin(xyz=(0.18, -0.15, 0.91)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.20, upper=0.70, effort=20.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("lift_panel")
    crank_wheel = object_model.get_part("crank_wheel")
    latch_arm = object_model.get_part("latch_arm")
    panel_slide = object_model.get_articulation("frame_to_lift_panel")
    crank_spin = object_model.get_articulation("frame_to_crank_wheel")
    latch_pivot = object_model.get_articulation("frame_to_latch_arm")

    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="panel_plate",
        negative_elem="bottom_sill",
        max_gap=0.002,
        max_penetration=0.001,
        name="closed panel rests on sill",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="x",
        positive_elem="right_web",
        negative_elem="panel_plate",
        min_gap=0.015,
        max_gap=0.040,
        name="right guide rail clears panel edge",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="x",
        positive_elem="panel_plate",
        negative_elem="left_web",
        min_gap=0.015,
        max_gap=0.040,
        name="left guide rail clears panel edge",
    )
    ctx.expect_gap(
        frame,
        crank_wheel,
        axis="y",
        positive_elem="crank_bearing",
        negative_elem="hub",
        max_gap=0.002,
        max_penetration=0.001,
        name="crank hub seats against bearing",
    )
    ctx.expect_gap(
        panel,
        latch_arm,
        axis="y",
        positive_elem="rack_strip",
        negative_elem="locking_pin",
        min_gap=0.0,
        max_gap=0.006,
        name="latch pin reaches panel rack",
    )
    ctx.expect_overlap(
        latch_arm,
        panel,
        axes="xz",
        elem_a="locking_pin",
        elem_b="rack_strip",
        min_overlap=0.010,
        name="latch pin aligns with height rack",
    )

    rest_panel_pos = ctx.part_world_position(panel)
    with ctx.pose({panel_slide: 0.22}):
        raised_panel_pos = ctx.part_world_position(panel)
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="panel_plate",
            elem_b="left_web",
            min_overlap=0.30,
            name="raised panel remains captured in rails",
        )
    ctx.check(
        "lift panel slides upward",
        rest_panel_pos is not None
        and raised_panel_pos is not None
        and raised_panel_pos[2] > rest_panel_pos[2] + 0.20,
        details=f"rest={rest_panel_pos}, raised={raised_panel_pos}",
    )

    rest_wheel_pos = ctx.part_world_position(crank_wheel)
    with ctx.pose({crank_spin: math.pi * 1.5}):
        spun_wheel_pos = ctx.part_world_position(crank_wheel)
    ctx.check(
        "crank wheel spins in place",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and max(abs(a - b) for a, b in zip(rest_wheel_pos, spun_wheel_pos)) < 1e-6,
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    rest_pin_aabb = ctx.part_element_world_aabb(latch_arm, elem="locking_pin")
    with ctx.pose({latch_pivot: 0.70}):
        swung_pin_aabb = ctx.part_element_world_aabb(latch_arm, elem="locking_pin")
    if rest_pin_aabb is not None and swung_pin_aabb is not None:
        rest_pin_x = (rest_pin_aabb[0][0] + rest_pin_aabb[1][0]) * 0.5
        swung_pin_x = (swung_pin_aabb[0][0] + swung_pin_aabb[1][0]) * 0.5
        latch_ok = swung_pin_x < rest_pin_x - 0.08
    else:
        latch_ok = False
    ctx.check(
        "latch arm pivots clear of rack",
        latch_ok,
        details=f"rest_pin_aabb={rest_pin_aabb}, swung_pin_aabb={swung_pin_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
