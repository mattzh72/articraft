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


def _handwheel_shape() -> cq.Workplane:
    """A connected annular handwheel with a clear central shaft bore."""

    thickness = 0.045
    outer_radius = 0.150
    inner_radius = 0.122
    hub_radius = 0.042
    bore_radius = 0.024

    rim = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    hub = (
        cq.Workplane("XY")
        .circle(hub_radius)
        .circle(bore_radius)
        .extrude(thickness * 1.18)
        .translate((0.0, 0.0, -thickness * 0.59))
    )
    wheel = rim.union(hub)

    spoke_inner = 0.032
    spoke_outer = inner_radius + 0.010
    spoke_len = spoke_outer - spoke_inner
    spoke_center = (spoke_outer + spoke_inner) / 2.0
    for angle in (0.0, 90.0, 180.0, 270.0):
        spoke = (
            cq.Workplane("XY")
            .box(spoke_len, 0.017, thickness * 0.72)
            .translate((spoke_center, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        wheel = wheel.union(spoke)

    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_sluice_gate")

    painted_steel = model.material("dark_blue_painted_steel", rgba=(0.05, 0.11, 0.16, 1.0))
    worn_steel = model.material("worn_galvanized_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    wet_plate = model.material("dark_wet_gate_plate", rgba=(0.09, 0.13, 0.14, 1.0))
    rubber = model.material("black_rubber_seal", rgba=(0.01, 0.01, 0.01, 1.0))
    yellow = model.material("safety_yellow_handwheel", rgba=(0.95, 0.66, 0.08, 1.0))
    dark = model.material("blackened_hardware", rgba=(0.03, 0.03, 0.03, 1.0))

    frame = model.part("frame")
    frame.visual(Box((0.08, 0.16, 1.22)), origin=Origin(xyz=(-0.30, 0.0, 0.68)), material=painted_steel, name="guide_rail_0")
    frame.visual(Box((0.08, 0.16, 1.22)), origin=Origin(xyz=(0.30, 0.0, 0.68)), material=painted_steel, name="guide_rail_1")
    frame.visual(Box((0.68, 0.16, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.05)), material=painted_steel, name="bottom_sill")
    frame.visual(Box((0.68, 0.16, 0.10)), origin=Origin(xyz=(0.0, 0.0, 1.28)), material=painted_steel, name="top_header")

    # Narrow front/back channel lips retain the gate without occupying the sliding slot.
    frame.visual(Box((0.036, 0.025, 1.12)), origin=Origin(xyz=(-0.248, -0.090, 0.68)), material=worn_steel, name="front_lip_0")
    frame.visual(Box((0.036, 0.025, 1.12)), origin=Origin(xyz=(-0.248, 0.090, 0.68)), material=worn_steel, name="rear_lip_0")
    frame.visual(Box((0.036, 0.025, 1.12)), origin=Origin(xyz=(0.248, -0.090, 0.68)), material=worn_steel, name="front_lip_1")
    frame.visual(Box((0.036, 0.025, 1.12)), origin=Origin(xyz=(0.248, 0.090, 0.68)), material=worn_steel, name="rear_lip_1")

    # Top yoke and actuator support.
    frame.visual(Box((0.075, 0.10, 0.38)), origin=Origin(xyz=(-0.23, -0.03, 1.50)), material=painted_steel, name="yoke_post_0")
    frame.visual(Box((0.075, 0.10, 0.38)), origin=Origin(xyz=(0.23, -0.03, 1.50)), material=painted_steel, name="yoke_post_1")
    frame.visual(Box((0.55, 0.10, 0.07)), origin=Origin(xyz=(0.0, -0.03, 1.68)), material=painted_steel, name="yoke_cap")
    frame.visual(Box((0.18, 0.060, 0.12)), origin=Origin(xyz=(0.0, -0.045, 1.49)), material=painted_steel, name="gear_housing")
    frame.visual(
        Cylinder(radius=0.014, length=0.80),
        origin=Origin(xyz=(0.10, -0.04, 1.54), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_steel,
        name="transverse_shaft",
    )
    frame.visual(
        Cylinder(radius=0.035, length=0.025),
        origin=Origin(xyz=(0.41095, -0.04, 1.54), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_steel,
        name="wheel_boss",
    )
    frame.visual(Box((0.12, 0.10, 0.035)), origin=Origin(xyz=(0.305, -0.11, 1.65)), material=painted_steel, name="lock_tab_mount")

    closure_panel = model.part("closure_panel")
    closure_panel.visual(Box((0.410, 0.035, 0.750)), origin=Origin(xyz=(0.0, -0.030, 0.375)), material=wet_plate, name="gate_plate")
    closure_panel.visual(Box((0.400, 0.055, 0.045)), origin=Origin(xyz=(0.0, -0.058, 0.125)), material=worn_steel, name="lower_stiffener")
    closure_panel.visual(Box((0.400, 0.055, 0.045)), origin=Origin(xyz=(0.0, -0.058, 0.395)), material=worn_steel, name="middle_stiffener")
    closure_panel.visual(Box((0.400, 0.055, 0.045)), origin=Origin(xyz=(0.0, -0.058, 0.720)), material=worn_steel, name="upper_stiffener")
    closure_panel.visual(Box((0.050, 0.055, 0.680)), origin=Origin(xyz=(0.0, -0.060, 0.365)), material=worn_steel, name="center_stiffener")
    closure_panel.visual(Box((0.035, 0.050, 0.725)), origin=Origin(xyz=(-0.210, -0.035, 0.365)), material=worn_steel, name="side_shoe_0")
    closure_panel.visual(Box((0.035, 0.050, 0.725)), origin=Origin(xyz=(0.210, -0.035, 0.365)), material=worn_steel, name="side_shoe_1")
    closure_panel.visual(Box((0.390, 0.045, 0.034)), origin=Origin(xyz=(0.0, -0.035, 0.017)), material=rubber, name="bottom_seal")
    closure_panel.visual(Box((0.120, 0.070, 0.070)), origin=Origin(xyz=(0.0, -0.072, 0.770)), material=worn_steel, name="stem_anchor")
    closure_panel.visual(Cylinder(radius=0.016, length=0.780), origin=Origin(xyz=(0.0, -0.105, 1.150)), material=worn_steel, name="threaded_screw")
    for idx, z in enumerate([0.80, 0.86, 0.92, 0.98, 1.04, 1.10, 1.16, 1.22, 1.28, 1.34, 1.40, 1.46, 1.52]):
        closure_panel.visual(Cylinder(radius=0.019, length=0.006), origin=Origin(xyz=(0.0, -0.105, z)), material=dark, name=f"screw_thread_{idx}")

    actuator_wheel = model.part("actuator_wheel")
    actuator_wheel.visual(
        mesh_from_cadquery(_handwheel_shape(), "actuator_wheel"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=yellow,
        name="handwheel",
    )

    lock_tab = model.part("lock_tab")
    lock_tab.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_steel,
        name="pivot_boss",
    )
    lock_tab.visual(Box((0.012, 0.035, 0.170)), origin=Origin(xyz=(0.0, 0.0, -0.085)), material=dark, name="tab_plate")
    lock_tab.visual(Box((0.014, 0.055, 0.026)), origin=Origin(xyz=(0.0, 0.018, -0.168)), material=dark, name="pawl_tooth")

    model.articulation(
        "frame_to_closure_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=closure_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.18, lower=0.0, upper=0.35),
    )
    model.articulation(
        "frame_to_actuator_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=actuator_wheel,
        origin=Origin(xyz=(0.45, -0.04, 1.54)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
    )
    model.articulation(
        "frame_to_lock_tab",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lock_tab,
        origin=Origin(xyz=(0.372, -0.155, 1.65)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.65, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("closure_panel")
    wheel = object_model.get_part("actuator_wheel")
    tab = object_model.get_part("lock_tab")
    slide = object_model.get_articulation("frame_to_closure_panel")
    wheel_joint = object_model.get_articulation("frame_to_actuator_wheel")
    tab_joint = object_model.get_articulation("frame_to_lock_tab")

    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="bottom_seal",
        negative_elem="bottom_sill",
        max_gap=0.002,
        max_penetration=0.0,
        name="gate seal sits on the sill at rest",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="x",
        positive_elem="front_lip_1",
        negative_elem="gate_plate",
        min_gap=0.010,
        max_gap=0.055,
        name="right guide lip clears sliding panel",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="x",
        positive_elem="gate_plate",
        negative_elem="front_lip_0",
        min_gap=0.010,
        max_gap=0.055,
        name="left guide lip clears sliding panel",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="yz",
        elem_a="handwheel",
        elem_b="transverse_shaft",
        min_overlap=0.020,
        name="handwheel is centered on transverse shaft",
    )
    ctx.expect_gap(
        wheel,
        tab,
        axis="x",
        positive_elem="handwheel",
        negative_elem="tab_plate",
        min_gap=0.010,
        max_gap=0.060,
        name="lock tab sits beside handwheel without rubbing",
    )

    rest_pos = ctx.part_world_position(panel)
    with ctx.pose({slide: 0.35}):
        raised_pos = ctx.part_world_position(panel)
        ctx.expect_gap(
            frame,
            panel,
            axis="z",
            positive_elem="top_header",
            negative_elem="gate_plate",
            min_gap=0.010,
            max_gap=0.060,
            name="raised panel remains below top header",
        )
    ctx.check(
        "closure panel slides upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    wheel_rest = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: math.pi}):
        wheel_turned = ctx.part_world_position(wheel)
    ctx.check(
        "actuator wheel rotates about fixed shaft center",
        wheel_rest is not None and wheel_turned is not None and max(abs(a - b) for a, b in zip(wheel_rest, wheel_turned)) < 1e-6,
        details=f"rest={wheel_rest}, turned={wheel_turned}",
    )

    tab_rest = ctx.part_element_world_aabb(tab, elem="pawl_tooth")
    with ctx.pose({tab_joint: 0.45}):
        tab_swung = ctx.part_element_world_aabb(tab, elem="pawl_tooth")
    ctx.check(
        "lock tab pawl swings at yoke pivot",
        tab_rest is not None
        and tab_swung is not None
        and abs(tab_rest[0][1] - tab_swung[0][1]) > 0.015,
        details=f"rest={tab_rest}, swung={tab_swung}",
    )

    return ctx.report()


object_model = build_object_model()
