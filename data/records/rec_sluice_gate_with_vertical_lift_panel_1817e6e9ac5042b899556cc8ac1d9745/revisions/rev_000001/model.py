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
    mesh_from_cadquery,
)
import cadquery as cq


def _handwheel_geometry() -> cq.Workplane:
    """Four-spoke industrial handwheel, centered on local Z."""
    thickness = 0.035
    outer_ring = cq.Workplane("XY").circle(0.19).circle(0.152).extrude(thickness)
    spoke_x = cq.Workplane("XY").rect(0.320, 0.036).extrude(thickness)
    spoke_y = cq.Workplane("XY").rect(0.036, 0.320).extrude(thickness)
    hub = cq.Workplane("XY").circle(0.058).extrude(thickness)
    rim_grip = (
        cq.Workplane("XY")
        .center(0.0, 0.168)
        .circle(0.030)
        .extrude(0.055)
    )
    return (
        outer_ring.union(spoke_x).union(spoke_y).union(hub).union(rim_grip)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drainage_channel_sluice_gate")

    concrete = model.material("cast_concrete", color=(0.55, 0.53, 0.48, 1.0))
    galvanized = model.material("galvanized_steel", color=(0.64, 0.68, 0.68, 1.0))
    dark_steel = model.material("dark_gate_steel", color=(0.15, 0.18, 0.20, 1.0))
    yellow = model.material("yellow_handwheel", color=(0.95, 0.68, 0.10, 1.0))
    cap_blue = model.material("blue_protective_cap", color=(0.05, 0.22, 0.55, 1.0))

    frame = model.part("frame")

    # Drainage-channel civil works and fixed gate frame.
    frame.visual(
        Box((1.70, 0.56, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="channel_floor",
    )
    for x, name in ((-0.78, "abutment_0"), (0.78, "abutment_1")):
        frame.visual(
            Box((0.18, 0.56, 0.50)),
            origin=Origin(xyz=(x, 0.0, 0.31)),
            material=concrete,
            name=name,
        )

    frame.visual(
        Box((1.14, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=galvanized,
        name="sill",
    )

    # Twin upright C-channel guides.  The flanges stand clear of the gate plate,
    # leaving a visible vertical sliding slot on both sides.
    for side, sx in (("guide_0", -1.0), ("guide_1", 1.0)):
        frame.visual(
            Box((0.08, 0.24, 1.52)),
            origin=Origin(xyz=(sx * 0.64, 0.0, 0.96)),
            material=galvanized,
            name=f"{side}_web",
        )
    for x, y, name in (
        (-0.59, -0.075, "guide_0_front_flange"),
        (-0.59, 0.075, "guide_0_rear_flange"),
        (0.59, -0.075, "guide_1_front_flange"),
        (0.59, 0.075, "guide_1_rear_flange"),
    ):
        frame.visual(
            Box((0.12, 0.035, 1.52)),
            origin=Origin(xyz=(x, y, 0.96)),
            material=galvanized,
            name=name,
        )

    frame.visual(
        Box((1.46, 0.28, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
        material=galvanized,
        name="top_crossbeam",
    )
    frame.visual(
        Box((0.62, 0.17, 0.035)),
        origin=Origin(xyz=(0.0, 0.215, 1.8075)),
        material=galvanized,
        name="rear_hinge_mount",
    )
    frame.visual(
        Box((0.34, 0.22, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.84)),
        material=galvanized,
        name="actuator_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 1.82)),
        material=dark_steel,
        name="screw_stem",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.91)),
        material=dark_steel,
        name="screw_head",
    )

    # Rear cap hinge support fixed to the top of the frame.
    for x, name in ((-0.22, "rear_hinge_bracket_0"), (0.22, "rear_hinge_bracket_1")):
        frame.visual(
            Box((0.060, 0.040, 0.19)),
            origin=Origin(xyz=(x, 0.285, 1.885)),
            material=galvanized,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.018, length=0.56),
        origin=Origin(xyz=(0.0, 0.270, 1.985), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_hinge_pin",
    )

    panel = model.part("closure_panel")
    panel.visual(
        Box((1.04, 0.060, 0.95)),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=dark_steel,
        name="gate_plate",
    )
    for z, name in ((0.23, "stiffener_low"), (0.50, "stiffener_mid"), (0.77, "stiffener_high")):
        panel.visual(
            Box((0.94, 0.030, 0.045)),
            origin=Origin(xyz=(0.0, -0.043, z)),
            material=galvanized,
            name=name,
        )
    for x, name in ((-0.500, "edge_shoe_0"), (0.500, "edge_shoe_1")):
        panel.visual(
            Box((0.040, 0.070, 0.95)),
            origin=Origin(xyz=(x, 0.0, 0.475)),
            material=galvanized,
            name=name,
        )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_geometry(), "industrial_handwheel"),
        material=yellow,
        name="wheel",
    )
    handwheel.visual(
        Cylinder(radius=0.045, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=yellow,
        name="hub_boss",
    )
    handwheel.visual(
        Cylinder(radius=0.020, length=0.450),
        origin=Origin(xyz=(0.0, 0.0, -0.290)),
        material=dark_steel,
        name="drive_stem",
    )

    cap = model.part("protective_cap")
    # Slotted two-piece top cover clears the actuator stem while the hinged cap
    # still shields the exposed screw head below the handwheel.
    for x, name in ((-0.185, "top_leaf_0"), (0.185, "top_leaf_1")):
        cap.visual(
            Box((0.190, 0.420, 0.040)),
            origin=Origin(xyz=(x, -0.210, -0.020)),
            material=cap_blue,
            name=name,
        )
    cap.visual(
        Box((0.560, 0.035, 0.120)),
        origin=Origin(xyz=(0.0, -0.420, -0.100)),
        material=cap_blue,
        name="front_lip",
    )
    cap.visual(
        Box((0.560, 0.040, 0.040)),
        origin=Origin(xyz=(0.0, -0.035, -0.020)),
        material=cap_blue,
        name="rear_leaf",
    )
    for x, name in ((-0.280, "side_skirt_0"), (0.280, "side_skirt_1")):
        cap.visual(
            Box((0.035, 0.420, 0.120)),
            origin=Origin(xyz=(x, -0.210, -0.100)),
            material=cap_blue,
            name=name,
        )

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.08, lower=0.0, upper=0.45),
    )
    model.articulation(
        "frame_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.0, 2.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=6.0),
    )
    model.articulation(
        "frame_to_cap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cap,
        origin=Origin(xyz=(0.0, 0.240, 1.985)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.0, lower=0.0, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("closure_panel")
    cap = object_model.get_part("protective_cap")
    wheel = object_model.get_part("handwheel")
    slide = object_model.get_articulation("frame_to_panel")
    wheel_joint = object_model.get_articulation("frame_to_handwheel")
    cap_hinge = object_model.get_articulation("frame_to_cap")

    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="gate_plate",
        negative_elem="sill",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed gate bears on sill",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="x",
        positive_elem="guide_1_front_flange",
        negative_elem="gate_plate",
        min_gap=0.005,
        max_gap=0.020,
        name="right guide has running clearance",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="x",
        positive_elem="gate_plate",
        negative_elem="guide_0_front_flange",
        min_gap=0.005,
        max_gap=0.020,
        name="left guide has running clearance",
    )

    rest_panel_z = ctx.part_world_position(panel)[2]
    with ctx.pose({slide: 0.45}):
        raised_panel_z = ctx.part_world_position(panel)[2]
        ctx.expect_gap(
            frame,
            panel,
            axis="z",
            positive_elem="top_crossbeam",
            negative_elem="gate_plate",
            min_gap=0.040,
            name="raised panel clears top crossbeam",
        )
    ctx.check(
        "closure panel slides upward",
        raised_panel_z > rest_panel_z + 0.40,
        details=f"rest_z={rest_panel_z}, raised_z={raised_panel_z}",
    )

    closed_cap_aabb = ctx.part_world_aabb(cap)
    with ctx.pose({cap_hinge: 0.75}):
        open_cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "protective cap opens upward",
        closed_cap_aabb is not None
        and open_cap_aabb is not None
        and open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.12,
        details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
    )

    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: 6.28}):
        spun_wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "handwheel rotates in place continuously",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and max(abs(a - b) for a, b in zip(rest_wheel_pos, spun_wheel_pos)) < 1e-6,
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
