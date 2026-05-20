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


def _build_handwheel_shape() -> object:
    hub = cq.Workplane("XZ").circle(0.055).extrude(0.05)
    collar = cq.Workplane("XZ").circle(0.08).extrude(0.012, both=True)
    rim = (
        cq.Workplane("XZ")
        .circle(0.196)
        .circle(0.164)
        .extrude(0.018)
        .translate((0.0, 0.026, 0.0))
    )

    wheel = hub.union(collar).union(rim)
    for spoke_angle_deg in (0.0, 30.0, 60.0, 90.0, 120.0, 150.0):
        spoke = (
            cq.Workplane("XZ")
            .rect(0.34, 0.028)
            .extrude(0.016)
            .translate((0.0, 0.027, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), spoke_angle_deg)
        )
        wheel = wheel.union(spoke)

    grip = (
        cq.Workplane("XZ")
        .center(0.0, 0.205)
        .circle(0.024)
        .extrude(0.07)
        .translate((0.0, 0.014, 0.0))
    )
    return wheel.union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flood_control_gate")

    frame_steel = model.material("frame_steel", rgba=(0.33, 0.36, 0.39, 1.0))
    gate_blue = model.material("gate_blue", rgba=(0.19, 0.39, 0.50, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.54, 0.56, 0.58, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.35, 0.36, 2.45)),
        origin=Origin(xyz=(-0.805, 0.0, 1.225)),
        material=frame_steel,
        name="column_0",
    )
    frame.visual(
        Box((0.35, 0.36, 2.45)),
        origin=Origin(xyz=(0.805, 0.0, 1.225)),
        material=frame_steel,
        name="column_1",
    )
    frame.visual(
        Box((1.96, 0.36, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 2.30)),
        material=frame_steel,
        name="top_beam",
    )
    frame.visual(
        Box((1.96, 0.42, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=frame_steel,
        name="sill",
    )
    frame.visual(
        Box((0.09, 0.08, 2.48)),
        origin=Origin(xyz=(-0.545, -0.095, 1.24)),
        material=frame_steel,
        name="guide_0",
    )
    frame.visual(
        Box((0.09, 0.08, 2.48)),
        origin=Origin(xyz=(0.545, -0.095, 1.24)),
        material=frame_steel,
        name="guide_1",
    )
    frame.visual(
        Box((0.70, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.41)),
        material=frame_steel,
        name="operator_pedestal",
    )
    frame.visual(
        Box((0.56, 0.42, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 2.62)),
        material=housing_gray,
        name="operator_box",
    )
    frame.visual(
        Cylinder(radius=0.09, length=0.01),
        origin=Origin(xyz=(0.0, 0.211, 2.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_gray,
        name="front_flange",
    )

    panel = model.part("panel")
    panel.visual(
        Box((0.94, 0.05, 1.62)),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=gate_blue,
        name="leaf",
    )
    panel.visual(
        Box((0.10, 0.08, 1.62)),
        origin=Origin(xyz=(-0.42, 0.02, 0.81)),
        material=gate_blue,
        name="stile_0",
    )
    panel.visual(
        Box((0.10, 0.08, 1.62)),
        origin=Origin(xyz=(0.42, 0.02, 0.81)),
        material=gate_blue,
        name="stile_1",
    )
    for idx, rib_z in enumerate((0.38, 0.81, 1.24)):
        panel.visual(
            Box((0.74, 0.08, 0.10)),
            origin=Origin(xyz=(0.0, 0.02, rib_z)),
            material=gate_blue,
            name=f"rib_{idx}",
        )
    panel.visual(
        Box((0.94, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.02, 1.57)),
        material=gate_blue,
        name="top_rib",
    )
    panel.visual(
        Box((0.06, 0.05, 1.66)),
        origin=Origin(xyz=(-0.50, -0.03, 0.83)),
        material=gate_blue,
        name="guide_shoe_0",
    )
    panel.visual(
        Box((0.06, 0.05, 1.66)),
        origin=Origin(xyz=(0.50, -0.03, 0.83)),
        material=gate_blue,
        name="guide_shoe_1",
    )

    model.articulation(
        "gate_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.55, effort=2500.0, velocity=0.08),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_build_handwheel_shape(), "handwheel"),
        material=frame_steel,
        name="wheel",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.266, 2.62)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=6.0),
    )

    access_door = model.part("access_door")
    access_door.visual(
        Box((0.018, 0.28, 0.22)),
        origin=Origin(xyz=(0.009, -0.14, 0.11)),
        material=housing_gray,
        name="door_panel",
    )
    access_door.visual(
        Box((0.010, 0.18, 0.12)),
        origin=Origin(xyz=(0.023, -0.14, 0.11)),
        material=frame_steel,
        name="door_stiffener",
    )
    access_door.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.033, -0.22, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_steel,
        name="latch_pull",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=access_door,
        origin=Origin(xyz=(0.28, 0.14, 2.51)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=12.0, velocity=1.6),
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
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    gate_lift = object_model.get_articulation("gate_lift")
    handwheel = object_model.get_part("handwheel")
    access_door = object_model.get_part("access_door")
    wheel_spin = object_model.get_articulation("wheel_spin")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        frame,
        handwheel,
        elem_a="front_flange",
        elem_b="wheel",
        reason="The handwheel hub is simplified as a solid casting seated over the fixed operator flange instead of modeling the bore and keyed shaft explicitly.",
    )

    with ctx.pose({gate_lift: 0.0}):
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="leaf",
            negative_elem="sill",
            min_gap=0.005,
            max_gap=0.015,
            name="panel seats just above the sill",
        )
        ctx.expect_contact(
            panel,
            frame,
            elem_a="guide_shoe_0",
            elem_b="guide_0",
            name="left guide shoe bears on the guide bar",
        )
        ctx.expect_contact(
            panel,
            frame,
            elem_a="guide_shoe_1",
            elem_b="guide_1",
            name="right guide shoe bears on the guide bar",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xz",
            elem_a="guide_shoe_0",
            elem_b="guide_0",
            min_overlap=0.020,
            name="left guide shoe stays engaged with the guide bar",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xz",
            elem_a="guide_shoe_1",
            elem_b="guide_1",
            min_overlap=0.020,
            name="right guide shoe stays engaged with the guide bar",
        )

    rest_pos = ctx.part_world_position(panel)
    with ctx.pose({gate_lift: 0.55}):
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="leaf",
            negative_elem="sill",
            min_gap=0.54,
            name="raised panel clears high above the sill",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xz",
            elem_a="guide_shoe_0",
            elem_b="guide_0",
            min_overlap=0.020,
            name="left guide shoe remains retained at full lift",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xz",
            elem_a="guide_shoe_1",
            elem_b="guide_1",
            min_overlap=0.020,
            name="right guide shoe remains retained at full lift",
        )
        raised_pos = ctx.part_world_position(panel)
    ctx.check(
        "panel lifts upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.50,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_contact(
            access_door,
            frame,
            elem_a="door_panel",
            elem_b="operator_box",
            name="access door closes against the operator box side",
        )

    with ctx.pose({door_hinge: 1.2}):
        door_open = ctx.part_element_world_aabb(access_door, elem="door_panel")

    with ctx.pose({wheel_spin: 0.0}):
        ctx.expect_overlap(
            handwheel,
            frame,
            axes="xz",
            elem_a="wheel",
            elem_b="front_flange",
            min_overlap=0.10,
            name="handwheel stays centered on the front operator flange",
        )
        wheel_rest = ctx.part_element_world_aabb(handwheel, elem="wheel")
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        ctx.expect_overlap(
            handwheel,
            frame,
            axes="xz",
            elem_a="wheel",
            elem_b="front_flange",
            min_overlap=0.10,
            name="handwheel remains centered on the flange while spinning",
        )
        wheel_quarter = ctx.part_element_world_aabb(handwheel, elem="wheel")
    door_closed = ctx.part_element_world_aabb(access_door, elem="door_panel")
    door_closed_center = None
    door_open_center = None
    if door_closed is not None:
        door_closed_center = tuple((lo + hi) * 0.5 for lo, hi in zip(door_closed[0], door_closed[1]))
    if door_open is not None:
        door_open_center = tuple((lo + hi) * 0.5 for lo, hi in zip(door_open[0], door_open[1]))
    ctx.check(
        "access door swings outward from the box side",
        door_closed_center is not None
        and door_open_center is not None
        and door_open_center[0] > door_closed_center[0] + 0.10,
        details=f"closed={door_closed_center}, open={door_open_center}",
    )
    wheel_rest_max = None
    wheel_quarter_max = None
    if wheel_rest is not None:
        wheel_rest_max = wheel_rest[1]
    if wheel_quarter is not None:
        wheel_quarter_max = wheel_quarter[1]
    ctx.check(
        "handwheel grip sweeps around the rim",
        wheel_rest_max is not None
        and wheel_quarter_max is not None
        and abs(wheel_rest_max[0] - wheel_quarter_max[0]) > 0.02
        and abs(wheel_rest_max[2] - wheel_quarter_max[2]) > 0.02,
        details=f"rest_max={wheel_rest_max}, quarter_turn_max={wheel_quarter_max}",
    )

    return ctx.report()


object_model = build_object_model()
