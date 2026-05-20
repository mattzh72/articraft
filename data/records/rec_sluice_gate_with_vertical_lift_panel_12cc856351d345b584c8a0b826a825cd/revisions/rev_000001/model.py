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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flood_control_sluice_gate")

    concrete = model.material("weathered_concrete", rgba=(0.56, 0.57, 0.55, 1.0))
    dark_steel = model.material("dark_galvanized_steel", rgba=(0.18, 0.20, 0.21, 1.0))
    painted_gate = model.material("blue_green_painted_steel", rgba=(0.06, 0.30, 0.34, 1.0))
    worn_edges = model.material("worn_bare_metal", rgba=(0.58, 0.60, 0.58, 1.0))
    gearbox_gray = model.material("gearbox_gray", rgba=(0.31, 0.34, 0.35, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.72, 0.12, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    handwheel_ring = mesh_from_geometry(
        TorusGeometry(radius=0.32, tube=0.035, radial_segments=18, tubular_segments=72),
        "handwheel_ring",
    )

    frame = model.part("frame")
    frame.visual(
        Box((3.90, 0.72, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=concrete,
        name="bottom_sill",
    )
    frame.visual(
        Box((0.45, 0.72, 3.65)),
        origin=Origin(xyz=(-1.725, 0.0, 1.825)),
        material=concrete,
        name="side_pier_0",
    )
    frame.visual(
        Box((0.45, 0.72, 3.65)),
        origin=Origin(xyz=(1.725, 0.0, 1.825)),
        material=concrete,
        name="side_pier_1",
    )
    frame.visual(
        Box((3.90, 0.76, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 3.75)),
        material=concrete,
        name="top_beam",
    )
    frame.visual(
        Box((1.25, 0.10, 0.10)),
        origin=Origin(xyz=(-1.05, -0.405, 3.48)),
        material=dark_steel,
        name="top_lintel_angle_0",
    )
    frame.visual(
        Box((1.25, 0.10, 0.10)),
        origin=Origin(xyz=(1.05, -0.405, 3.48)),
        material=dark_steel,
        name="top_lintel_angle_1",
    )
    frame.visual(
        Box((3.30, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, -0.625, 0.39)),
        material=dark_steel,
        name="sill_wear_angle",
    )

    frame.visual(
        Box((0.16, 0.28, 3.05)),
        origin=Origin(xyz=(-1.48, -0.47, 1.875)),
        material=dark_steel,
        name="guide_web_0",
    )
    frame.visual(
        Box((0.08, 0.06, 3.05)),
        origin=Origin(xyz=(-1.39, -0.58, 1.875)),
        material=dark_steel,
        name="guide_front_lip_0",
    )
    frame.visual(
        Box((0.08, 0.06, 3.05)),
        origin=Origin(xyz=(-1.39, -0.36, 1.875)),
        material=dark_steel,
        name="guide_rear_lip_0",
    )
    frame.visual(
        Box((0.26, 0.12, 0.22)),
        origin=Origin(xyz=(-1.55, -0.47, 3.35)),
        material=dark_steel,
        name="guide_cap_0",
    )
    frame.visual(
        Box((0.16, 0.28, 3.05)),
        origin=Origin(xyz=(1.48, -0.47, 1.875)),
        material=dark_steel,
        name="guide_web_1",
    )
    frame.visual(
        Box((0.08, 0.06, 3.05)),
        origin=Origin(xyz=(1.39, -0.58, 1.875)),
        material=dark_steel,
        name="guide_front_lip_1",
    )
    frame.visual(
        Box((0.08, 0.06, 3.05)),
        origin=Origin(xyz=(1.39, -0.36, 1.875)),
        material=dark_steel,
        name="guide_rear_lip_1",
    )
    frame.visual(
        Box((0.26, 0.12, 0.22)),
        origin=Origin(xyz=(1.55, -0.47, 3.35)),
        material=dark_steel,
        name="guide_cap_1",
    )

    for x in (-1.48, 1.48):
        for z in (0.72, 1.72, 2.72):
            frame.visual(
                Cylinder(radius=0.045, length=0.08),
                origin=Origin(xyz=(x, -0.615, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=worn_edges,
                name=f"anchor_bolt_{x}_{z}",
            )

    lift_panel = model.part("lift_panel")
    lift_panel.visual(
        Box((2.65, 0.12, 2.65)),
        origin=Origin(xyz=(0.0, -0.47, 1.325)),
        material=painted_gate,
        name="gate_plate",
    )
    for z in (0.48, 1.30, 2.12):
        lift_panel.visual(
            Box((2.42, 0.11, 0.10)),
            origin=Origin(xyz=(0.0, -0.555, z)),
            material=dark_steel,
            name=f"horizontal_stiffener_{z}",
        )
    for x in (-0.82, 0.0, 0.82):
        lift_panel.visual(
            Box((0.12, 0.11, 2.38)),
            origin=Origin(xyz=(x, -0.555, 1.325)),
            material=dark_steel,
            name=f"vertical_stiffener_{x}",
        )
    lift_panel.visual(
        Box((2.25, 0.09, 0.08)),
        origin=Origin(xyz=(0.0, -0.61, 1.33), rpy=(0.0, -0.77, 0.0)),
        material=dark_steel,
        name="diagonal_brace_0",
    )
    lift_panel.visual(
        Box((2.25, 0.09, 0.08)),
        origin=Origin(xyz=(0.0, -0.62, 1.33), rpy=(0.0, 0.77, 0.0)),
        material=dark_steel,
        name="diagonal_brace_1",
    )
    lift_panel.visual(
        Box((0.52, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, -0.47, 2.74)),
        material=dark_steel,
        name="lifting_lug",
    )
    lift_panel.visual(
        Cylinder(radius=0.035, length=1.00),
        origin=Origin(xyz=(0.0, -0.47, 3.15)),
        material=worn_edges,
        name="lift_screw",
    )
    lift_panel.visual(
        Box((2.55, 0.045, 0.08)),
        origin=Origin(xyz=(0.0, -0.545, 0.04)),
        material=black_rubber,
        name="bottom_seal",
    )

    operator_box = model.part("operator_box")
    operator_box.visual(
        Box((1.70, 0.90, 0.12)),
        origin=Origin(xyz=(0.0, -0.02, 0.06)),
        material=dark_steel,
        name="mounting_plate",
    )
    operator_box.visual(
        Box((1.45, 0.78, 0.72)),
        origin=Origin(xyz=(0.0, -0.02, 0.48)),
        material=gearbox_gray,
        name="gearbox_housing",
    )
    operator_box.visual(
        Box((1.56, 0.84, 0.10)),
        origin=Origin(xyz=(0.0, -0.02, 0.89)),
        material=dark_steel,
        name="weather_cap",
    )
    operator_box.visual(
        Box((0.34, 0.22, 0.18)),
        origin=Origin(xyz=(0.0, -0.42, 0.18)),
        material=dark_steel,
        name="front_bearing_block",
    )
    operator_box.visual(
        Cylinder(radius=0.075, length=0.11),
        origin=Origin(xyz=(0.0, -0.465, 0.52), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edges,
        name="handwheel_bushing",
    )
    operator_box.visual(
        Cylinder(radius=0.012, length=0.46),
        origin=Origin(xyz=(0.765, -0.31, 0.48)),
        material=worn_edges,
        name="hinge_pin",
    )
    operator_box.visual(
        Cylinder(radius=0.040, length=0.14),
        origin=Origin(xyz=(0.765, -0.31, 0.32)),
        material=dark_steel,
        name="lower_hinge_knuckle",
    )
    operator_box.visual(
        Cylinder(radius=0.040, length=0.14),
        origin=Origin(xyz=(0.765, -0.31, 0.64)),
        material=dark_steel,
        name="upper_hinge_knuckle",
    )
    for x in (-0.68, 0.68):
        for y in (-0.36, 0.32):
            operator_box.visual(
                Cylinder(radius=0.035, length=0.025),
                origin=Origin(xyz=(x, y, 0.135)),
                material=worn_edges,
                name=f"base_bolt_{x}_{y}",
            )

    handwheel = model.part("handwheel")
    handwheel.visual(
        Cylinder(radius=0.052, length=0.24),
        origin=Origin(xyz=(0.0, -0.12, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edges,
        name="shaft",
    )
    handwheel.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(0.0, -0.30, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    handwheel.visual(
        handwheel_ring,
        origin=Origin(xyz=(0.0, -0.34, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_yellow,
        name="rim",
    )
    for angle, name in (
        (0.0, "spoke_0"),
        (math.pi / 2.0, "spoke_1"),
        (math.pi / 4.0, "spoke_2"),
        (-math.pi / 4.0, "spoke_3"),
    ):
        handwheel.visual(
            Box((0.64, 0.045, 0.055)),
            origin=Origin(xyz=(0.0, -0.34, 0.0), rpy=(0.0, angle, 0.0)),
            material=safety_yellow,
            name=name,
        )

    inspection_door = model.part("inspection_door")
    inspection_door.visual(
        Cylinder(radius=0.036, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    inspection_door.visual(
        Box((0.050, 0.055, 0.14)),
        origin=Origin(xyz=(0.045, 0.0425, 0.0)),
        material=dark_steel,
        name="hinge_leaf",
    )
    inspection_door.visual(
        Box((0.045, 0.38, 0.40)),
        origin=Origin(xyz=(0.026, 0.25, 0.0)),
        material=gearbox_gray,
        name="door_panel",
    )
    inspection_door.visual(
        Box((0.052, 0.32, 0.035)),
        origin=Origin(xyz=(0.035, 0.25, 0.13)),
        material=dark_steel,
        name="upper_door_rib",
    )
    inspection_door.visual(
        Box((0.052, 0.32, 0.035)),
        origin=Origin(xyz=(0.035, 0.25, -0.13)),
        material=dark_steel,
        name="lower_door_rib",
    )
    inspection_door.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.053, 0.37, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edges,
        name="turn_latch",
    )

    model.articulation(
        "panel_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lift_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120000.0, velocity=0.12, lower=0.0, upper=1.10),
    )
    model.articulation(
        "operator_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=operator_box,
        origin=Origin(xyz=(0.0, -0.02, 4.00)),
    )
    model.articulation(
        "handwheel_rotation",
        ArticulationType.CONTINUOUS,
        parent=operator_box,
        child=handwheel,
        origin=Origin(xyz=(0.0, -0.41, 0.52)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=6.0),
    )
    model.articulation(
        "inspection_hinge",
        ArticulationType.REVOLUTE,
        parent=operator_box,
        child=inspection_door,
        origin=Origin(xyz=(0.765, -0.31, 0.48)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    panel = object_model.get_part("lift_panel")
    operator_box = object_model.get_part("operator_box")
    handwheel = object_model.get_part("handwheel")
    door = object_model.get_part("inspection_door")
    panel_slide = object_model.get_articulation("panel_slide")
    handwheel_rotation = object_model.get_articulation("handwheel_rotation")
    inspection_hinge = object_model.get_articulation("inspection_hinge")

    ctx.allow_overlap(
        operator_box,
        door,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The visible hinge pin is intentionally captured through the inspection door barrel.",
    )
    ctx.allow_overlap(
        operator_box,
        handwheel,
        elem_a="handwheel_bushing",
        elem_b="shaft",
        reason="The handwheel shaft is intentionally captured through the gearbox bushing.",
    )

    ctx.expect_contact(
        operator_box,
        frame,
        elem_a="mounting_plate",
        elem_b="top_beam",
        name="operator box mounting plate sits on the top beam",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="x",
        positive_elem="guide_web_1",
        negative_elem="gate_plate",
        min_gap=0.03,
        max_gap=0.12,
        name="gate clears the guide on one side",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="x",
        positive_elem="gate_plate",
        negative_elem="guide_web_0",
        min_gap=0.03,
        max_gap=0.12,
        name="gate clears the guide on the opposite side",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="z",
        elem_a="gate_plate",
        elem_b="guide_web_0",
        min_overlap=2.0,
        name="closed gate is deeply retained by the side guide",
    )
    ctx.expect_within(
        operator_box,
        door,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.03,
        name="inspection door barrel surrounds the hinge pin",
    )
    ctx.expect_overlap(
        operator_box,
        door,
        axes="z",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.14,
        name="hinge pin passes through the inspection door barrel",
    )
    ctx.expect_within(
        handwheel,
        operator_box,
        axes="xz",
        inner_elem="shaft",
        outer_elem="handwheel_bushing",
        margin=0.03,
        name="handwheel shaft is centered in the gearbox bushing",
    )
    ctx.expect_overlap(
        handwheel,
        operator_box,
        axes="y",
        elem_a="shaft",
        elem_b="handwheel_bushing",
        min_overlap=0.10,
        name="handwheel shaft remains inserted through the bushing",
    )

    rest_panel_position = ctx.part_world_position(panel)
    with ctx.pose({panel_slide: 1.10}):
        raised_panel_position = ctx.part_world_position(panel)
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="gate_plate",
            elem_b="guide_web_0",
            min_overlap=1.0,
            name="raised gate remains captured in the side guide",
        )
    ctx.check(
        "panel slide raises the lift panel",
        rest_panel_position is not None
        and raised_panel_position is not None
        and raised_panel_position[2] > rest_panel_position[2] + 1.0,
        details=f"rest={rest_panel_position}, raised={raised_panel_position}",
    )

    ctx.check(
        "handwheel is a continuous rotary control",
        handwheel_rotation.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={handwheel_rotation.articulation_type}",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({inspection_hinge: 1.20}):
        opened_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "inspection door swings outward on side hinge",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][0] > closed_door_aabb[1][0] + 0.20,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
