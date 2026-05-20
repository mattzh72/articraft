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


STEEL = Material("galvanized_steel", rgba=(0.52, 0.55, 0.55, 1.0))
DARK_STEEL = Material("dark_oiled_steel", rgba=(0.08, 0.09, 0.09, 1.0))
GATE_BLUE = Material("painted_gate_steel", rgba=(0.16, 0.28, 0.34, 1.0))
CONCRETE = Material("weathered_concrete", rgba=(0.43, 0.43, 0.40, 1.0))
BLACK = Material("black_handwheel", rgba=(0.01, 0.012, 0.012, 1.0))
SAFETY_YELLOW = Material("safety_yellow_pawl", rgba=(0.95, 0.70, 0.08, 1.0))


def _handwheel_mesh():
    """Large cast handwheel: continuous rim, hub, and broad spokes."""
    thickness = 0.060
    rim = (
        cq.Workplane("XY")
        .circle(0.320)
        .circle(0.255)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    wheel = rim
    for angle in (0.0, 60.0, 120.0):
        spoke = (
            cq.Workplane("XY")
            .box(0.540, 0.040, 0.045)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        wheel = wheel.union(spoke)
    hub = (
        cq.Workplane("XY")
        .circle(0.085)
        .extrude(0.090)
        .translate((0.0, 0.0, -0.045))
    )
    boss = (
        cq.Workplane("XY")
        .circle(0.045)
        .extrude(0.125)
        .translate((0.0, 0.0, -0.0625))
    )
    return wheel.union(hub).union(boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_channel_sluice_gate")

    frame = model.part("frame")
    frame.visual(
        Box((0.30, 0.24, 3.00)),
        origin=Origin(xyz=(-1.32, 0.08, 1.50)),
        material=CONCRETE,
        name="side_wall_0",
    )
    frame.visual(
        Box((0.30, 0.24, 3.00)),
        origin=Origin(xyz=(1.32, 0.08, 1.50)),
        material=CONCRETE,
        name="side_wall_1",
    )
    frame.visual(
        Box((2.94, 0.24, 0.24)),
        origin=Origin(xyz=(0.0, 0.08, 3.12)),
        material=CONCRETE,
        name="top_beam",
    )
    frame.visual(
        Box((2.94, 0.28, 0.20)),
        origin=Origin(xyz=(0.0, 0.08, 0.10)),
        material=CONCRETE,
        name="sill",
    )
    frame.visual(
        Box((1.88, 0.025, 2.50)),
        origin=Origin(xyz=(0.0, 0.185, 1.45)),
        material=Material("dark_channel_shadow", rgba=(0.03, 0.04, 0.045, 1.0)),
        name="water_shadow",
    )

    for name, x in (("guide_rail_0", -0.98), ("guide_rail_1", 0.98)):
        rail = model.part(name)
        rail.visual(
            Box((0.16, 0.18, 2.65)),
            origin=Origin(),
            material=STEEL,
            name="guide_channel",
        )
        rail.visual(
            Box((0.035, 0.012, 2.45)),
            origin=Origin(xyz=(0.0, -0.096, 0.0)),
            material=DARK_STEEL,
            name="front_wear_slot",
        )
        for index, z in enumerate((-0.80, 0.0, 0.80)):
            rail.visual(
                Box((0.42, 0.050, 0.12)),
                origin=Origin(xyz=(0.0, 0.115, z)),
                material=STEEL,
                name=f"stand_off_{index}",
            )
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.FIXED,
            parent=frame,
            child=rail,
            origin=Origin(xyz=(x, -0.18, 1.45)),
        )

    lift_plate = model.part("lift_plate")
    lift_plate.visual(
        Box((1.68, 0.070, 2.00)),
        origin=Origin(),
        material=GATE_BLUE,
        name="plate_skin",
    )
    lift_plate.visual(
        Box((1.54, 0.040, 0.085)),
        origin=Origin(xyz=(0.0, -0.055, -0.58)),
        material=STEEL,
        name="lower_stiffener",
    )
    lift_plate.visual(
        Box((1.54, 0.040, 0.085)),
        origin=Origin(xyz=(0.0, -0.055, 0.08)),
        material=STEEL,
        name="middle_stiffener",
    )
    lift_plate.visual(
        Box((1.54, 0.040, 0.085)),
        origin=Origin(xyz=(0.0, -0.055, 0.70)),
        material=STEEL,
        name="upper_stiffener",
    )
    lift_plate.visual(
        Box((0.11, 0.040, 1.82)),
        origin=Origin(xyz=(0.0, -0.055, 0.02)),
        material=STEEL,
        name="center_rib",
    )
    lift_plate.visual(
        Box((0.28, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, -0.10, 0.99)),
        material=STEEL,
        name="lifting_lug",
    )
    for index, z in enumerate((-0.58, 0.62)):
        lift_plate.visual(
            Box((0.060, 0.060, 0.24)),
            origin=Origin(xyz=(-0.870, 0.0, z)),
            material=STEEL,
            name=f"guide_shoe_0_{index}",
        )
        lift_plate.visual(
            Box((0.060, 0.060, 0.24)),
            origin=Origin(xyz=(0.870, 0.0, z)),
            material=STEEL,
            name=f"guide_shoe_1_{index}",
        )
    lift_plate.visual(
        Cylinder(radius=0.035, length=0.92),
        origin=Origin(xyz=(0.0, -0.10, 1.40)),
        material=STEEL,
        name="lift_stem",
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lift_plate,
        origin=Origin(xyz=(0.0, -0.18, 1.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.08, lower=0.0, upper=0.75),
    )

    housing = model.part("operator_housing")
    housing.visual(
        Box((0.23, 0.34, 0.08)),
        origin=Origin(xyz=(-0.30, 0.0, 0.04)),
        material=STEEL,
        name="foot_pad_0",
    )
    housing.visual(
        Box((0.23, 0.34, 0.08)),
        origin=Origin(xyz=(0.30, 0.0, 0.04)),
        material=STEEL,
        name="foot_pad_1",
    )
    housing.visual(
        Box((0.83, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.13, 0.04)),
        material=STEEL,
        name="rear_base_bridge",
    )
    housing.visual(
        Box((0.18, 0.12, 0.18)),
        origin=Origin(xyz=(0.0, 0.08, 0.15)),
        material=STEEL,
        name="gear_pedestal",
    )
    housing.visual(
        Box((0.56, 0.30, 0.36)),
        origin=Origin(xyz=(0.0, -0.05, 0.39)),
        material=DARK_STEEL,
        name="worm_gear_case",
    )
    housing.visual(
        Cylinder(radius=0.205, length=0.16),
        origin=Origin(xyz=(0.0, -0.30, 0.45), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK_STEEL,
        name="round_gear_cover",
    )
    housing.visual(
        Box((0.34, 0.08, 0.22)),
        origin=Origin(xyz=(0.0, -0.205, 0.45)),
        material=DARK_STEEL,
        name="cover_neck",
    )
    housing.visual(
        Cylinder(radius=0.045, length=0.070),
        origin=Origin(xyz=(0.0, -0.3825, 0.45), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="gearbox_shaft",
    )
    housing.visual(
        Box((0.18, 0.16, 0.08)),
        origin=Origin(xyz=(0.30, -0.25, 0.58)),
        material=STEEL,
        name="pawl_bracket_web",
    )
    housing.visual(
        Box((0.16, 0.19, 0.08)),
        origin=Origin(xyz=(0.42, -0.34, 0.58)),
        material=STEEL,
        name="pawl_bracket",
    )
    housing.visual(
        Cylinder(radius=0.038, length=0.19),
        origin=Origin(xyz=(0.42, -0.500, 0.58), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="pawl_pivot_pin",
    )
    model.articulation(
        "frame_to_operator_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(xyz=(0.0, -0.08, 3.24)),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_mesh(), "cast_handwheel", tolerance=0.002, angular_tolerance=0.15),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLACK,
        name="rim_spokes_hub",
    )
    model.articulation(
        "handwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=handwheel,
        origin=Origin(xyz=(0.0, -0.480, 0.45)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.5),
    )

    pawl = model.part("locking_pawl")
    pawl.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=SAFETY_YELLOW,
        name="pivot_eye",
    )
    pawl.visual(
        Box((0.26, 0.028, 0.050)),
        origin=Origin(xyz=(-0.125, 0.0, -0.030), rpy=(0.0, -0.22, 0.0)),
        material=SAFETY_YELLOW,
        name="pawl_lever",
    )
    pawl.visual(
        Box((0.070, 0.034, 0.070)),
        origin=Origin(xyz=(-0.255, 0.0, -0.065), rpy=(0.0, -0.45, 0.0)),
        material=DARK_STEEL,
        name="locking_tooth",
    )
    model.articulation(
        "pawl_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pawl,
        origin=Origin(xyz=(0.42, -0.610, 0.58)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.55, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    plate = object_model.get_part("lift_plate")
    rail_0 = object_model.get_part("guide_rail_0")
    rail_1 = object_model.get_part("guide_rail_1")
    housing = object_model.get_part("operator_housing")
    handwheel = object_model.get_part("handwheel")
    pawl = object_model.get_part("locking_pawl")
    slide = object_model.get_articulation("gate_slide")
    wheel_joint = object_model.get_articulation("handwheel_spin")
    pawl_joint = object_model.get_articulation("pawl_pivot")

    ctx.check(
        "lift plate is a vertical prismatic gate",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "handwheel is continuous on the gearbox shaft",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.check(
        "locking pawl is a limited lever",
        pawl_joint.articulation_type == ArticulationType.REVOLUTE
        and pawl_joint.motion_limits is not None
        and pawl_joint.motion_limits.lower < 0.0
        and pawl_joint.motion_limits.upper > 0.5,
        details=f"type={pawl_joint.articulation_type}, limits={pawl_joint.motion_limits}",
    )

    ctx.expect_gap(
        frame,
        rail_0,
        axis="y",
        min_gap=0.030,
        positive_elem="side_wall_0",
        negative_elem="guide_channel",
        name="guide rail 0 body stands proud of the wall",
    )
    ctx.expect_gap(
        frame,
        rail_1,
        axis="y",
        min_gap=0.030,
        positive_elem="side_wall_1",
        negative_elem="guide_channel",
        name="guide rail 1 body stands proud of the wall",
    )
    ctx.expect_gap(
        plate,
        rail_0,
        axis="x",
        min_gap=0.035,
        max_gap=0.075,
        positive_elem="plate_skin",
        negative_elem="guide_channel",
        name="plate clears rail 0 slot",
    )
    ctx.expect_gap(
        rail_1,
        plate,
        axis="x",
        min_gap=0.035,
        max_gap=0.075,
        positive_elem="guide_channel",
        negative_elem="plate_skin",
        name="plate clears rail 1 slot",
    )
    ctx.expect_contact(
        housing,
        frame,
        elem_a="foot_pad_0",
        elem_b="top_beam",
        name="operator housing foot pad bears on top beam",
    )
    ctx.expect_contact(
        pawl,
        housing,
        elem_a="pivot_eye",
        elem_b="pawl_pivot_pin",
        contact_tol=0.002,
        name="pawl eye is mounted on its short pivot pin",
    )

    rest_pos = ctx.part_world_position(plate)
    with ctx.pose({slide: 0.75}):
        raised_pos = ctx.part_world_position(plate)
        ctx.expect_overlap(
            plate,
            rail_0,
            axes="z",
            min_overlap=1.0,
            elem_a="plate_skin",
            elem_b="guide_channel",
            name="raised plate remains guided by rail 0",
        )
        ctx.expect_overlap(
            plate,
            rail_1,
            axes="z",
            min_overlap=1.0,
            elem_a="plate_skin",
            elem_b="guide_channel",
            name="raised plate remains guided by rail 1",
        )
    ctx.check(
        "gate slide raises the lift plate",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    wheel_rest = ctx.part_world_position(handwheel)
    with ctx.pose({wheel_joint: math.tau}):
        wheel_turn = ctx.part_world_position(handwheel)
    ctx.check(
        "handwheel rotation stays on the shaft center",
        wheel_rest is not None and wheel_turn is not None and max(abs(a - b) for a, b in zip(wheel_rest, wheel_turn)) < 1e-6,
        details=f"rest={wheel_rest}, turned={wheel_turn}",
    )

    return ctx.report()


object_model = build_object_model()
