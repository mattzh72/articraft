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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_windshield_sun_visor")

    safety_yellow = model.material("safety_yellow", rgba=(0.96, 0.72, 0.05, 1.0))
    black = model.material("black_powdercoat", rgba=(0.03, 0.035, 0.035, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    bolt_steel = model.material("zinc_fasteners", rgba=(0.62, 0.62, 0.58, 1.0))
    smoked_poly = model.material("smoked_polycarbonate", rgba=(0.08, 0.12, 0.14, 0.42))

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((0.70, 0.12, 0.030)),
        origin=Origin(xyz=(0.22, -0.080, 0.065)),
        material=black,
        name="roof_bolt_plate",
    )
    roof_mount.visual(
        Box((0.36, 0.026, 0.075)),
        origin=Origin(xyz=(0.0, -0.038, 0.025)),
        material=black,
        name="hinge_back_web",
    )
    roof_mount.visual(
        Cylinder(radius=0.027, length=0.072),
        origin=Origin(xyz=(-0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="primary_fixed_knuckle_0",
    )
    roof_mount.visual(
        Cylinder(radius=0.027, length=0.072),
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="primary_fixed_knuckle_1",
    )
    for x in (-0.131, 0.131):
        roof_mount.visual(
            Cylinder(radius=0.034, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"primary_pin_cap_{0 if x < 0 else 1}",
        )
    roof_mount.visual(
        Cylinder(radius=0.009, length=0.290),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_steel,
        name="primary_hinge_pin",
    )
    roof_mount.visual(
        Box((0.36, 0.044, 0.012)),
        origin=Origin(xyz=(0.0, 0.043, 0.052)),
        material=safety_yellow,
        name="primary_guard_hood",
    )
    for x in (-0.170, 0.170):
        roof_mount.visual(
            Box((0.016, 0.060, 0.072)),
            origin=Origin(xyz=(x, -0.005, 0.025)),
            material=safety_yellow,
            name=f"guard_strut_{0 if x < 0 else 1}",
        )
    roof_mount.visual(
        Box((0.060, 0.030, 0.045)),
        origin=Origin(xyz=(-0.170, 0.065, -0.022)),
        material=safety_yellow,
        name="primary_stop_block",
    )
    roof_mount.visual(
        Box((0.020, 0.105, 0.030)),
        origin=Origin(xyz=(-0.170, 0.020, -0.012)),
        material=safety_yellow,
        name="primary_stop_bridge",
    )
    for i, x in enumerate((-0.060, 0.100, 0.260, 0.420)):
        roof_mount.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(x, -0.080, 0.083)),
            material=bolt_steel,
            name=f"roof_bolt_{i}",
        )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.024, length=0.080),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="primary_center_knuckle",
    )
    hinge_arm.visual(
        Box((0.052, 0.040, 0.140)),
        origin=Origin(xyz=(0.0, 0.020, -0.093)),
        material=dark_steel,
        name="drop_web",
    )
    hinge_arm.visual(
        Box((0.056, 0.100, 0.045)),
        origin=Origin(xyz=(0.0, 0.075, -0.160)),
        material=dark_steel,
        name="forward_arm_box",
    )
    hinge_arm.visual(
        Box((0.038, 0.185, 0.018)),
        origin=Origin(xyz=(0.0, 0.077, -0.085), rpy=(-0.66, 0.0, 0.0)),
        material=safety_yellow,
        name="diagonal_tension_brace",
    )
    hinge_arm.visual(
        Box((0.205, 0.018, 0.018)),
        origin=Origin(xyz=(-0.072, 0.028, -0.045)),
        material=safety_yellow,
        name="primary_stop_tab",
    )
    hinge_arm.visual(
        Cylinder(radius=0.042, length=0.038),
        origin=Origin(xyz=(0.0, 0.160, -0.122)),
        material=dark_steel,
        name="secondary_upper_bearing",
    )
    hinge_arm.visual(
        Box((0.105, 0.080, 0.014)),
        origin=Origin(xyz=(0.0, 0.160, -0.102)),
        material=safety_yellow,
        name="secondary_guard_plate",
    )
    hinge_arm.visual(
        Box((0.112, 0.008, 0.070)),
        origin=Origin(xyz=(-0.067, 0.125, -0.110)),
        material=safety_yellow,
        name="secondary_lockout_quadrant",
    )
    for i, z in enumerate((-0.135, -0.113, -0.091)):
        hinge_arm.visual(
            Cylinder(radius=0.0065, length=0.014),
            origin=Origin(xyz=(-0.104, 0.119, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"lockout_index_hole_{i}",
        )
    hinge_arm.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(xyz=(-0.068, 0.145, -0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_steel,
        name="spring_lock_pin",
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        Box((0.500, 0.010, 0.210)),
        origin=Origin(xyz=(0.300, 0.0, -0.178)),
        material=smoked_poly,
        name="smoked_shield",
    )
    visor_panel.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(),
        material=dark_steel,
        name="secondary_center_knuckle",
    )
    visor_panel.visual(
        Box((0.052, 0.020, 0.075)),
        origin=Origin(xyz=(0.026, 0.0, -0.034)),
        material=dark_steel,
        name="pivot_neck_plate",
    )
    visor_panel.visual(
        Box((0.590, 0.026, 0.030)),
        origin=Origin(xyz=(0.315, 0.0, -0.060)),
        material=black,
        name="top_frame",
    )
    visor_panel.visual(
        Box((0.590, 0.026, 0.030)),
        origin=Origin(xyz=(0.315, 0.0, -0.300)),
        material=black,
        name="bottom_frame",
    )
    for x, nm in ((0.035, "inner_side_frame"), (0.595, "outer_side_frame")):
        visor_panel.visual(
            Box((0.042, 0.026, 0.270)),
            origin=Origin(xyz=(x, 0.0, -0.180)),
            material=black,
            name=nm,
        )
    visor_panel.visual(
        Box((0.530, 0.018, 0.018)),
        origin=Origin(xyz=(0.325, -0.010, -0.178), rpy=(0.0, 0.34, 0.0)),
        material=safety_yellow,
        name="diagonal_panel_brace",
    )
    visor_panel.visual(
        Box((0.110, 0.030, 0.040)),
        origin=Origin(xyz=(0.070, 0.0, -0.085)),
        material=safety_yellow,
        name="secondary_stop_lug",
    )
    for i, (x, z) in enumerate(
        ((0.095, -0.060), (0.215, -0.060), (0.435, -0.060), (0.555, -0.060),
         (0.095, -0.300), (0.215, -0.300), (0.435, -0.300), (0.555, -0.300))
    ):
        visor_panel.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(xyz=(x, -0.018, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt_steel,
            name=f"frame_bolt_{i}",
        )

    model.articulation(
        "roof_to_arm",
        ArticulationType.REVOLUTE,
        parent=roof_mount,
        child=hinge_arm,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.2, lower=0.0, upper=1.15),
    )
    model.articulation(
        "arm_to_panel",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.0, 0.160, -0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=38.0, velocity=1.0, lower=-0.08, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_mount")
    arm = object_model.get_part("hinge_arm")
    panel = object_model.get_part("visor_panel")
    roof_hinge = object_model.get_articulation("roof_to_arm")
    swing = object_model.get_articulation("arm_to_panel")

    ctx.allow_overlap(
        roof,
        arm,
        elem_a="primary_hinge_pin",
        elem_b="primary_center_knuckle",
        reason="The steel hinge pin intentionally passes through the rotating center knuckle.",
    )
    ctx.expect_within(
        roof,
        arm,
        axes="yz",
        inner_elem="primary_hinge_pin",
        outer_elem="primary_center_knuckle",
        margin=0.0,
        name="primary hinge pin is captured inside the rotating knuckle bore",
    )
    ctx.expect_overlap(
        roof,
        arm,
        axes="x",
        elem_a="primary_hinge_pin",
        elem_b="primary_center_knuckle",
        min_overlap=0.070,
        name="primary hinge pin spans the center knuckle",
    )
    ctx.expect_overlap(
        roof,
        arm,
        axes="yz",
        elem_a="primary_fixed_knuckle_0",
        elem_b="primary_center_knuckle",
        min_overlap=0.035,
        name="primary hinge knuckles share the hinge axis region",
    )
    ctx.expect_gap(
        arm,
        roof,
        axis="x",
        positive_elem="primary_center_knuckle",
        negative_elem="primary_fixed_knuckle_0",
        min_gap=0.004,
        max_gap=0.025,
        name="left primary knuckle has axial running clearance",
    )
    ctx.expect_gap(
        roof,
        arm,
        axis="x",
        positive_elem="primary_fixed_knuckle_1",
        negative_elem="primary_center_knuckle",
        min_gap=0.004,
        max_gap=0.025,
        name="right primary knuckle has axial running clearance",
    )
    ctx.expect_gap(
        arm,
        panel,
        axis="z",
        positive_elem="secondary_upper_bearing",
        negative_elem="secondary_center_knuckle",
        min_gap=0.002,
        max_gap=0.012,
        name="secondary pivot bearing clears the rotating knuckle",
    )
    ctx.expect_overlap(
        arm,
        panel,
        axes="xy",
        elem_a="secondary_upper_bearing",
        elem_b="secondary_center_knuckle",
        min_overlap=0.030,
        name="secondary pivot bearing is concentric with panel knuckle",
    )

    rest = ctx.part_world_position(panel)
    with ctx.pose({roof_hinge: 1.0}):
        raised = ctx.part_world_position(panel)
    ctx.check(
        "roof hinge raises visor assembly",
        rest is not None and raised is not None and raised[2] > rest[2] + 0.11,
        details=f"rest={rest}, raised={raised}",
    )

    rest_box = ctx.part_world_aabb(panel)
    with ctx.pose({swing: 1.1}):
        swung_box = ctx.part_world_aabb(panel)
    rest_center = None
    swung_center = None
    if rest_box is not None:
        rest_center = tuple((rest_box[0][i] + rest_box[1][i]) * 0.5 for i in range(3))
    if swung_box is not None:
        swung_center = tuple((swung_box[0][i] + swung_box[1][i]) * 0.5 for i in range(3))
    ctx.check(
        "secondary pivot swings visor sideways",
        rest_center is not None
        and swung_center is not None
        and abs(swung_center[0] - rest_center[0]) > 0.12,
        details=f"rest={rest_center}, swung={swung_center}",
    )

    return ctx.report()


object_model = build_object_model()
