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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_windshield_sun_visor")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.015, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.037, 0.038, 1.0))
    anodized = model.material("anodized_aluminum", rgba=(0.58, 0.60, 0.58, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    datum_blue = model.material("datum_blue", rgba=(0.08, 0.18, 0.34, 1.0))
    amber = model.material("smoke_amber_polycarbonate", rgba=(0.55, 0.38, 0.12, 0.58))
    white = model.material("white_index_fill", rgba=(0.92, 0.92, 0.86, 1.0))
    red = model.material("red_zero_pointer", rgba=(0.85, 0.06, 0.03, 1.0))

    # Root part: the machined roof datum bracket and fixed primary hinge pieces.
    roof = model.part("roof_mount")
    roof.visual(
        Box((0.185, 0.082, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=datum_blue,
        name="roof_datum_plate",
    )
    roof.visual(
        Box((0.070, 0.026, 0.058)),
        origin=Origin(xyz=(0.0, -0.001, 0.027)),
        material=datum_blue,
        name="hinge_pedestal",
    )
    roof.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="fixed_barrel",
    )
    roof.visual(
        Cylinder(radius=0.004, length=0.168),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="primary_pin",
    )
    roof.visual(
        Box((0.204, 0.008, 0.009)),
        origin=Origin(xyz=(0.0, -0.040, 0.056)),
        material=brushed,
        name="front_datum_rail",
    )
    roof.visual(
        Box((0.150, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.042, 0.052)),
        material=brushed,
        name="rear_gap_gauge_land",
    )
    for i, x in enumerate((-0.062, 0.062)):
        roof.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(x, 0.020, 0.062)),
            material=matte_black,
            name=f"mount_screw_{i}",
        )
    for i, x in enumerate((-0.074, -0.050, -0.026, 0.0, 0.026, 0.050, 0.074)):
        height = 0.014 if i == 3 else 0.009
        roof.visual(
            Box((0.0022, 0.0035, height)),
            origin=Origin(xyz=(x, -0.044, 0.060 + 0.5 * height)),
            material=white if i != 3 else red,
            name=f"roof_index_{i}",
        )

    # Moving primary hinge leaf and calibrated offset arm.  The secondary pivot
    # spindle at the outboard end is deliberately visible.
    arm = model.part("hinge_arm")
    arm.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(-0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="moving_knuckle_0",
    )
    arm.visual(
        Box((0.030, 0.018, 0.024)),
        origin=Origin(xyz=(-0.056, -0.011, -0.018)),
        material=anodized,
        name="knuckle_lug_0",
    )
    arm.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="moving_knuckle_1",
    )
    arm.visual(
        Box((0.030, 0.018, 0.024)),
        origin=Origin(xyz=(0.056, -0.011, -0.018)),
        material=anodized,
        name="knuckle_lug_1",
    )
    arm.visual(
        Box((0.150, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.023, -0.034)),
        material=anodized,
        name="lower_hinge_bridge",
    )
    arm.visual(
        Box((0.232, 0.016, 0.012)),
        origin=Origin(xyz=(-0.096, -0.028, -0.045)),
        material=satin_black,
        name="calibration_arm",
    )
    arm.visual(
        Box((0.092, 0.004, 0.018)),
        origin=Origin(xyz=(-0.090, -0.038, -0.043)),
        material=brushed,
        name="machined_datum_flat",
    )
    arm.visual(
        Box((0.044, 0.030, 0.020)),
        origin=Origin(xyz=(-0.205, -0.028, -0.051)),
        material=anodized,
        name="secondary_socket",
    )
    arm.visual(
        Cylinder(radius=0.006, length=0.058),
        origin=Origin(xyz=(-0.205, -0.028, -0.082)),
        material=brushed,
        name="secondary_spindle",
    )
    arm.visual(
        Box((0.012, 0.006, 0.018)),
        origin=Origin(xyz=(-0.156, -0.039, -0.035)),
        material=red,
        name="arm_zero_pointer",
    )
    for i, x in enumerate((-0.132, -0.112, -0.092, -0.072, -0.052)):
        arm.visual(
            Box((0.002, 0.003, 0.010 if i == 2 else 0.007)),
            origin=Origin(xyz=(x, -0.041, -0.032)),
            material=white,
            name=f"arm_index_{i}",
        )

    # The panel frame uses the secondary pivot as its part frame.  Its datum
    # features are raised/embedded on the panel so they remain physically tied
    # to the visor rather than reading as free labels.
    panel = model.part("visor_panel")
    panel.visual(
        Box((0.400, 0.010, 0.150)),
        origin=Origin(xyz=(0.214, 0.0, -0.105)),
        material=amber,
        name="shade_panel",
    )
    panel.visual(
        Box((0.404, 0.014, 0.016)),
        origin=Origin(xyz=(0.216, 0.0, -0.030)),
        material=satin_black,
        name="top_edge_frame",
    )
    panel.visual(
        Box((0.404, 0.014, 0.014)),
        origin=Origin(xyz=(0.216, 0.0, -0.180)),
        material=satin_black,
        name="lower_edge_frame",
    )
    panel.visual(
        Box((0.018, 0.016, 0.132)),
        origin=Origin(xyz=(0.412, 0.0, -0.105)),
        material=brushed,
        name="datum_edge_land",
    )
    panel.visual(
        Box((0.028, 0.012, 0.014)),
        origin=Origin(xyz=(0.026, 0.0, -0.026)),
        material=anodized,
        name="pivot_leaf",
    )
    panel.visual(
        Cylinder(radius=0.016, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=anodized,
        name="pivot_boss",
    )
    panel.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=brushed,
        name="thrust_washer",
    )
    for i, x in enumerate((0.080, 0.130, 0.180, 0.230, 0.280, 0.330)):
        panel.visual(
            Box((0.003, 0.002, 0.016 if i in (0, 5) else 0.010)),
            origin=Origin(xyz=(x, -0.006, -0.033)),
            material=white,
            name=f"panel_index_{i}",
        )
    panel.visual(
        Box((0.056, 0.004, 0.010)),
        origin=Origin(xyz=(0.210, -0.007, -0.060)),
        material=white,
        name="alignment_window",
    )

    # A separately articulated lock knob gives the adjustment feature its own
    # explicit rotary kinematics while remaining seated on the secondary socket.
    knob = model.part("lock_knob")
    knob.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=matte_black,
        name="knurled_cap",
    )
    for i, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
        knob.visual(
            Box((0.044, 0.003, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.048), rpy=(0.0, 0.0, angle)),
            material=satin_black,
            name=f"grip_bar_{i}",
        )
    knob.visual(
        Box((0.030, 0.003, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=white,
        name="knob_pointer",
    )

    primary = model.articulation(
        "roof_to_arm",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.25, upper=1.20),
    )
    secondary = model.articulation(
        "arm_to_panel",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=panel,
        origin=Origin(xyz=(-0.205, -0.028, -0.082)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.08, upper=1.35),
    )
    model.articulation(
        "arm_to_knob",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=knob,
        origin=Origin(xyz=(-0.205, -0.028, -0.082)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0),
    )

    model.meta["description"] = (
        "Calibration-oriented windshield sun visor with datum rails, index marks, "
        "controlled hinge gaps, a roof pitch hinge, secondary swing pivot, and "
        "separate lock-knob rotation."
    )
    # Keep references live for linters and to document the intended mechanisms.
    _ = (primary, secondary)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_mount")
    arm = object_model.get_part("hinge_arm")
    panel = object_model.get_part("visor_panel")
    knob = object_model.get_part("lock_knob")
    primary = object_model.get_articulation("roof_to_arm")
    secondary = object_model.get_articulation("arm_to_panel")

    ctx.allow_overlap(
        roof,
        arm,
        elem_a="primary_pin",
        elem_b="moving_knuckle_0",
        reason="The fixed hinge pin is intentionally captured inside the moving primary knuckle.",
    )
    ctx.allow_overlap(
        roof,
        arm,
        elem_a="primary_pin",
        elem_b="moving_knuckle_1",
        reason="The fixed hinge pin is intentionally captured inside the moving primary knuckle.",
    )
    ctx.allow_overlap(
        arm,
        panel,
        elem_a="secondary_spindle",
        elem_b="pivot_boss",
        reason="The secondary spindle is intentionally seated through the visor pivot boss.",
    )
    ctx.allow_overlap(
        arm,
        panel,
        elem_a="secondary_spindle",
        elem_b="thrust_washer",
        reason="The secondary spindle intentionally passes through the captured thrust washer.",
    )
    ctx.allow_overlap(
        arm,
        panel,
        elem_a="secondary_socket",
        elem_b="thrust_washer",
        reason="The washer is intentionally captured in the shallow counterbore of the secondary socket.",
    )

    ctx.expect_within(
        roof,
        arm,
        axes="yz",
        inner_elem="primary_pin",
        outer_elem="moving_knuckle_0",
        margin=0.001,
        name="primary pin centered in knuckle 0",
    )
    ctx.expect_within(
        roof,
        arm,
        axes="yz",
        inner_elem="primary_pin",
        outer_elem="moving_knuckle_1",
        margin=0.001,
        name="primary pin centered in knuckle 1",
    )
    ctx.expect_overlap(
        roof,
        arm,
        axes="x",
        elem_a="primary_pin",
        elem_b="moving_knuckle_0",
        min_overlap=0.026,
        name="primary pin retained in knuckle 0",
    )
    ctx.expect_overlap(
        roof,
        arm,
        axes="x",
        elem_a="primary_pin",
        elem_b="moving_knuckle_1",
        min_overlap=0.026,
        name="primary pin retained in knuckle 1",
    )
    ctx.expect_gap(
        arm,
        roof,
        axis="x",
        positive_elem="moving_knuckle_1",
        negative_elem="fixed_barrel",
        min_gap=0.003,
        max_gap=0.010,
        name="right primary barrel running gap",
    )
    ctx.expect_gap(
        roof,
        arm,
        axis="x",
        positive_elem="fixed_barrel",
        negative_elem="moving_knuckle_0",
        min_gap=0.003,
        max_gap=0.010,
        name="left primary barrel running gap",
    )
    ctx.expect_within(
        arm,
        panel,
        axes="xy",
        inner_elem="secondary_spindle",
        outer_elem="pivot_boss",
        margin=0.001,
        name="secondary spindle centered in boss",
    )
    ctx.expect_overlap(
        arm,
        panel,
        axes="z",
        elem_a="secondary_spindle",
        elem_b="pivot_boss",
        min_overlap=0.035,
        name="secondary spindle retained through boss",
    )
    ctx.expect_within(
        arm,
        panel,
        axes="xy",
        inner_elem="secondary_spindle",
        outer_elem="thrust_washer",
        margin=0.001,
        name="secondary spindle centered in washer",
    )
    ctx.expect_overlap(
        arm,
        panel,
        axes="z",
        elem_a="secondary_spindle",
        elem_b="thrust_washer",
        min_overlap=0.004,
        name="secondary spindle passes through washer",
    )
    ctx.expect_overlap(
        panel,
        arm,
        axes="xy",
        elem_a="thrust_washer",
        elem_b="secondary_socket",
        min_overlap=0.028,
        name="washer registered under socket land",
    )
    ctx.expect_overlap(
        arm,
        panel,
        axes="z",
        elem_a="secondary_socket",
        elem_b="thrust_washer",
        min_overlap=0.004,
        name="washer captured by socket counterbore",
    )
    ctx.expect_contact(
        knob,
        arm,
        elem_a="knurled_cap",
        elem_b="secondary_socket",
        contact_tol=0.001,
        name="lock knob seated on secondary socket",
    )

    rest_panel = ctx.part_world_aabb(panel)
    with ctx.pose({primary: 0.85}):
        stowed_panel = ctx.part_world_aabb(panel)
    ctx.check(
        "primary hinge raises visor toward roof",
        rest_panel is not None
        and stowed_panel is not None
        and stowed_panel[1][2] > rest_panel[1][2] + 0.015,
        details=f"rest={rest_panel}, stowed={stowed_panel}",
    )

    rest_position = ctx.part_world_aabb(panel)
    with ctx.pose({secondary: 0.90}):
        swung_position = ctx.part_world_aabb(panel)
    ctx.check(
        "secondary pivot swings visor laterally",
        rest_position is not None
        and swung_position is not None
        and swung_position[1][1] > rest_position[1][1] + 0.100,
        details=f"rest={rest_position}, swung={swung_position}",
    )

    return ctx.report()


object_model = build_object_model()
