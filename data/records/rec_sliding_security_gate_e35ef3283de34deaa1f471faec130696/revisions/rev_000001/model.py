from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_sliding_security_gate")

    model.material("concrete", rgba=(0.45, 0.45, 0.42, 1.0))
    model.material("galvanized_steel", rgba=(0.62, 0.65, 0.63, 1.0))
    model.material("dark_steel", rgba=(0.10, 0.12, 0.12, 1.0))
    model.material("worn_rail", rgba=(0.33, 0.34, 0.33, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.015, 0.014, 1.0))
    model.material("zinc_hardware", rgba=(0.78, 0.76, 0.68, 1.0))
    model.material("hazard_yellow", rgba=(0.95, 0.72, 0.06, 1.0))

    fixed = model.part("fixed_track")
    fixed.visual(
        Box((5.70, 0.55, 0.12)),
        origin=Origin(xyz=(2.50, 0.0, 0.06)),
        material="concrete",
        name="anchor_slab",
    )
    fixed.visual(
        Box((5.50, 0.20, 0.035)),
        origin=Origin(xyz=(2.50, 0.0, 0.1375)),
        material="galvanized_steel",
        name="track_base",
    )
    fixed.visual(
        Cylinder(radius=0.025, length=5.45),
        origin=Origin(xyz=(2.50, 0.0, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material="worn_rail",
        name="round_rail",
    )
    for y, name in ((0.125, "front_mud_channel"), (-0.125, "rear_mud_channel")):
        fixed.visual(
            Box((5.50, 0.035, 0.080)),
            origin=Origin(xyz=(2.50, y, 0.180)),
            material="galvanized_steel",
            name=name,
        )

    for x, name in ((-0.19, "latch_post"), (5.19, "end_post")):
        fixed.visual(
            Box((0.18, 0.28, 2.28)),
            origin=Origin(xyz=(x, 0.0, 1.26)),
            material="dark_steel",
            name=name,
        )
        fixed.visual(
            Box((0.32, 0.38, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.1375)),
            material="zinc_hardware",
            name=f"{name}_foot_plate",
        )

    fixed.visual(
        Box((5.60, 0.24, 0.12)),
        origin=Origin(xyz=(2.50, 0.0, 2.36)),
        material="dark_steel",
        name="top_header",
    )
    fixed.visual(
        Box((5.30, 0.48, 0.050)),
        origin=Origin(xyz=(2.50, 0.0, 2.075)),
        material="galvanized_steel",
        name="guide_channel_roof",
    )
    for y, name in ((0.220, "front_guide_cheek"), (-0.220, "rear_guide_cheek")):
        fixed.visual(
            Box((5.30, 0.035, 0.18)),
            origin=Origin(xyz=(2.50, y, 1.985)),
            material="galvanized_steel",
            name=name,
        )
    for i, x in enumerate((0.60, 2.50, 4.40)):
        fixed.visual(
            Box((0.12, 0.08, 0.30)),
            origin=Origin(xyz=(x, 0.0, 2.205)),
            material="galvanized_steel",
            name=f"guide_hanger_{i}",
        )

    # Serviceable vertical guide rollers are bolted to the top channel and
    # straddle the gate wear strip with visible running clearance.
    fixed.visual(
        Box((0.12, 0.11, 0.045)),
        origin=Origin(xyz=(0.45, 0.1586, 2.020)),
        material="zinc_hardware",
        name="guide_bracket_0",
    )
    fixed.visual(
        Cylinder(radius=0.012, length=0.30),
        origin=Origin(xyz=(0.45, 0.130, 1.910)),
        material="zinc_hardware",
        name="guide_shaft_0",
    )
    fixed.visual(
        Cylinder(radius=0.035, length=0.120),
        origin=Origin(xyz=(0.45, 0.130, 1.910)),
        material="rubber_black",
        name="guide_roller_0",
    )
    fixed.visual(
        Box((0.12, 0.11, 0.045)),
        origin=Origin(xyz=(0.45, -0.1586, 2.020)),
        material="zinc_hardware",
        name="guide_bracket_1",
    )
    fixed.visual(
        Cylinder(radius=0.012, length=0.30),
        origin=Origin(xyz=(0.45, -0.130, 1.910)),
        material="zinc_hardware",
        name="guide_shaft_1",
    )
    fixed.visual(
        Cylinder(radius=0.035, length=0.120),
        origin=Origin(xyz=(0.45, -0.130, 1.910)),
        material="rubber_black",
        name="guide_roller_1",
    )
    fixed.visual(
        Box((0.12, 0.11, 0.045)),
        origin=Origin(xyz=(4.55, 0.1586, 2.020)),
        material="zinc_hardware",
        name="guide_bracket_2",
    )
    fixed.visual(
        Cylinder(radius=0.012, length=0.30),
        origin=Origin(xyz=(4.55, 0.130, 1.910)),
        material="zinc_hardware",
        name="guide_shaft_2",
    )
    fixed.visual(
        Cylinder(radius=0.035, length=0.120),
        origin=Origin(xyz=(4.55, 0.130, 1.910)),
        material="rubber_black",
        name="guide_roller_2",
    )
    fixed.visual(
        Box((0.12, 0.11, 0.045)),
        origin=Origin(xyz=(4.55, -0.1586, 2.020)),
        material="zinc_hardware",
        name="guide_bracket_3",
    )
    fixed.visual(
        Cylinder(radius=0.012, length=0.30),
        origin=Origin(xyz=(4.55, -0.130, 1.910)),
        material="zinc_hardware",
        name="guide_shaft_3",
    )
    fixed.visual(
        Cylinder(radius=0.035, length=0.120),
        origin=Origin(xyz=(4.55, -0.130, 1.910)),
        material="rubber_black",
        name="guide_roller_3",
    )

    fixed.visual(
        Box((0.08, 0.08, 0.20)),
        origin=Origin(xyz=(-0.090, -0.095, 1.02)),
        material="rubber_black",
        name="closed_bumper",
    )
    fixed.visual(
        Box((0.10, 0.08, 0.22)),
        origin=Origin(xyz=(5.16, -0.095, 1.02)),
        material="rubber_black",
        name="open_bumper",
    )
    fixed.visual(
        Box((0.12, 0.035, 0.24)),
        origin=Origin(xyz=(-0.160, 0.1575, 1.25)),
        material="zinc_hardware",
        name="latch_backplate",
    )
    fixed.visual(
        Box((0.16, 0.14, 0.035)),
        origin=Origin(xyz=(-0.100, 0.225, 1.330)),
        material="zinc_hardware",
        name="receiver_upper_jaw",
    )
    fixed.visual(
        Box((0.16, 0.14, 0.035)),
        origin=Origin(xyz=(-0.100, 0.225, 1.170)),
        material="zinc_hardware",
        name="receiver_lower_jaw",
    )

    gate = model.part("gate_leaf")
    gate.visual(
        Box((3.00, 0.10, 0.12)),
        origin=Origin(xyz=(1.50, 0.0, 0.52)),
        material="dark_steel",
        name="bottom_tube",
    )
    gate.visual(
        Box((3.00, 0.10, 0.12)),
        origin=Origin(xyz=(1.50, 0.0, 1.80)),
        material="dark_steel",
        name="top_tube",
    )
    for x, name in ((0.06, "lead_stile"), (2.94, "tail_stile")):
        gate.visual(
            Box((0.12, 0.10, 1.52)),
            origin=Origin(xyz=(x, 0.0, 1.115)),
            material="dark_steel",
            name=name,
        )
    for i, x in enumerate((0.42, 0.72, 1.02, 1.32, 1.62, 1.92, 2.22, 2.52)):
        gate.visual(
            Box((0.050, 0.055, 1.30)),
            origin=Origin(xyz=(x, 0.0, 1.115)),
            material="galvanized_steel",
            name=f"picket_{i}",
        )
    gate.visual(
        Box((2.80, 0.060, 0.060)),
        origin=Origin(xyz=(1.50, 0.0, 1.115)),
        material="galvanized_steel",
        name="service_midrail",
    )
    gate.visual(
        Box((2.98, 0.070, 0.070)),
        origin=Origin(xyz=(1.50, 0.0, 1.115), rpy=(0.0, -0.435, 0.0)),
        material="galvanized_steel",
        name="diagonal_brace",
    )
    gate.visual(
        Box((3.05, 0.150, 0.10)),
        origin=Origin(xyz=(1.50, 0.0, 1.910)),
        material="worn_rail",
        name="top_wear_strip",
    )
    gate.visual(
        Box((0.08, 0.20, 0.22)),
        origin=Origin(xyz=(0.030, 0.140, 1.250)),
        material="zinc_hardware",
        name="latch_mount_plate",
    )
    gate.visual(
        Box((0.20, 0.045, 0.075)),
        origin=Origin(xyz=(-0.080, 0.225, 1.250)),
        material="zinc_hardware",
        name="latch_tongue",
    )
    gate.visual(
        Box((0.050, 0.035, 0.160)),
        origin=Origin(xyz=(0.010, 0.255, 1.250)),
        material="zinc_hardware",
        name="lock_tab",
    )
    gate.visual(
        Cylinder(radius=0.013, length=0.060),
        origin=Origin(xyz=(0.020, 0.255, 1.250), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="lock_hole_boss",
    )

    roller_centers = (0.55, 2.45)
    for i, x in enumerate(roller_centers):
        gate.visual(
            Box((0.32, 0.025, 0.31)),
            origin=Origin(xyz=(x, 0.075, 0.315)),
            material="galvanized_steel",
            name=f"roller_cheek_{i}_0",
        )
        gate.visual(
            Box((0.32, 0.025, 0.31)),
            origin=Origin(xyz=(x, -0.075, 0.315)),
            material="galvanized_steel",
            name=f"roller_cheek_{i}_1",
        )
        gate.visual(
            Box((0.28, 0.13, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.462)),
            material="galvanized_steel",
            name=f"roller_saddle_{i}",
        )
        for side, y in (("front", 0.086), ("rear", -0.086)):
            for level, z in (("low", 0.245), ("high", 0.355)):
                gate.visual(
                    Cylinder(radius=0.018, length=0.024),
                    origin=Origin(xyz=(x - 0.095, y, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                    material="zinc_hardware",
                    name=f"roller_bolt_{i}_{side}_{level}_0",
                )
                gate.visual(
                    Cylinder(radius=0.018, length=0.024),
                    origin=Origin(xyz=(x + 0.095, y, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                    material="zinc_hardware",
                    name=f"roller_bolt_{i}_{side}_{level}_1",
                )
        gate.visual(
            Box((0.18, 0.020, 0.070)),
            origin=Origin(xyz=(x, 0.060, 0.520)),
            material="zinc_hardware",
            name=f"access_cover_{i}",
        )

    gate.visual(
        Cylinder(radius=0.018, length=0.210),
        origin=Origin(xyz=(0.55, 0.0, 0.315), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="zinc_hardware",
        name="roller_axle_0",
    )
    gate.visual(
        Cylinder(radius=0.018, length=0.210),
        origin=Origin(xyz=(2.45, 0.0, 0.315), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="zinc_hardware",
        name="roller_axle_1",
    )

    slide = model.articulation(
        "track_slide",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=gate,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.40, lower=0.0, upper=2.10),
        motion_properties=MotionProperties(damping=40.0, friction=8.0),
    )
    slide.meta["description"] = "Full leaf travel along the anchored round rail and upper guide channel."

    for i, x in enumerate(roller_centers):
        roller = model.part(f"lower_roller_{i}")
        roller.visual(
            Cylinder(radius=0.110, length=0.070),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material="rubber_black",
            name="roller_tread",
        )
        roller.visual(
            Cylinder(radius=0.048, length=0.090),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material="zinc_hardware",
            name="roller_hub",
        )
        roller.visual(
            Cylinder(radius=0.120, length=0.012),
            origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="zinc_hardware",
            name="front_flange",
        )
        roller.visual(
            Cylinder(radius=0.120, length=0.012),
            origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="zinc_hardware",
            name="rear_flange",
        )
        model.articulation(
            f"roller_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=gate,
            child=roller,
            origin=Origin(xyz=(x, 0.0, 0.315)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=8.0),
            motion_properties=MotionProperties(damping=0.05, friction=0.02),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_track")
    gate = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("track_slide")
    roller_0 = object_model.get_part("lower_roller_0")
    roller_1 = object_model.get_part("lower_roller_1")

    ctx.allow_overlap(
        gate,
        roller_0,
        elem_a="roller_axle_0",
        elem_b="roller_hub",
        reason="The replaceable lower wheel hub is represented as rotating around a captured axle shaft.",
    )
    ctx.allow_overlap(
        gate,
        roller_0,
        elem_a="roller_axle_0",
        elem_b="roller_tread",
        reason="The solid roller proxy contains the captured axle bore at the wheel center.",
    )
    ctx.allow_overlap(
        gate,
        roller_0,
        elem_a="roller_axle_0",
        elem_b="front_flange",
        reason="The serviceable wheel flange is clamped around the captured axle shaft.",
    )
    ctx.allow_overlap(
        gate,
        roller_0,
        elem_a="roller_axle_0",
        elem_b="rear_flange",
        reason="The serviceable wheel flange is clamped around the captured axle shaft.",
    )
    ctx.allow_overlap(
        gate,
        roller_1,
        elem_a="roller_axle_1",
        elem_b="roller_hub",
        reason="The replaceable lower wheel hub is represented as rotating around a captured axle shaft.",
    )
    ctx.allow_overlap(
        gate,
        roller_1,
        elem_a="roller_axle_1",
        elem_b="roller_tread",
        reason="The solid roller proxy contains the captured axle bore at the wheel center.",
    )
    ctx.allow_overlap(
        gate,
        roller_1,
        elem_a="roller_axle_1",
        elem_b="front_flange",
        reason="The serviceable wheel flange is clamped around the captured axle shaft.",
    )
    ctx.allow_overlap(
        gate,
        roller_1,
        elem_a="roller_axle_1",
        elem_b="rear_flange",
        reason="The serviceable wheel flange is clamped around the captured axle shaft.",
    )
    ctx.expect_overlap(
        gate,
        roller_0,
        axes="y",
        min_overlap=0.070,
        elem_a="roller_axle_0",
        elem_b="roller_hub",
        name="front roller hub is captured on axle",
    )
    ctx.expect_overlap(
        gate,
        roller_1,
        axes="y",
        min_overlap=0.070,
        elem_a="roller_axle_1",
        elem_b="roller_hub",
        name="rear roller hub is captured on axle",
    )
    ctx.expect_overlap(
        gate,
        roller_0,
        axes="y",
        min_overlap=0.010,
        elem_a="roller_axle_0",
        elem_b="front_flange",
        name="front wheel flange is retained on axle",
    )
    ctx.expect_overlap(
        gate,
        roller_1,
        axes="y",
        min_overlap=0.010,
        elem_a="roller_axle_1",
        elem_b="front_flange",
        name="rear wheel flange is retained on axle",
    )

    ctx.expect_gap(
        roller_0,
        fixed,
        axis="z",
        min_gap=-0.001,
        max_gap=0.003,
        positive_elem="roller_tread",
        negative_elem="round_rail",
        name="front lower roller rides on rail",
    )
    ctx.expect_gap(
        roller_1,
        fixed,
        axis="z",
        min_gap=-0.001,
        max_gap=0.003,
        positive_elem="roller_tread",
        negative_elem="round_rail",
        name="rear lower roller rides on rail",
    )
    ctx.expect_gap(
        fixed,
        gate,
        axis="y",
        min_gap=0.006,
        max_gap=0.040,
        positive_elem="guide_roller_0",
        negative_elem="top_wear_strip",
        name="front guide roller clears wear strip",
    )
    ctx.expect_gap(
        gate,
        fixed,
        axis="y",
        min_gap=0.006,
        max_gap=0.040,
        positive_elem="top_wear_strip",
        negative_elem="guide_roller_1",
        name="rear guide roller clears wear strip",
    )
    ctx.expect_overlap(
        gate,
        fixed,
        axes="xy",
        min_overlap=0.02,
        elem_a="latch_tongue",
        elem_b="receiver_upper_jaw",
        name="latch tongue aligns with receiver",
    )
    ctx.expect_gap(
        fixed,
        gate,
        axis="z",
        min_gap=0.015,
        max_gap=0.040,
        positive_elem="receiver_upper_jaw",
        negative_elem="latch_tongue",
        name="upper latch jaw clears tongue",
    )
    ctx.expect_gap(
        gate,
        fixed,
        axis="z",
        min_gap=0.015,
        max_gap=0.040,
        positive_elem="latch_tongue",
        negative_elem="receiver_lower_jaw",
        name="lower latch jaw clears tongue",
    )

    closed_pos = ctx.part_world_position(gate)
    with ctx.pose({slide: 2.10}):
        ctx.expect_within(
            roller_0,
            fixed,
            axes="x",
            margin=0.0,
            inner_elem="roller_tread",
            outer_elem="round_rail",
            name="front roller stays on rail at full travel",
        )
        ctx.expect_within(
            roller_1,
            fixed,
            axes="x",
            margin=0.0,
            inner_elem="roller_tread",
            outer_elem="round_rail",
            name="rear roller stays on rail at full travel",
        )
        ctx.expect_gap(
            fixed,
            gate,
            axis="y",
            min_gap=0.006,
            max_gap=0.040,
            positive_elem="guide_roller_2",
            negative_elem="top_wear_strip",
            name="open front guide roller clearance",
        )
        ctx.expect_gap(
            gate,
            fixed,
            axis="y",
            min_gap=0.006,
            max_gap=0.040,
            positive_elem="top_wear_strip",
            negative_elem="guide_roller_3",
            name="open rear guide roller clearance",
        )
        open_pos = ctx.part_world_position(gate)

    ctx.check(
        "gate slides in opening direction",
        closed_pos is not None and open_pos is not None and open_pos[0] > closed_pos[0] + 2.0,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
