from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_sliding_security_gate")

    galvanized = Material("galvanized_zinc", rgba=(0.55, 0.58, 0.58, 1.0))
    dark_steel = Material("blackened_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    concrete = Material("sealed_concrete", rgba=(0.48, 0.48, 0.44, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.015, 1.0))
    datum_blue = Material("datum_blue", rgba=(0.05, 0.20, 0.85, 1.0))
    white = Material("etched_white", rgba=(0.92, 0.92, 0.86, 1.0))
    yellow = Material("adjustment_yellow", rgba=(0.95, 0.72, 0.08, 1.0))

    fixed_frame = model.part("fixed_frame")
    fixed_frame.visual(
        Box((5.00, 0.52, 0.10)),
        origin=Origin(xyz=(0.65, 0.0, 0.05)),
        material=concrete,
        name="level_concrete_pad",
    )
    fixed_frame.visual(
        Box((4.85, 0.055, 0.045)),
        origin=Origin(xyz=(0.65, 0.0, 0.1225)),
        material=galvanized,
        name="lower_rail",
    )
    fixed_frame.visual(
        Box((4.85, 0.018, 0.09)),
        origin=Origin(xyz=(0.65, -0.060, 0.165)),
        material=dark_steel,
        name="front_channel_lip",
    )
    fixed_frame.visual(
        Box((4.85, 0.018, 0.09)),
        origin=Origin(xyz=(0.65, 0.060, 0.165)),
        material=dark_steel,
        name="rear_channel_lip",
    )
    fixed_frame.visual(
        Box((0.12, 0.18, 2.12)),
        origin=Origin(xyz=(-1.56, 0.0, 1.12)),
        material=galvanized,
        name="latch_post",
    )
    fixed_frame.visual(
        Box((0.12, 0.18, 2.12)),
        origin=Origin(xyz=(2.86, 0.0, 1.12)),
        material=galvanized,
        name="end_guide_post",
    )
    fixed_frame.visual(
        Box((4.50, 0.16, 0.06)),
        origin=Origin(xyz=(0.65, 0.0, 2.08)),
        material=galvanized,
        name="top_channel_cap",
    )
    fixed_frame.visual(
        Box((4.50, 0.020, 0.12)),
        origin=Origin(xyz=(0.65, -0.082, 2.00)),
        material=dark_steel,
        name="front_top_channel_lip",
    )
    fixed_frame.visual(
        Box((4.50, 0.020, 0.12)),
        origin=Origin(xyz=(0.65, 0.082, 2.00)),
        material=dark_steel,
        name="rear_top_channel_lip",
    )

    # End stops establish the controlled closed and open travel gaps.
    fixed_frame.visual(
        Box((0.040, 0.10, 0.24)),
        origin=Origin(xyz=(-1.36, 0.0, 0.24)),
        material=rubber,
        name="closed_stop_pad",
    )
    fixed_frame.visual(
        Box((0.040, 0.10, 0.24)),
        origin=Origin(xyz=(2.55, 0.0, 0.24)),
        material=rubber,
        name="open_stop_pad",
    )

    # Latch receiver: an open U-shaped mouth with a flat blue datum backplate.
    fixed_frame.visual(
        Box((0.030, 0.10, 0.14)),
        origin=Origin(xyz=(-1.515, -0.065, 1.10)),
        material=datum_blue,
        name="receiver_backplate",
    )
    fixed_frame.visual(
        Box((0.160, 0.10, 0.020)),
        origin=Origin(xyz=(-1.420, -0.065, 1.145)),
        material=galvanized,
        name="receiver_upper_jaw",
    )
    fixed_frame.visual(
        Box((0.160, 0.10, 0.020)),
        origin=Origin(xyz=(-1.420, -0.065, 1.055)),
        material=galvanized,
        name="receiver_lower_jaw",
    )
    fixed_frame.visual(
        Box((0.012, 0.11, 0.11)),
        origin=Origin(xyz=(-1.340, -0.065, 1.10)),
        material=yellow,
        name="receiver_mouth_datum",
    )

    # Adjustable top guide rollers and their brackets constrain side wander.
    for x in (-0.75, 1.65):
        for y, side in ((-0.060, "front"), (0.060, "rear")):
            fixed_frame.visual(
                Cylinder(radius=0.025, length=0.28),
                origin=Origin(xyz=(x, y, 1.78)),
                material=rubber,
                name=f"{side}_guide_roller_{0 if x < 0 else 1}",
            )
            fixed_frame.visual(
                Box((0.045, 0.040, 0.20)),
                origin=Origin(xyz=(x, y, 1.985)),
                material=galvanized,
                name=f"{side}_guide_bracket_{0 if x < 0 else 1}",
            )
            fixed_frame.visual(
                Cylinder(radius=0.012, length=0.016),
                origin=Origin(xyz=(x, y - 0.022 if y < 0 else y + 0.022, 1.965), rpy=(pi / 2, 0.0, 0.0)),
                material=yellow,
                name=f"{side}_guide_adjuster_{0 if x < 0 else 1}",
            )

    # Scale and index marks are intentionally seated into the pad/rail.
    for i, x in enumerate((-1.10, -0.70, -0.30, 0.10, 0.50, 0.90, 1.30, 1.70, 2.10, 2.50)):
        major = i % 2 == 0
        fixed_frame.visual(
            Box((0.010, 0.14 if major else 0.09, 0.006)),
            origin=Origin(xyz=(x, -0.195, 0.101)),
            material=white,
            name=f"track_index_{i}",
        )
    fixed_frame.visual(
        Box((0.030, 0.16, 0.010)),
        origin=Origin(xyz=(-1.30, -0.195, 0.102)),
        material=datum_blue,
        name="closed_zero_mark",
    )
    fixed_frame.visual(
        Box((0.030, 0.16, 0.010)),
        origin=Origin(xyz=(2.50, -0.195, 0.102)),
        material=datum_blue,
        name="open_limit_mark",
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((2.62, 0.065, 0.115)),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=galvanized,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((2.62, 0.065, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 1.86)),
        material=galvanized,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((0.085, 0.070, 1.64)),
        origin=Origin(xyz=(-1.285, 0.0, 1.10)),
        material=galvanized,
        name="latch_stile",
    )
    gate_leaf.visual(
        Box((0.085, 0.070, 1.64)),
        origin=Origin(xyz=(1.285, 0.0, 1.10)),
        material=galvanized,
        name="end_stile",
    )
    gate_leaf.visual(
        Box((2.50, 0.050, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        material=galvanized,
        name="mid_rail",
    )
    for i, x in enumerate((-0.98, -0.70, -0.42, -0.14, 0.14, 0.42, 0.70, 0.98)):
        gate_leaf.visual(
            Box((0.035, 0.040, 1.50)),
            origin=Origin(xyz=(x, 0.0, 1.10)),
            material=dark_steel,
            name=f"picket_{i}",
        )

    # Datum faces are separate visual features but overlap the frame, so they
    # read as machined pads rather than loose labels.
    gate_leaf.visual(
        Box((0.022, 0.090, 0.16)),
        origin=Origin(xyz=(-1.302, 0.0, 0.34)),
        material=datum_blue,
        name="closed_datum_face",
    )
    gate_leaf.visual(
        Box((0.022, 0.090, 0.16)),
        origin=Origin(xyz=(1.302, 0.0, 0.34)),
        material=datum_blue,
        name="open_datum_face",
    )
    gate_leaf.visual(
        Box((0.012, 0.006, 0.50)),
        origin=Origin(xyz=(-0.98, -0.018, 1.34)),
        material=white,
        name="moving_index_line",
    )
    gate_leaf.visual(
        Box((0.22, 0.014, 0.12)),
        origin=Origin(xyz=(-1.18, -0.034, 1.10)),
        material=galvanized,
        name="latch_mount_plate",
    )
    gate_leaf.visual(
        Box((0.10, 0.052, 0.014)),
        origin=Origin(xyz=(-1.245, -0.056, 1.1245)),
        material=galvanized,
        name="upper_latch_keeper",
    )
    gate_leaf.visual(
        Box((0.10, 0.052, 0.014)),
        origin=Origin(xyz=(-1.245, -0.056, 1.0755)),
        material=galvanized,
        name="lower_latch_keeper",
    )

    # Roller forks and shim stacks are fixed to the moving leaf.
    for i, x in enumerate((-0.72, 0.72)):
        gate_leaf.visual(
            Box((0.17, 0.012, 0.21)),
            origin=Origin(xyz=(x, -0.090, 0.245)),
            material=galvanized,
            name=f"front_roller_fork_{i}",
        )
        gate_leaf.visual(
            Box((0.17, 0.012, 0.21)),
            origin=Origin(xyz=(x, 0.090, 0.245)),
            material=galvanized,
            name=f"rear_roller_fork_{i}",
        )
        gate_leaf.visual(
            Box((0.17, 0.20, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.335)),
            material=galvanized,
            name=f"roller_crosshead_{i}",
        )
        gate_leaf.visual(
            Cylinder(radius=0.015, length=0.22),
            origin=Origin(xyz=(x, 0.0, 0.225), rpy=(pi / 2, 0.0, 0.0)),
            material=galvanized,
            name=f"roller_axle_{i}",
        )
        gate_leaf.visual(
            Box((0.12, 0.050, 0.008)),
            origin=Origin(xyz=(x, -0.074, 0.338)),
            material=yellow,
            name=f"shim_stack_{i}",
        )

    gate_slide = model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=gate_leaf,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=850.0, velocity=0.35, lower=0.0, upper=1.20),
        motion_properties=MotionProperties(damping=35.0, friction=6.0),
    )

    for i, x in enumerate((-0.72, 0.72)):
        roller = model.part(f"roller_{i}")
        roller.visual(
            Cylinder(radius=0.080, length=0.050),
            origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
            material=rubber,
            name="wheel_tire",
        )
        roller.visual(
            Cylinder(radius=0.040, length=0.060),
            origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
            material=galvanized,
            name="wheel_hub",
        )
        roller.visual(
            Box((0.020, 0.006, 0.060)),
            origin=Origin(xyz=(0.072, -0.023, 0.0)),
            material=white,
            name="rotation_witness",
        )
        model.articulation(
            f"roller_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=gate_leaf,
            child=roller,
            origin=Origin(xyz=(x, 0.0, 0.225)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=20.0),
            motion_properties=MotionProperties(damping=0.05, friction=0.02),
        )

    latch_bolt = model.part("latch_bolt")
    latch_bolt.visual(
        Box((0.180, 0.035, 0.035)),
        origin=Origin(),
        material=dark_steel,
        name="bolt_body",
    )
    latch_bolt.visual(
        Box((0.045, 0.060, 0.060)),
        origin=Origin(xyz=(0.105, -0.100, 0.0)),
        material=yellow,
        name="pull_tab",
    )
    latch_bolt.visual(
        Box((0.035, 0.080, 0.025)),
        origin=Origin(xyz=(0.105, -0.040, 0.0)),
        material=yellow,
        name="pull_stem",
    )
    model.articulation(
        "latch_throw",
        ArticulationType.PRISMATIC,
        parent=gate_leaf,
        child=latch_bolt,
        origin=Origin(xyz=(-1.230, -0.090, 1.10)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.12, lower=0.0, upper=0.10),
        motion_properties=MotionProperties(damping=8.0, friction=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("fixed_frame")
    gate = object_model.get_part("gate_leaf")
    roller_0 = object_model.get_part("roller_0")
    roller_1 = object_model.get_part("roller_1")
    latch = object_model.get_part("latch_bolt")
    gate_slide = object_model.get_articulation("gate_slide")
    latch_throw = object_model.get_articulation("latch_throw")

    for i, roller in enumerate((roller_0, roller_1)):
        ctx.allow_overlap(
            gate,
            roller,
            elem_a=f"roller_axle_{i}",
            elem_b="wheel_hub",
            reason="The fixed axle is intentionally captured through the roller hub.",
        )
        ctx.allow_overlap(
            gate,
            roller,
            elem_a=f"roller_axle_{i}",
            elem_b="wheel_tire",
            reason="The tire is a simplified solid wheel proxy around the captured axle.",
        )
        ctx.expect_within(
            gate,
            roller,
            axes="xz",
            inner_elem=f"roller_axle_{i}",
            outer_elem="wheel_hub",
            margin=0.001,
            name=f"roller {i} axle centered in hub bore",
        )
        ctx.expect_overlap(
            gate,
            roller,
            axes="y",
            elem_a=f"roller_axle_{i}",
            elem_b="wheel_hub",
            min_overlap=0.055,
            name=f"roller {i} axle spans hub width",
        )

    # The rolling hardware sits on the lower rail and remains captured laterally
    # by the channel lips in the calibrated closed position.
    ctx.expect_gap(
        roller_0,
        frame,
        axis="z",
        positive_elem="wheel_tire",
        negative_elem="lower_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="front roller sits on lower rail",
    )
    ctx.expect_gap(
        roller_1,
        frame,
        axis="z",
        positive_elem="wheel_tire",
        negative_elem="lower_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear roller sits on lower rail",
    )
    ctx.expect_within(
        roller_0,
        frame,
        axes="y",
        inner_elem="wheel_tire",
        outer_elem="lower_rail",
        margin=0.004,
        name="roller 0 centered in lower rail channel",
    )
    ctx.expect_within(
        roller_1,
        frame,
        axes="y",
        inner_elem="wheel_tire",
        outer_elem="lower_rail",
        margin=0.004,
        name="roller 1 centered in lower rail channel",
    )

    ctx.expect_gap(
        gate,
        frame,
        axis="x",
        positive_elem="closed_datum_face",
        negative_elem="closed_stop_pad",
        min_gap=0.020,
        max_gap=0.040,
        name="closed stop has controlled calibration gap",
    )
    ctx.expect_gap(
        latch,
        frame,
        axis="x",
        positive_elem="bolt_body",
        negative_elem="receiver_mouth_datum",
        min_gap=0.010,
        max_gap=0.030,
        name="retracted latch has visible receiver gap",
    )
    ctx.expect_within(
        gate,
        frame,
        axes="y",
        inner_elem="top_rail",
        outer_elem="top_channel_cap",
        margin=0.002,
        name="gate top rail captured by upper channel",
    )

    closed_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: 1.20}):
        ctx.expect_gap(
            frame,
            gate,
            axis="x",
            positive_elem="open_stop_pad",
            negative_elem="open_datum_face",
            min_gap=0.010,
            max_gap=0.030,
            name="open stop has controlled calibration gap",
        )
        ctx.expect_overlap(
            roller_0,
            frame,
            axes="x",
            elem_a="wheel_tire",
            elem_b="lower_rail",
            min_overlap=0.05,
            name="roller 0 remains over rail at full travel",
        )
        ctx.expect_overlap(
            roller_1,
            frame,
            axes="x",
            elem_a="wheel_tire",
            elem_b="lower_rail",
            min_overlap=0.05,
            name="roller 1 remains over rail at full travel",
        )
        ctx.expect_overlap(
            gate,
            frame,
            axes="x",
            elem_a="top_rail",
            elem_b="top_channel_cap",
            min_overlap=1.0,
            name="upper guide channel overlaps gate through travel",
        )
        open_pos = ctx.part_world_position(gate)

    ctx.check(
        "gate leaf translates along rail",
        closed_pos is not None and open_pos is not None and open_pos[0] > closed_pos[0] + 1.15,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    with ctx.pose({latch_throw: 0.10}):
        ctx.expect_overlap(
            latch,
            frame,
            axes="x",
            elem_a="bolt_body",
            elem_b="receiver_upper_jaw",
            min_overlap=0.04,
            name="thrown latch enters receiver mouth",
        )
        ctx.expect_within(
            latch,
            frame,
            axes="yz",
            inner_elem="bolt_body",
            outer_elem="receiver_backplate",
            margin=0.002,
            name="thrown latch is centered on receiver datum",
        )

    return ctx.report()


object_model = build_object_model()
