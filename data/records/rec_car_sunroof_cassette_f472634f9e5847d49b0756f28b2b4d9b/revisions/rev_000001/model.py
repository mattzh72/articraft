from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_soft_top_sunroof_cassette")

    aluminum = model.material("brushed_aluminium", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_shadow = model.material("channel_shadow", rgba=(0.015, 0.016, 0.018, 1.0))
    canvas_mat = model.material("black_canvas", rgba=(0.025, 0.027, 0.025, 1.0))
    seam_mat = model.material("canvas_seams", rgba=(0.055, 0.058, 0.052, 1.0))
    nylon = model.material("black_nylon_shoe", rgba=(0.05, 0.052, 0.055, 1.0))
    steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.30, 1.0))

    frame = model.part("frame")

    # Overall cassette footprint is approximately an automotive roof aperture:
    # 1.1 m wide and just under 0.9 m front-to-rear.
    for idx, (x, floor_name) in enumerate(((-0.52, "channel_floor_0"), (0.52, "channel_floor_1"))):
        frame.visual(
            Box((0.084, 0.86, 0.016)),
            origin=Origin(xyz=(x, 0.0, 0.013)),
            material=aluminum,
            name=floor_name,
        )
        # Raised C-channel lips leave a real open slot for the nylon guide shoe.
        for lip_idx, lip_x in enumerate((x - 0.035 if x < 0 else x + 0.035, x + 0.035 if x < 0 else x - 0.035)):
            frame.visual(
                Box((0.012, 0.86, 0.030)),
                origin=Origin(xyz=(lip_x, 0.0, 0.035)),
                material=aluminum,
                name=f"channel_lip_{idx}_{lip_idx}",
            )
        frame.visual(
            Box((0.050, 0.82, 0.003)),
            origin=Origin(xyz=(x, 0.0, 0.0225)),
            material=dark_shadow,
            name=f"dark_slot_{idx}",
        )

    frame.visual(
        Box((1.10, 0.070, 0.042)),
        origin=Origin(xyz=(0.0, -0.455, 0.027)),
        material=aluminum,
        name="front_crossmember",
    )
    frame.visual(
        Box((1.10, 0.060, 0.046)),
        origin=Origin(xyz=(0.0, 0.445, 0.029)),
        material=aluminum,
        name="rear_crossmember",
    )
    frame.visual(
        Box((0.98, 0.032, 0.024)),
        origin=Origin(xyz=(0.0, 0.427, 0.050)),
        material=aluminum,
        name="rear_bridge",
    )
    frame.visual(
        Box((0.98, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, -0.390, 0.046)),
        material=aluminum,
        name="front_bridge",
    )

    # Bearing cheeks cradle the rear roller without occupying the drum envelope.
    for idx, x in enumerate((-0.488, 0.488)):
        frame.visual(
            Box((0.030, 0.066, 0.088)),
            origin=Origin(xyz=(x, 0.345, 0.088)),
            material=aluminum,
            name=f"bearing_cheek_{idx}",
        )
        frame.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(xyz=(x * 0.995, 0.345, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"bearing_bush_{idx}",
        )

    # Small flush fastener heads make the aluminium extrusion read as a cassette.
    screw_sites = [
        (-0.565, -0.33),
        (-0.565, 0.06),
        (-0.565, 0.32),
        (0.565, -0.33),
        (0.565, 0.06),
        (0.565, 0.32),
        (-0.36, -0.455),
        (0.36, -0.455),
        (-0.36, 0.445),
        (0.36, 0.445),
    ]
    for idx, (x, y) in enumerate(screw_sites):
        screw_z = 0.050 if y < -0.42 else 0.052
        frame.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x, y, screw_z)),
            material=steel,
            name=f"screw_{idx}",
        )

    shoe_0 = model.part("shoe_0")
    shoe_0.visual(
        Box((0.044, 0.094, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=nylon,
        name="shoe_body",
    )
    shoe_0.visual(
        Box((0.020, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=nylon,
        name="guide_pin",
    )

    shoe_1 = model.part("shoe_1")
    shoe_1.visual(
        Box((0.044, 0.094, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=nylon,
        name="shoe_body",
    )
    shoe_1.visual(
        Box((0.020, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=nylon,
        name="guide_pin",
    )

    canvas_panel = model.part("canvas_panel")
    canvas_panel.visual(
        Box((0.950, 0.540, 0.010)),
        origin=Origin(xyz=(0.0, 0.270, 0.0)),
        material=canvas_mat,
        name="fabric_sheet",
    )
    canvas_panel.visual(
        Box((0.840, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.555, 0.0)),
        material=canvas_mat,
        name="rear_lead_strip",
    )
    canvas_panel.visual(
        Box((0.980, 0.035, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=aluminum,
        name="front_bow",
    )
    for idx, (x, bead_name) in enumerate(((-0.520, "side_bead_0"), (0.520, "side_bead_1"))):
        canvas_panel.visual(
            Box((0.032, 0.570, 0.014)),
            origin=Origin(xyz=(x, 0.285, -0.013)),
            material=canvas_mat,
            name=bead_name,
        )
        bridge_x = x + (0.025 if x < 0 else -0.025)
        canvas_panel.visual(
            Box((0.050, 0.540, 0.008)),
            origin=Origin(xyz=(bridge_x, 0.270, -0.007)),
            material=canvas_mat,
            name=f"edge_fold_{idx}",
        )
    for idx, y in enumerate((0.095, 0.185, 0.275, 0.365, 0.455)):
        canvas_panel.visual(
            Box((0.860, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.007)),
            material=seam_mat,
            name=f"transverse_seam_{idx}",
        )

    rear_drum = model.part("rear_drum")
    rear_drum.visual(
        Cylinder(radius=0.034, length=0.940),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="steel_core",
    )
    rear_drum.visual(
        Cylinder(radius=0.058, length=0.880),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=canvas_mat,
        name="canvas_roll",
    )
    rear_drum.visual(
        Box((0.820, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, -0.045, -0.034)),
        material=canvas_mat,
        name="drum_tail",
    )
    for idx, x in enumerate((-0.455, 0.455)):
        rear_drum.visual(
            Cylinder(radius=0.062, length=0.028),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name=f"end_cap_{idx}",
        )

    shoe_limits = MotionLimits(effort=75.0, velocity=0.35, lower=0.0, upper=0.32)
    model.articulation(
        "frame_to_shoe_0",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shoe_0,
        origin=Origin(xyz=(-0.520, -0.270, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=shoe_limits,
    )
    model.articulation(
        "frame_to_shoe_1",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shoe_1,
        origin=Origin(xyz=(0.520, -0.270, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=shoe_limits,
        mimic=Mimic("frame_to_shoe_0"),
    )
    model.articulation(
        "shoe_0_to_canvas_panel",
        ArticulationType.FIXED,
        parent=shoe_0,
        child=canvas_panel,
        origin=Origin(xyz=(0.520, 0.0, 0.031)),
    )
    model.articulation(
        "frame_to_rear_drum",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_drum,
        origin=Origin(xyz=(0.0, 0.345, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    shoe_0 = object_model.get_part("shoe_0")
    shoe_1 = object_model.get_part("shoe_1")
    canvas = object_model.get_part("canvas_panel")
    drum = object_model.get_part("rear_drum")
    slide = object_model.get_articulation("frame_to_shoe_0")
    drum_joint = object_model.get_articulation("frame_to_rear_drum")

    ctx.allow_overlap(
        canvas,
        drum,
        elem_a="rear_lead_strip",
        elem_b="drum_tail",
        reason="The rear canvas lead strip is intentionally clamped into the first wrap on the roller drum.",
    )
    ctx.allow_overlap(
        canvas,
        drum,
        elem_a="rear_lead_strip",
        elem_b="canvas_roll",
        reason="The fabric's rear edge intentionally tucks under the rolled canvas layer on the drum.",
    )
    ctx.expect_overlap(
        canvas,
        drum,
        axes="xy",
        elem_a="rear_lead_strip",
        elem_b="drum_tail",
        min_overlap=0.020,
        name="canvas lead strip is captured by drum tail",
    )
    ctx.expect_gap(
        canvas,
        drum,
        axis="z",
        positive_elem="rear_lead_strip",
        negative_elem="drum_tail",
        max_penetration=0.010,
        name="canvas-to-drum clamp overlap stays shallow",
    )
    ctx.expect_gap(
        drum,
        canvas,
        axis="y",
        positive_elem="canvas_roll",
        negative_elem="rear_lead_strip",
        max_penetration=0.016,
        name="canvas edge tucks shallowly under rolled layer",
    )

    ctx.expect_within(
        shoe_0,
        frame,
        axes="xy",
        inner_elem="shoe_body",
        outer_elem="channel_floor_0",
        margin=0.002,
        name="shoe 0 runs inside its side channel",
    )
    ctx.expect_within(
        shoe_1,
        frame,
        axes="xy",
        inner_elem="shoe_body",
        outer_elem="channel_floor_1",
        margin=0.002,
        name="shoe 1 runs inside its side channel",
    )
    ctx.expect_gap(
        shoe_0,
        frame,
        axis="z",
        positive_elem="shoe_body",
        negative_elem="channel_floor_0",
        min_gap=0.001,
        max_gap=0.006,
        name="shoe 0 is supported just above channel floor",
    )
    ctx.expect_gap(
        shoe_1,
        frame,
        axis="z",
        positive_elem="shoe_body",
        negative_elem="channel_floor_1",
        min_gap=0.001,
        max_gap=0.006,
        name="shoe 1 is supported just above channel floor",
    )
    ctx.expect_gap(
        canvas,
        shoe_0,
        axis="z",
        positive_elem="side_bead_0",
        negative_elem="guide_pin",
        min_gap=0.0,
        max_gap=0.002,
        name="shoe 0 guide pin reaches canvas edge bead",
    )
    ctx.expect_gap(
        canvas,
        shoe_1,
        axis="z",
        positive_elem="side_bead_1",
        negative_elem="guide_pin",
        min_gap=0.0,
        max_gap=0.002,
        name="shoe 1 guide pin reaches canvas edge bead",
    )

    closed_pos = ctx.part_world_position(canvas)
    with ctx.pose({slide: 0.32, drum_joint: math.pi}):
        retracted_pos = ctx.part_world_position(canvas)
        ctx.expect_within(
            shoe_0,
            frame,
            axes="xy",
            inner_elem="shoe_body",
            outer_elem="channel_floor_0",
            margin=0.002,
            name="shoe 0 remains in channel when retracted",
        )
        ctx.expect_within(
            shoe_1,
            frame,
            axes="xy",
            inner_elem="shoe_body",
            outer_elem="channel_floor_1",
            margin=0.002,
            name="shoe 1 remains in channel when retracted",
        )

    ctx.check(
        "canvas panel retracts rearward",
        closed_pos is not None and retracted_pos is not None and retracted_pos[1] > closed_pos[1] + 0.25,
        details=f"closed={closed_pos}, retracted={retracted_pos}",
    )

    return ctx.report()


object_model = build_object_model()
