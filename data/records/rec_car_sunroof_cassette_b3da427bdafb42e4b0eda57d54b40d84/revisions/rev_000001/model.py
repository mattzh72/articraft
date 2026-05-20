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
    model = ArticulatedObject(name="popup_vent_sunroof_cassette")

    aluminium = model.material("brushed_aluminium", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_track = model.material("dark_anodized_track", rgba=(0.03, 0.035, 0.04, 1.0))
    tinted_glass = model.material("blue_tinted_glass", rgba=(0.12, 0.28, 0.36, 0.42))
    black_ceramic = model.material("black_ceramic_frit", rgba=(0.005, 0.006, 0.007, 1.0))
    rubber = model.material("black_rubber_seal", rgba=(0.01, 0.01, 0.01, 1.0))
    nylon = model.material("dark_nylon_shoe", rgba=(0.08, 0.08, 0.075, 1.0))

    # Vehicle-style intrinsic frame:
    # +X runs from the front hinge rail toward the rear lifted edge,
    # +Y spans across the cassette, and +Z is upward.
    frame = model.part("frame")
    frame.visual(
        Box((0.080, 0.700, 0.035)),
        origin=Origin(xyz=(-0.040, 0.0, -0.035)),
        material=aluminium,
        name="front_rail",
    )
    frame.visual(
        Box((0.060, 0.700, 0.035)),
        origin=Origin(xyz=(0.810, 0.0, -0.035)),
        material=aluminium,
        name="rear_rail",
    )
    for suffix, sign in (("0", 1.0), ("1", -1.0)):
        frame.visual(
            Box((0.880, 0.050, 0.035)),
            origin=Origin(xyz=(0.400, sign * 0.325, -0.035)),
            material=aluminium,
            name=f"side_rail_{suffix}",
        )
        frame.visual(
            Box((0.880, 0.035, 0.035)),
            origin=Origin(xyz=(0.400, sign * 0.235, -0.035)),
            material=aluminium,
            name=f"inner_side_rail_{suffix}",
        )

    # Raised, open-top channel tracks for the two shoe blocks.
    for suffix, sign in (("0", 1.0), ("1", -1.0)):
        channel_y = sign * 0.275
        frame.visual(
            Box((0.700, 0.030, 0.004)),
            origin=Origin(xyz=(0.390, channel_y, -0.021)),
            material=dark_track,
            name=f"track_floor_{suffix}",
        )
        frame.visual(
            Box((0.700, 0.008, 0.018)),
            origin=Origin(xyz=(0.390, channel_y - sign * 0.0185, -0.013)),
            material=aluminium,
            name=f"track_inner_lip_{suffix}",
        )
        frame.visual(
            Box((0.700, 0.008, 0.018)),
            origin=Origin(xyz=(0.390, channel_y + sign * 0.021, -0.013)),
            material=aluminium,
            name=f"track_outer_lip_{suffix}",
        )

    # Alternating hinge knuckles mounted to the front rail.  The central
    # knuckle lives on the glass panel so the true revolute axis is visible.
    hinge_rpy = (math.pi / 2.0, 0.0, 0.0)
    for suffix, y in (("0", 0.165), ("1", -0.165)):
        frame.visual(
            Cylinder(radius=0.010, length=0.125),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=hinge_rpy),
            material=aluminium,
            name=f"frame_knuckle_{suffix}",
        )
        frame.visual(
            Box((0.055, 0.110, 0.010)),
            origin=Origin(xyz=(-0.028, y, -0.013)),
            material=aluminium,
            name=f"hinge_leaf_{suffix}",
        )

    screw_locations = [
        (-0.040, -0.270),
        (-0.040, 0.270),
        (0.805, -0.270),
        (0.805, 0.270),
        (0.240, -0.316),
        (0.560, -0.316),
        (0.240, 0.316),
        (0.560, 0.316),
    ]
    for index, (x, y) in enumerate(screw_locations):
        frame.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(x, y, -0.016)),
            material=dark_track,
            name=f"fastener_{index}",
        )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        Box((0.730, 0.490, 0.008)),
        origin=Origin(xyz=(0.415, 0.0, 0.004)),
        material=tinted_glass,
        name="glass_pane",
    )
    glass_panel.visual(
        Box((0.730, 0.026, 0.002)),
        origin=Origin(xyz=(0.415, 0.232, 0.009)),
        material=black_ceramic,
        name="frit_side_0",
    )
    glass_panel.visual(
        Box((0.730, 0.026, 0.002)),
        origin=Origin(xyz=(0.415, -0.232, 0.009)),
        material=black_ceramic,
        name="frit_side_1",
    )
    glass_panel.visual(
        Box((0.045, 0.490, 0.002)),
        origin=Origin(xyz=(0.024, 0.0, 0.009)),
        material=black_ceramic,
        name="frit_front",
    )
    glass_panel.visual(
        Box((0.055, 0.490, 0.002)),
        origin=Origin(xyz=(0.752, 0.0, 0.009)),
        material=black_ceramic,
        name="frit_rear",
    )
    glass_panel.visual(
        Box((0.030, 0.505, 0.010)),
        origin=Origin(xyz=(0.790, 0.0, -0.003)),
        material=rubber,
        name="rear_seal",
    )
    glass_panel.visual(
        Cylinder(radius=0.009, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_rpy),
        material=aluminium,
        name="panel_knuckle",
    )
    glass_panel.visual(
        Box((0.075, 0.185, 0.004)),
        origin=Origin(xyz=(0.038, 0.0, -0.002)),
        material=aluminium,
        name="panel_hinge_leaf",
    )

    # The two nylon shoes are separate prismatic parts constrained by the
    # channel tracks.  They are linearly coupled to the pop-up hinge so they
    # visibly ride rearward as the rear glass edge lifts.
    shoe_parts = []
    for suffix, sign in (("0", 1.0), ("1", -1.0)):
        shoe = model.part(f"guide_shoe_{suffix}")
        shoe.visual(
            Box((0.100, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=nylon,
            name="shoe_block",
        )
        shoe.visual(
            Cylinder(radius=0.006, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, -0.006), rpy=hinge_rpy),
            material=rubber,
            name="side_roller",
        )
        shoe.visual(
            Box((0.060, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, -sign * 0.010, -0.001)),
            material=nylon,
            name="shoe_riser",
        )
        shoe.visual(
            Box((0.060, 0.017, 0.002)),
            origin=Origin(xyz=(0.0, -sign * 0.0185, 0.002)),
            material=nylon,
            name="side_bridge",
        )
        shoe.visual(
            Box((0.045, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, -sign * 0.029, 0.003)),
            material=nylon,
            name="panel_pad",
        )
        shoe_parts.append((shoe, sign))

    model.articulation(
        "front_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=glass_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.2, lower=0.0, upper=0.36),
    )

    for suffix, (shoe, sign) in zip(("0", "1"), shoe_parts):
        model.articulation(
            f"shoe_slide_{suffix}",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=shoe,
            origin=Origin(xyz=(0.520, sign * 0.275, -0.005)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.080),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("glass_panel")
    shoe_0 = object_model.get_part("guide_shoe_0")
    shoe_1 = object_model.get_part("guide_shoe_1")
    hinge = object_model.get_articulation("front_hinge")
    slide_0 = object_model.get_articulation("shoe_slide_0")
    slide_1 = object_model.get_articulation("shoe_slide_1")

    ctx.expect_within(
        panel,
        frame,
        axes="xy",
        inner_elem="glass_pane",
        margin=0.010,
        name="closed glass pane sits inside the aluminium cassette footprint",
    )
    ctx.expect_gap(
        shoe_0,
        frame,
        axis="z",
        positive_elem="shoe_block",
        negative_elem="track_floor_0",
        max_gap=0.001,
        max_penetration=0.0005,
        name="guide shoe 0 is seated on its channel floor",
    )
    ctx.expect_gap(
        shoe_1,
        frame,
        axis="z",
        positive_elem="shoe_block",
        negative_elem="track_floor_1",
        max_gap=0.001,
        max_penetration=0.0005,
        name="guide shoe 1 is seated on its channel floor",
    )
    ctx.expect_within(
        shoe_0,
        frame,
        axes="xy",
        inner_elem="shoe_block",
        outer_elem="track_floor_0",
        margin=0.0,
        name="guide shoe 0 stays inside its channel at rest",
    )
    ctx.expect_within(
        shoe_1,
        frame,
        axes="xy",
        inner_elem="shoe_block",
        outer_elem="track_floor_1",
        margin=0.0,
        name="guide shoe 1 stays inside its channel at rest",
    )

    closed_rear = ctx.part_element_world_aabb(panel, elem="rear_seal")
    shoe_rest = ctx.part_world_position(shoe_0)
    with ctx.pose({hinge: 0.36, slide_0: 0.068, slide_1: 0.068}):
        raised_rear = ctx.part_element_world_aabb(panel, elem="rear_seal")
        shoe_raised = ctx.part_world_position(shoe_0)
        ctx.expect_within(
            shoe_0,
            frame,
            axes="xy",
            inner_elem="shoe_block",
            outer_elem="track_floor_0",
            margin=0.0,
            name="guide shoe 0 remains in its track while the panel tilts",
        )
        ctx.expect_within(
            shoe_1,
            frame,
            axes="xy",
            inner_elem="shoe_block",
            outer_elem="track_floor_1",
            margin=0.0,
            name="guide shoe 1 remains in its track while the panel tilts",
        )

    ctx.check(
        "rear glass edge tilts upward about the front hinge",
        closed_rear is not None
        and raised_rear is not None
        and raised_rear[1][2] > closed_rear[1][2] + 0.20,
        details=f"closed_rear={closed_rear}, raised_rear={raised_rear}",
    )
    ctx.check(
        "guide shoes ride rearward as the glass pops up",
        shoe_rest is not None and shoe_raised is not None and shoe_raised[0] > shoe_rest[0] + 0.055,
        details=f"rest={shoe_rest}, raised={shoe_raised}",
    )

    return ctx.report()


object_model = build_object_model()
