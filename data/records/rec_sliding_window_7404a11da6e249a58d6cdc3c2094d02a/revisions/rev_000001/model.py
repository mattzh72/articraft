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
    model = ArticulatedObject(name="rugged_sliding_window")

    powder_coat = Material("chipped charcoal powder coat", rgba=(0.10, 0.12, 0.11, 1.0))
    worn_edge = Material("worn exposed metal edges", rgba=(0.48, 0.48, 0.43, 1.0))
    black_rubber = Material("black epdm rubber seals", rgba=(0.01, 0.01, 0.01, 1.0))
    smoked_glass = Material("smoked safety glass", rgba=(0.33, 0.48, 0.56, 0.38))
    stainless = Material("stainless fasteners and roller track", rgba=(0.73, 0.72, 0.67, 1.0))
    nylon = Material("dark nylon rollers", rgba=(0.03, 0.03, 0.035, 1.0))

    frame = model.part("frame")

    # Heavy outer rectangular frame with an open center aperture.
    frame.visual(Box((1.60, 0.16, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.425)), material=powder_coat, name="top_rail")
    frame.visual(Box((1.60, 0.18, 0.06)), origin=Origin(xyz=(0.0, 0.0, -0.445)), material=powder_coat, name="bottom_sill")
    frame.visual(Box((0.12, 0.16, 0.95)), origin=Origin(xyz=(-0.74, 0.0, 0.0)), material=powder_coat, name="jamb_0")
    frame.visual(Box((0.12, 0.16, 0.95)), origin=Origin(xyz=(0.74, 0.0, 0.0)), material=powder_coat, name="jamb_1")

    # Boxed guide channels, with lips and a central divider making the slide path legible.
    frame.visual(Box((1.36, 0.018, 0.11)), origin=Origin(xyz=(0.0, -0.083, -0.335)), material=powder_coat, name="lower_front_lip")
    frame.visual(Box((1.36, 0.010, 0.10)), origin=Origin(xyz=(0.0, 0.0, -0.335)), material=black_rubber, name="lower_track_divider")
    frame.visual(Box((1.36, 0.018, 0.11)), origin=Origin(xyz=(0.0, 0.083, -0.335)), material=powder_coat, name="lower_rear_lip")
    frame.visual(Box((1.36, 0.018, 0.11)), origin=Origin(xyz=(0.0, -0.083, 0.335)), material=powder_coat, name="upper_front_lip")
    frame.visual(Box((1.36, 0.010, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.335)), material=black_rubber, name="upper_track_divider")
    frame.visual(Box((1.36, 0.018, 0.11)), origin=Origin(xyz=(0.0, 0.083, 0.335)), material=powder_coat, name="upper_rear_lip")

    frame.visual(Box((1.34, 0.012, 0.010)), origin=Origin(xyz=(0.0, -0.035, -0.394)), material=stainless, name="front_roller_track")
    frame.visual(Box((1.34, 0.012, 0.010)), origin=Origin(xyz=(0.0, 0.035, -0.394)), material=stainless, name="rear_roller_track")
    frame.visual(Box((1.34, 0.018, 0.016)), origin=Origin(xyz=(0.0, -0.035, -0.407)), material=powder_coat, name="front_track_bed")
    frame.visual(Box((1.34, 0.018, 0.016)), origin=Origin(xyz=(0.0, 0.035, -0.407)), material=powder_coat, name="rear_track_bed")
    frame.visual(Box((1.34, 0.012, 0.010)), origin=Origin(xyz=(0.0, -0.035, 0.394)), material=worn_edge, name="front_top_wear_strip")
    frame.visual(Box((1.34, 0.012, 0.010)), origin=Origin(xyz=(0.0, 0.035, 0.394)), material=worn_edge, name="rear_top_wear_strip")

    # Fixed rear light carried by the frame; its right stop ties it into the jamb.
    frame.visual(Box((0.50, 0.010, 0.50)), origin=Origin(xyz=(0.34, 0.035, 0.0)), material=smoked_glass, name="fixed_glass")
    frame.visual(Box((0.66, 0.050, 0.065)), origin=Origin(xyz=(0.34, 0.035, 0.2825)), material=powder_coat, name="fixed_top_rail")
    frame.visual(Box((0.66, 0.050, 0.065)), origin=Origin(xyz=(0.34, 0.035, -0.2825)), material=powder_coat, name="fixed_bottom_rail")
    frame.visual(Box((0.055, 0.050, 0.68)), origin=Origin(xyz=(0.6175, 0.035, 0.0)), material=powder_coat, name="fixed_side_stile")
    frame.visual(Box((0.055, 0.050, 0.68)), origin=Origin(xyz=(0.0625, 0.035, 0.0)), material=powder_coat, name="fixed_meeting_stile")
    frame.visual(Box((0.020, 0.060, 0.62)), origin=Origin(xyz=(0.678, 0.035, 0.0)), material=powder_coat, name="fixed_retainer")
    frame.visual(Box((0.040, 0.012, 0.62)), origin=Origin(xyz=(-0.016, 0.004, 0.0)), material=black_rubber, name="fixed_weather_lap")

    # Exposed reinforcement plates and service fasteners on the rugged frame.
    for idx, (x, z) in enumerate(((-0.675, -0.405), (-0.675, 0.405), (0.675, -0.405), (0.675, 0.405))):
        frame.visual(Box((0.20, 0.010, 0.16)), origin=Origin(xyz=(x, -0.086, z)), material=worn_edge, name=f"corner_plate_{idx}")

    screw_points = [
        (-0.69, 0.34), (-0.37, 0.43), (0.0, 0.43), (0.37, 0.43), (0.69, 0.34),
        (-0.69, -0.34), (-0.37, -0.43), (0.0, -0.43), (0.37, -0.43), (0.69, -0.34),
        (-0.74, 0.12), (-0.74, -0.12), (0.74, 0.12), (0.74, -0.12),
    ]
    for idx, (x, z) in enumerate(screw_points):
        frame.visual(
            Cylinder(radius=0.012, length=0.009),
            origin=Origin(xyz=(x, -0.0835, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"frame_screw_{idx}",
        )

    sash = model.part("sash")
    # Moving front-lane sash.  It is centered on its closed position; the joint
    # frame is therefore also the sash center at q=0.
    sash.visual(Box((0.66, 0.055, 0.065)), origin=Origin(xyz=(0.0, 0.0, 0.3075)), material=powder_coat, name="sash_top_rail")
    sash.visual(Box((0.66, 0.055, 0.065)), origin=Origin(xyz=(0.0, 0.0, -0.3075)), material=powder_coat, name="sash_bottom_rail")
    sash.visual(Box((0.055, 0.055, 0.68)), origin=Origin(xyz=(-0.3025, 0.0, 0.0)), material=powder_coat, name="sash_side_stile")
    sash.visual(Box((0.060, 0.055, 0.68)), origin=Origin(xyz=(0.3025, 0.0, 0.0)), material=powder_coat, name="sash_meeting_stile")
    sash.visual(Box((0.55, 0.010, 0.55)), origin=Origin(xyz=(0.0, -0.002, 0.0)), material=smoked_glass, name="sash_glass")
    sash.visual(Box((0.040, 0.018, 0.62)), origin=Origin(xyz=(0.322, 0.018, 0.0)), material=black_rubber, name="meeting_flange")

    # Glass gaskets and molded grip ribs visibly seat the pane and show seal breaks.
    sash.visual(Box((0.54, 0.014, 0.018)), origin=Origin(xyz=(0.0, -0.010, 0.263)), material=black_rubber, name="top_glass_gasket")
    sash.visual(Box((0.54, 0.014, 0.018)), origin=Origin(xyz=(0.0, -0.010, -0.263)), material=black_rubber, name="bottom_glass_gasket")
    sash.visual(Box((0.018, 0.014, 0.54)), origin=Origin(xyz=(-0.263, -0.010, 0.0)), material=black_rubber, name="side_glass_gasket")
    sash.visual(Box((0.018, 0.014, 0.54)), origin=Origin(xyz=(0.263, -0.010, 0.0)), material=black_rubber, name="meeting_glass_gasket")
    for idx, z in enumerate((-0.12, 0.0, 0.12)):
        sash.visual(Box((0.018, 0.012, 0.070)), origin=Origin(xyz=(-0.275, -0.027, z)), material=black_rubber, name=f"grip_rib_{idx}")

    # Corner reinforcement plates and exposed sash screws.
    for idx, (x, z) in enumerate(((-0.282, -0.285), (-0.282, 0.285), (0.282, -0.285), (0.282, 0.285))):
        sash.visual(Box((0.115, 0.008, 0.095)), origin=Origin(xyz=(x, -0.031, z)), material=worn_edge, name=f"sash_corner_plate_{idx}")
        sash.visual(
            Cylinder(radius=0.009, length=0.007),
            origin=Origin(xyz=(x - 0.025, -0.032, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"sash_screw_a_{idx}",
        )
        sash.visual(
            Cylinder(radius=0.009, length=0.007),
            origin=Origin(xyz=(x + 0.025, -0.032, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"sash_screw_b_{idx}",
        )

    # Captive roller trucks on the bottom rail; the wheel bottoms are tangent to
    # the frame's front roller track in the closed pose.
    sash.visual(Box((0.070, 0.034, 0.035)), origin=Origin(xyz=(-0.21, 0.0, -0.344)), material=powder_coat, name="roller_bracket_0")
    sash.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(-0.21, 0.0, -0.365), rpy=(pi / 2.0, 0.0, 0.0)),
        material=nylon,
        name="roller_0",
    )
    sash.visual(Box((0.070, 0.034, 0.035)), origin=Origin(xyz=(0.21, 0.0, -0.344)), material=powder_coat, name="roller_bracket_1")
    sash.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.21, 0.0, -0.365), rpy=(pi / 2.0, 0.0, 0.0)),
        material=nylon,
        name="roller_1",
    )

    model.articulation(
        "sash_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(-0.34, -0.035, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.66),
        motion_properties=MotionProperties(damping=3.0, friction=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("sash_slide")

    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        positive_elem="sash_bottom_rail",
        negative_elem="lower_front_lip",
        min_gap=0.006,
        name="front channel lip clears moving sash",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        positive_elem="lower_track_divider",
        negative_elem="sash_bottom_rail",
        min_gap=0.0015,
        name="track divider keeps sash in front lane",
    )
    ctx.expect_contact(
        sash,
        frame,
        elem_a="roller_0",
        elem_b="front_roller_track",
        contact_tol=0.002,
        name="front roller rides on metal track",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="xz",
        elem_a="meeting_flange",
        elem_b="fixed_weather_lap",
        min_overlap=0.010,
        name="closed meeting rail has weather-lap overlap",
    )

    closed_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.66}):
        ctx.expect_within(
            sash,
            frame,
            axes="xz",
            margin=0.02,
            name="open sash remains inside rugged frame aperture",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="roller_0",
            elem_b="front_roller_track",
            contact_tol=0.002,
            name="roller remains supported at full travel",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="xz",
            elem_a="sash_glass",
            elem_b="fixed_glass",
            min_overlap=0.35,
            name="open sash stacks over fixed light on parallel track",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash slides horizontally to the service side",
        closed_pos is not None and open_pos is not None and open_pos[0] > closed_pos[0] + 0.60,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
