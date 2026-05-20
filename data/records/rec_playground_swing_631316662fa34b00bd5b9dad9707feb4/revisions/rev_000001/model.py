from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="standing_platform_swing")

    frame_mat = model.material("powder_coated_frame", rgba=(0.18, 0.22, 0.24, 1.0))
    pin_mat = model.material("dark_pivot_pins", rgba=(0.04, 0.045, 0.045, 1.0))
    link_mat = model.material("galvanized_links", rgba=(0.62, 0.66, 0.65, 1.0))
    deck_mat = model.material("black_rubber_tread", rgba=(0.015, 0.016, 0.014, 1.0))
    rail_mat = model.material("safety_red_rail", rgba=(0.72, 0.08, 0.035, 1.0))

    top_z = 2.00
    link_len = 1.20
    side_y = 0.43

    frame = model.part("frame")
    frame.visual(
        Box((0.76, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, -0.58, 0.04)),
        material=frame_mat,
        name="foot_0",
    )
    frame.visual(
        Box((0.76, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.58, 0.04)),
        material=frame_mat,
        name="foot_1",
    )
    frame.visual(
        Box((0.12, 0.12, 2.12)),
        origin=Origin(xyz=(0.0, -0.58, 1.06)),
        material=frame_mat,
        name="post_0",
    )
    frame.visual(
        Box((0.12, 0.12, 2.12)),
        origin=Origin(xyz=(0.0, 0.58, 1.06)),
        material=frame_mat,
        name="post_1",
    )
    frame.visual(
        Box((0.18, 1.36, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 2.14)),
        material=frame_mat,
        name="top_beam",
    )

    # Two clevis-style top hangers fixed to the beam; the rotating link eyes sit
    # in the gaps and are captured by the dark pins.
    for idx, y in enumerate((-side_y, side_y)):
        for plate_idx, y_offset in enumerate((-0.055, 0.055)):
            frame.visual(
                Box((0.12, 0.018, 0.20)),
                origin=Origin(xyz=(0.0, y + y_offset, top_z - 0.01)),
                material=frame_mat,
                name=f"top_yoke_{idx}_{plate_idx}",
            )
        frame.visual(
            Cylinder(radius=0.012, length=0.17),
            origin=Origin(xyz=(0.0, y, top_z), rpy=(pi / 2, 0.0, 0.0)),
            material=pin_mat,
            name=f"top_pin_{idx}",
        )

    side_links = model.part("side_links")
    for idx, y in enumerate((-side_y, side_y)):
        side_links.visual(
            Cylinder(radius=0.044, length=0.070),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=link_mat,
            name=f"top_eye_{idx}",
        )
        side_links.visual(
            Cylinder(radius=0.017, length=link_len - 0.08),
            origin=Origin(xyz=(0.0, y, -link_len / 2.0)),
            material=link_mat,
            name=f"side_bar_{idx}",
        )
        side_links.visual(
            Cylinder(radius=0.044, length=0.070),
            origin=Origin(xyz=(0.0, y, -link_len), rpy=(pi / 2, 0.0, 0.0)),
            material=link_mat,
            name=f"lower_eye_{idx}",
        )
    side_links.visual(
        Cylinder(radius=0.014, length=2.0 * side_y),
        origin=Origin(xyz=(-0.020, 0.0, -link_len / 2.0), rpy=(pi / 2, 0.0, 0.0)),
        material=link_mat,
        name="spreader_tube",
    )

    platform = model.part("platform")
    platform.visual(
        Box((0.72, 0.84, 0.035)),
        origin=Origin(xyz=(0.05, 0.0, -0.075)),
        material=deck_mat,
        name="tread_deck",
    )
    platform.visual(
        Box((0.74, 0.052, 0.065)),
        origin=Origin(xyz=(0.05, -side_y, -0.080)),
        material=rail_mat,
        name="side_rail_0",
    )
    platform.visual(
        Box((0.74, 0.052, 0.065)),
        origin=Origin(xyz=(0.05, side_y, -0.080)),
        material=rail_mat,
        name="side_rail_1",
    )
    platform.visual(
        Box((0.065, 0.90, 0.065)),
        origin=Origin(xyz=(-0.325, 0.0, -0.080)),
        material=rail_mat,
        name="rear_rail",
    )
    platform.visual(
        Box((0.065, 0.90, 0.065)),
        origin=Origin(xyz=(0.425, 0.0, -0.080)),
        material=rail_mat,
        name="front_rail",
    )
    for idx, x in enumerate((-0.18, -0.06, 0.06, 0.18, 0.30)):
        platform.visual(
            Box((0.020, 0.56, 0.008)),
            origin=Origin(xyz=(x, 0.0, -0.058)),
            material=rail_mat,
            name=f"tread_rib_{idx}",
        )

    # Lower clevises are part of the platform frame.  Their pins pass through
    # the side-link lower eyes, keeping the standing deck clipped to the pivots.
    for idx, y in enumerate((-side_y, side_y)):
        platform.visual(
            Box((0.13, 0.16, 0.045)),
            origin=Origin(xyz=(0.0, y, -0.085)),
            material=rail_mat,
            name=f"clevis_base_{idx}",
        )
        for plate_idx, y_offset in enumerate((-0.055, 0.055)):
            platform.visual(
                Box((0.11, 0.018, 0.16)),
                origin=Origin(xyz=(0.0, y + y_offset, -0.010)),
                material=rail_mat,
                name=f"lower_yoke_{idx}_{plate_idx}",
            )
        platform.visual(
            Cylinder(radius=0.012, length=0.17),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=pin_mat,
            name=f"lower_pin_{idx}",
        )

    # Short front hand bar for a standing rider.
    for idx, y in enumerate((-0.24, 0.24)):
        platform.visual(
            Cylinder(radius=0.018, length=0.46),
            origin=Origin(xyz=(0.425, y, 0.135)),
            material=rail_mat,
            name=f"hand_post_{idx}",
        )
    platform.visual(
        Cylinder(radius=0.022, length=0.54),
        origin=Origin(xyz=(0.425, 0.0, 0.37), rpy=(pi / 2, 0.0, 0.0)),
        material=rail_mat,
        name="hand_bar",
    )

    top_joint = model.articulation(
        "top_pivots",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=side_links,
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "lower_pivots",
        ArticulationType.REVOLUTE,
        parent=side_links,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, -link_len)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.2, lower=-0.45, upper=0.45),
        mimic=Mimic(joint=top_joint.name, multiplier=-1.0, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    side_links = object_model.get_part("side_links")
    platform = object_model.get_part("platform")
    top_joint = object_model.get_articulation("top_pivots")
    lower_joint = object_model.get_articulation("lower_pivots")

    for idx in (0, 1):
        ctx.allow_overlap(
            frame,
            side_links,
            elem_a=f"top_pin_{idx}",
            elem_b=f"top_eye_{idx}",
            reason="The beam-mounted pivot pin is intentionally captured inside the rotating top link eye.",
        )
        ctx.expect_overlap(
            frame,
            side_links,
            axes="xyz",
            min_overlap=0.015,
            elem_a=f"top_pin_{idx}",
            elem_b=f"top_eye_{idx}",
            name=f"top pivot {idx} pin captured in eye",
        )
        ctx.allow_overlap(
            platform,
            side_links,
            elem_a=f"lower_pin_{idx}",
            elem_b=f"lower_eye_{idx}",
            reason="The platform clevis pin intentionally passes through the lower link eye so the deck stays clipped to the pivot.",
        )
        ctx.expect_overlap(
            platform,
            side_links,
            axes="xyz",
            min_overlap=0.015,
            elem_a=f"lower_pin_{idx}",
            elem_b=f"lower_eye_{idx}",
            name=f"lower pivot {idx} pin captured in eye",
        )

    mimic = lower_joint.mimic
    ctx.check(
        "lower pivots counter-rotate with top swing",
        mimic is not None
        and mimic.joint == top_joint.name
        and abs(mimic.multiplier + 1.0) < 1e-9,
        details=f"mimic={mimic}",
    )

    rest_pos = ctx.part_world_position(platform)
    with ctx.pose({top_joint: 0.35}):
        swung_pos = ctx.part_world_position(platform)
        for idx in (0, 1):
            ctx.expect_overlap(
                platform,
                side_links,
                axes="xyz",
                min_overlap=0.015,
                elem_a=f"lower_pin_{idx}",
                elem_b=f"lower_eye_{idx}",
                name=f"lower pivot {idx} remains clipped while swinging",
            )
    ctx.check(
        "platform swings under gantry",
        rest_pos is not None and swung_pos is not None and swung_pos[0] < rest_pos[0] - 0.20,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
