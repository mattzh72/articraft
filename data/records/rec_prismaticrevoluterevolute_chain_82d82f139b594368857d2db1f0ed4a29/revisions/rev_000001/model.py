from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="overhead_rail_inspection_arm")

    steel = model.material("painted_steel", rgba=(0.46, 0.49, 0.50, 1.0))
    dark = model.material("dark_bearing", rgba=(0.05, 0.055, 0.06, 1.0))
    rail_mat = model.material("polished_rail", rgba=(0.70, 0.72, 0.70, 1.0))
    safety = model.material("safety_yellow", rgba=(1.0, 0.72, 0.08, 1.0))
    arm_mat = model.material("arm_blue", rgba=(0.10, 0.22, 0.42, 1.0))
    plate_mat = model.material("inspection_plate_orange", rgba=(0.95, 0.36, 0.10, 1.0))

    beam = model.part("grounded_beam")
    # Floor feet and two uprights make the rail read as a grounded gantry, not a
    # free-floating ceiling strip.
    for x in (-1.35, 1.35):
        beam.visual(
            Box((0.42, 0.34, 0.05)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=steel,
            name=f"floor_foot_{'neg' if x < 0 else 'pos'}",
        )
        beam.visual(
            Box((0.12, 0.14, 2.18)),
            origin=Origin(xyz=(x, 0.0, 1.115)),
            material=steel,
            name=f"upright_{'neg' if x < 0 else 'pos'}",
        )
        beam.visual(
            Box((0.26, 0.22, 0.10)),
            origin=Origin(xyz=(x, 0.0, 2.20)),
            material=steel,
            name=f"top_knee_{'neg' if x < 0 else 'pos'}",
        )
        for y in (-0.11, 0.11):
            beam.visual(
                Cylinder(radius=0.018, length=0.018),
                origin=Origin(xyz=(x, y, 0.059)),
                material=dark,
                name=f"anchor_{'neg' if x < 0 else 'pos'}_{'a' if y < 0 else 'b'}",
            )

    # Box-built I-beam and twin rail strips under it.  The rail strips are part
    # of the fixed structure; the moving carriage below intentionally clears
    # them by a few millimetres.
    beam.visual(
        Box((2.85, 0.24, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.30)),
        material=steel,
        name="top_flange",
    )
    beam.visual(
        Box((2.85, 0.07, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 2.16)),
        material=steel,
        name="web",
    )
    beam.visual(
        Box((2.85, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.02)),
        material=steel,
        name="bottom_flange",
    )
    for y, rail_name in ((-0.075, "rail_near"), (0.075, "rail_far")):
        beam.visual(
            Box((2.55, 0.036, 0.035)),
            origin=Origin(xyz=(0.0, y, 1.965)),
            material=rail_mat,
            name=rail_name,
        )
    for x, stop_name in ((-0.82, "travel_stop_neg"), (0.82, "travel_stop_pos")):
        beam.visual(
            Box((0.055, 0.26, 0.070)),
            origin=Origin(xyz=(x, 0.0, 1.975)),
            material=dark,
            name=stop_name,
        )

    shuttle = model.part("shuttle")
    # The shuttle frame is the lower hinge center.  All carriage features sit
    # above it so the hanging arm is visibly carried by the moving shuttle.
    shuttle.visual(
        Box((0.34, 0.29, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=safety,
        name="upper_cover",
    )
    shuttle.visual(
        Box((0.30, 0.22, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=safety,
        name="lower_cover",
    )
    for y in (-0.165, 0.165):
        shuttle.visual(
            Box((0.30, 0.040, 0.22)),
            origin=Origin(xyz=(0.0, y, 0.105)),
            material=safety,
            name=f"side_cheek_{'near' if y < 0 else 'far'}",
        )
    for x, y, roller_name in (
        (-0.105, -0.075, "roller_neg_near"),
        (-0.105, 0.075, "roller_neg_far"),
        (0.105, -0.075, "roller_pos_near"),
        (0.105, 0.075, "roller_pos_far"),
    ):
        shuttle.visual(
            Cylinder(radius=0.025, length=0.060),
            origin=Origin(xyz=(x, y, 0.198), rpy=(pi / 2, 0.0, 0.0)),
            material=dark,
            name=roller_name,
        )
    for y in (-0.075, 0.075):
        shuttle.visual(
            Box((0.080, 0.030, 0.160)),
            origin=Origin(xyz=(0.0, y, -0.025)),
            material=safety,
            name=f"first_clevis_plate_{'near' if y < 0 else 'far'}",
        )
        shuttle.visual(
            Cylinder(radius=0.040, length=0.050),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=safety,
            name=f"first_hinge_lug_{'near' if y < 0 else 'far'}",
        )
    shuttle.visual(
        Cylinder(radius=0.016, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=dark,
        name="first_hinge_pin",
    )
    shuttle.visual(
        Box((0.13, 0.13, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark,
        name="hinge_mount_pad",
    )

    short_link = model.part("short_link")
    short_link.visual(
        Cylinder(radius=0.034, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=rail_mat,
        name="top_bushing",
    )
    short_link.visual(
        Box((0.070, 0.050, 0.270)),
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        material=arm_mat,
        name="short_tube",
    )
    short_link.visual(
        Box((0.054, 0.046, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=arm_mat,
        name="upper_web",
    )
    short_link.visual(
        Box((0.115, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=arm_mat,
        name="lower_boss",
    )
    short_link.visual(
        Box((0.090, 0.180, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.306)),
        material=arm_mat,
        name="second_crosshead",
    )
    for y, plate_name, lug_name in (
        (-0.075, "second_clevis_plate_near", "second_hinge_lug_near"),
        (0.075, "second_clevis_plate_far", "second_hinge_lug_far"),
    ):
        short_link.visual(
            Box((0.082, 0.030, 0.160)),
            origin=Origin(xyz=(0.0, y, -0.380)),
            material=arm_mat,
            name=plate_name,
        )
        short_link.visual(
            Cylinder(radius=0.040, length=0.050),
            origin=Origin(xyz=(0.0, y, -0.380), rpy=(pi / 2, 0.0, 0.0)),
            material=arm_mat,
            name=lug_name,
        )
    short_link.visual(
        Cylinder(radius=0.016, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, -0.380), rpy=(pi / 2, 0.0, 0.0)),
        material=dark,
        name="second_hinge_pin",
    )

    long_link = model.part("long_link")
    long_link.visual(
        Cylinder(radius=0.034, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=rail_mat,
        name="top_bushing",
    )
    long_link.visual(
        Box((0.070, 0.050, 0.610)),
        origin=Origin(xyz=(0.0, 0.0, -0.365)),
        material=arm_mat,
        name="long_tube",
    )
    long_link.visual(
        Box((0.054, 0.046, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=arm_mat,
        name="upper_web",
    )
    long_link.visual(
        Box((0.055, 0.055, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, -0.700)),
        material=arm_mat,
        name="plate_neck",
    )
    long_link.visual(
        Box((0.320, 0.036, 0.210)),
        origin=Origin(xyz=(0.0, 0.0, -0.805)),
        material=plate_mat,
        name="rectangular_plate",
    )

    model.articulation(
        "beam_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=shuttle,
        origin=Origin(xyz=(-0.55, 0.0, 1.7245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=1.10),
    )
    model.articulation(
        "shuttle_to_short_link",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=short_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.1, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "short_to_long_link",
        ArticulationType.REVOLUTE,
        parent=short_link,
        child=long_link,
        origin=Origin(xyz=(0.0, 0.0, -0.380)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("grounded_beam")
    shuttle = object_model.get_part("shuttle")
    short_link = object_model.get_part("short_link")
    long_link = object_model.get_part("long_link")
    shuttle_slide = object_model.get_articulation("beam_to_shuttle")
    shoulder = object_model.get_articulation("shuttle_to_short_link")
    elbow = object_model.get_articulation("short_to_long_link")

    ctx.allow_overlap(
        shuttle,
        short_link,
        elem_a="first_hinge_pin",
        elem_b="top_bushing",
        reason="The shuttle hinge pin is intentionally captured inside the short-link bushing.",
    )
    ctx.allow_overlap(
        short_link,
        long_link,
        elem_a="second_hinge_pin",
        elem_b="top_bushing",
        reason="The second hinge pin is intentionally captured inside the long-link bushing.",
    )

    ctx.expect_within(
        shuttle,
        short_link,
        axes="xz",
        inner_elem="first_hinge_pin",
        outer_elem="top_bushing",
        margin=0.002,
        name="first hinge pin sits inside the bushing bore",
    )
    ctx.expect_overlap(
        shuttle,
        short_link,
        axes="y",
        elem_a="first_hinge_pin",
        elem_b="top_bushing",
        min_overlap=0.075,
        name="first hinge pin spans the bushing",
    )
    ctx.expect_within(
        short_link,
        long_link,
        axes="xz",
        inner_elem="second_hinge_pin",
        outer_elem="top_bushing",
        margin=0.002,
        name="second hinge pin sits inside the bushing bore",
    )
    ctx.expect_overlap(
        short_link,
        long_link,
        axes="y",
        elem_a="second_hinge_pin",
        elem_b="top_bushing",
        min_overlap=0.075,
        name="second hinge pin spans the bushing",
    )

    ctx.expect_contact(
        beam,
        shuttle,
        contact_tol=0.001,
        elem_a="rail_near",
        elem_b="roller_neg_near",
        name="carriage roller bears on the fixed rail",
    )
    ctx.expect_overlap(
        shuttle,
        beam,
        axes="y",
        elem_a="upper_cover",
        elem_b="rail_near",
        min_overlap=0.010,
        name="shuttle cover is centered under the rail pair",
    )

    rest_pos = ctx.part_world_position(shuttle)
    with ctx.pose({shuttle_slide: 1.10}):
        extended_pos = ctx.part_world_position(shuttle)
        ctx.expect_contact(
            beam,
            shuttle,
            contact_tol=0.001,
            elem_a="rail_far",
            elem_b="roller_pos_far",
            name="travel-end roller still bears on the rail",
        )
        ctx.expect_gap(
            beam,
            shuttle,
            axis="x",
            min_gap=0.030,
            positive_elem="travel_stop_pos",
            negative_elem="upper_cover",
            name="shuttle stops before the positive end bumper",
        )
    ctx.check(
        "shuttle travels along the beam",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 1.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({shoulder: 0.55, elbow: 0.85}):
        ctx.expect_gap(
            shuttle,
            long_link,
            axis="z",
            min_gap=0.050,
            positive_elem="lower_cover",
            negative_elem="top_bushing",
            name="positive fold keeps the long link below the shuttle cover",
        )
        ctx.expect_gap(
            long_link,
            short_link,
            axis="y",
            min_gap=0.015,
            positive_elem="long_tube",
            negative_elem="second_hinge_lug_near",
            name="positive fold clears the near clevis cheek",
        )
        ctx.expect_gap(
            short_link,
            long_link,
            axis="y",
            min_gap=0.015,
            positive_elem="second_hinge_lug_far",
            negative_elem="long_tube",
            name="positive fold clears the far clevis cheek",
        )
    with ctx.pose({shoulder: -0.55, elbow: -0.85}):
        ctx.expect_gap(
            shuttle,
            long_link,
            axis="z",
            min_gap=0.050,
            positive_elem="lower_cover",
            negative_elem="top_bushing",
            name="negative fold keeps the long link below the shuttle cover",
        )
        ctx.expect_gap(
            short_link,
            long_link,
            axis="y",
            min_gap=0.015,
            positive_elem="second_hinge_lug_far",
            negative_elem="long_tube",
            name="negative fold clears the far clevis cheek",
        )
        ctx.expect_gap(
            long_link,
            short_link,
            axis="y",
            min_gap=0.015,
            positive_elem="long_tube",
            negative_elem="second_hinge_lug_near",
            name="negative fold clears the near clevis cheek",
        )

    return ctx.report()


object_model = build_object_model()
