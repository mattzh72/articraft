from __future__ import annotations

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
    model = ArticulatedObject(name="compact_telescoping_drawer_slide")

    zinc = model.material("zinc_plated_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.28, 0.30, 0.31, 1.0))
    black = model.material("black_stop_plate", rgba=(0.02, 0.02, 0.018, 1.0))

    length = 0.340
    width = 0.055
    height = 0.030
    wall_t = 0.004
    lip_w = 0.012

    outer_channel = model.part("outer_channel")
    outer_channel.visual(
        Box((length, width, wall_t)),
        origin=Origin(xyz=(length / 2.0, 0.0, wall_t / 2.0)),
        material=zinc,
        name="bottom_web",
    )
    outer_channel.visual(
        Box((length, wall_t, height)),
        origin=Origin(xyz=(length / 2.0, -(width / 2.0 - wall_t / 2.0), height / 2.0)),
        material=zinc,
        name="side_wall_0",
    )
    outer_channel.visual(
        Box((length, lip_w, wall_t)),
        origin=Origin(
            xyz=(length / 2.0, -(width / 2.0 - wall_t - lip_w / 2.0), height - wall_t / 2.0)
        ),
        material=zinc,
        name="top_lip_0",
    )
    outer_channel.visual(
        Box((length, wall_t, height)),
        origin=Origin(xyz=(length / 2.0, width / 2.0 - wall_t / 2.0, height / 2.0)),
        material=zinc,
        name="side_wall_1",
    )
    outer_channel.visual(
        Box((length, lip_w, wall_t)),
        origin=Origin(xyz=(length / 2.0, width / 2.0 - wall_t - lip_w / 2.0, height - wall_t / 2.0)),
        material=zinc,
        name="top_lip_1",
    )
    outer_channel.visual(
        Box((0.006, width, height)),
        origin=Origin(xyz=(0.003, 0.0, height / 2.0)),
        material=zinc,
        name="rear_stop",
    )
    for idx, x in enumerate((0.090, 0.250)):
        outer_channel.visual(
            Cylinder(radius=0.006, length=0.002),
            origin=Origin(xyz=(x, 0.0, wall_t + 0.001)),
            material=dark_steel,
            name=f"mount_screw_{idx}",
        )

    inner_length = 0.286
    plate_t = 0.006
    joint_x = length - inner_length
    rail_bottom = 0.007
    rail_h = 0.016
    rail_center_z = rail_bottom + rail_h / 2.0
    flange_h = 0.004
    flange_center_z = 0.023

    inner_section = model.part("inner_section")
    inner_section.visual(
        Box((inner_length, 0.022, rail_h)),
        origin=Origin(xyz=(inner_length / 2.0, 0.0, rail_center_z)),
        material=dark_steel,
        name="rail_body",
    )
    inner_section.visual(
        Box((inner_length, 0.011, flange_h)),
        origin=Origin(xyz=(inner_length / 2.0, -0.0155, flange_center_z)),
        material=dark_steel,
        name="flange_0",
    )
    inner_section.visual(
        Box((inner_length, 0.011, flange_h)),
        origin=Origin(xyz=(inner_length / 2.0, 0.0155, flange_center_z)),
        material=dark_steel,
        name="flange_1",
    )
    inner_section.visual(
        Box((plate_t, 0.050, 0.030)),
        origin=Origin(xyz=(inner_length + plate_t / 2.0, 0.0, height / 2.0)),
        material=black,
        name="end_plate",
    )

    model.articulation(
        "channel_to_section",
        ArticulationType.PRISMATIC,
        parent=outer_channel,
        child=inner_section,
        origin=Origin(xyz=(joint_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_channel = object_model.get_part("outer_channel")
    inner_section = object_model.get_part("inner_section")
    slide = object_model.get_articulation("channel_to_section")

    ctx.expect_within(
        inner_section,
        outer_channel,
        axes="y",
        inner_elem="rail_body",
        outer_elem="bottom_web",
        margin=0.0,
        name="rail body is centered between channel walls",
    )
    ctx.expect_gap(
        inner_section,
        outer_channel,
        axis="z",
        positive_elem="rail_body",
        negative_elem="bottom_web",
        min_gap=0.002,
        max_gap=0.004,
        name="rail rides above the web with running clearance",
    )
    ctx.expect_gap(
        outer_channel,
        inner_section,
        axis="z",
        positive_elem="top_lip_0",
        negative_elem="flange_0",
        min_gap=0.0005,
        max_gap=0.002,
        name="capture flange clears the retaining lip",
    )
    ctx.expect_overlap(
        inner_section,
        outer_channel,
        axes="xy",
        elem_a="flange_0",
        elem_b="top_lip_0",
        min_overlap=0.006,
        name="flange sits under retaining lip footprint",
    )
    ctx.expect_overlap(
        inner_section,
        outer_channel,
        axes="x",
        elem_a="rail_body",
        elem_b="bottom_web",
        min_overlap=0.280,
        name="collapsed inner rail remains housed in channel",
    )

    rest_pos = ctx.part_world_position(inner_section)
    with ctx.pose({slide: 0.160}):
        ctx.expect_overlap(
            inner_section,
            outer_channel,
            axes="x",
            elem_a="rail_body",
            elem_b="bottom_web",
            min_overlap=0.120,
            name="extended rail keeps retained insertion",
        )
        ctx.expect_within(
            inner_section,
            outer_channel,
            axes="y",
            inner_elem="rail_body",
            outer_elem="bottom_web",
            margin=0.0,
            name="extended rail stays laterally captured",
        )
        extended_pos = ctx.part_world_position(inner_section)

    ctx.check(
        "prismatic section moves outward along slide",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.150,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
