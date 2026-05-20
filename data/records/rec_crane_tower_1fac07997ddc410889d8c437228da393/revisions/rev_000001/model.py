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
    model = ArticulatedObject(name="self_erecting_mini_tower_crane")

    yellow = model.material("powder_coated_yellow", rgba=(1.0, 0.76, 0.08, 1.0))
    dark = model.material("dark_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.64, 1.0))
    black = model.material("blackened_hook", rgba=(0.02, 0.018, 0.014, 1.0))
    counter = model.material("cast_counterweight", rgba=(0.15, 0.16, 0.16, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.65, 0.45, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=yellow,
        name="chassis_deck",
    )
    carriage.visual(
        Box((0.16, 0.16, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=yellow,
        name="mast_socket",
    )
    for i, x in enumerate((-0.28, 0.28)):
        carriage.visual(
            Box((0.06, 0.78, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material=yellow,
            name=f"stabilizer_beam_{i}",
        )
        for j, y in enumerate((-0.42, 0.42)):
            carriage.visual(
                Box((0.16, 0.11, 0.025)),
                origin=Origin(xyz=(x, y, 0.025)),
                material=steel,
                name=f"outrigger_pad_{i}_{j}",
            )
    for i, x in enumerate((-0.23, 0.23)):
        carriage.visual(
            Cylinder(radius=0.012, length=0.58),
            origin=Origin(xyz=(x, 0.0, 0.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"axle_{i}",
        )
        for j, y in enumerate((-0.245, 0.245)):
            carriage.visual(
                Cylinder(radius=0.055, length=0.040),
                origin=Origin(xyz=(x, y, 0.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark,
                name=f"wheel_{i}_{j}",
            )

    # Four open guide posts make a real telescoping sleeve rather than a solid
    # column; the moving mast runs through the clear middle without collision.
    for ix, x in enumerate((-0.10, 0.10)):
        for iy, y in enumerate((-0.10, 0.10)):
            carriage.visual(
                Box((0.030, 0.030, 0.67)),
                origin=Origin(xyz=(x, y, 0.415)),
                material=yellow,
                name=f"outer_sleeve_post_{ix}_{iy}",
            )
    for k, z in enumerate((0.10, 0.43, 0.75)):
        carriage.visual(
            Box((0.23, 0.030, 0.030)),
            origin=Origin(xyz=(0.0, -0.10, z)),
            material=yellow,
            name=f"sleeve_cross_x_{k}_0",
        )
        carriage.visual(
            Box((0.23, 0.030, 0.030)),
            origin=Origin(xyz=(0.0, 0.10, z)),
            material=yellow,
            name=f"sleeve_cross_x_{k}_1",
        )
        carriage.visual(
            Box((0.030, 0.23, 0.030)),
            origin=Origin(xyz=(-0.10, 0.0, z)),
            material=yellow,
            name=f"sleeve_cross_y_{k}_0",
        )
        carriage.visual(
            Box((0.030, 0.23, 0.030)),
            origin=Origin(xyz=(0.10, 0.0, z)),
            material=yellow,
            name=f"sleeve_cross_y_{k}_1",
        )

    telescoping_mast = model.part("telescoping_mast")
    for ix, x in enumerate((-0.055, 0.055)):
        for iy, y in enumerate((-0.055, 0.055)):
            telescoping_mast.visual(
                Box((0.022, 0.022, 1.55)),
                origin=Origin(xyz=(x, y, 0.325)),
                material=yellow,
                name=f"mast_post_{ix}_{iy}",
            )
    for k, z in enumerate((-0.42, -0.05, 0.32, 0.69, 1.06)):
        telescoping_mast.visual(
            Box((0.132, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, -0.055, z)),
            material=yellow,
            name=f"mast_cross_x_{k}_0",
        )
        telescoping_mast.visual(
            Box((0.132, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, 0.055, z)),
            material=yellow,
            name=f"mast_cross_x_{k}_1",
        )
        telescoping_mast.visual(
            Box((0.018, 0.132, 0.018)),
            origin=Origin(xyz=(-0.055, 0.0, z)),
            material=yellow,
            name=f"mast_cross_y_{k}_0",
        )
        telescoping_mast.visual(
            Box((0.018, 0.132, 0.018)),
            origin=Origin(xyz=(0.055, 0.0, z)),
            material=yellow,
            name=f"mast_cross_y_{k}_1",
        )
    telescoping_mast.visual(
        Cylinder(radius=0.115, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.120)),
        material=steel,
        name="slew_bearing",
    )

    jib = model.part("jib")
    jib.visual(
        Cylinder(radius=0.105, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=steel,
        name="turntable_plate",
    )
    jib.visual(
        Box((0.17, 0.17, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=yellow,
        name="slew_head",
    )
    jib.visual(
        Box((0.42, 0.055, 0.055)),
        origin=Origin(xyz=(-0.20, 0.0, 0.14)),
        material=yellow,
        name="counter_jib",
    )
    jib.visual(
        Box((0.16, 0.20, 0.13)),
        origin=Origin(xyz=(-0.43, 0.0, 0.10)),
        material=counter,
        name="counterweight",
    )
    for j, y in enumerate((-0.060, 0.060)):
        jib.visual(
            Box((1.62, 0.026, 0.026)),
            origin=Origin(xyz=(0.86, y, 0.095)),
            material=yellow,
            name=f"bottom_chord_{j}",
        )
    jib.visual(
        Box((1.48, 0.028, 0.028)),
        origin=Origin(xyz=(0.87, 0.0, 0.285)),
        material=yellow,
        name="top_chord",
    )
    for k, x in enumerate((0.14, 0.45, 0.76, 1.07, 1.38, 1.66)):
        jib.visual(
            Box((0.026, 0.158, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.095)),
            material=yellow,
            name=f"bottom_tie_{k}",
        )
        jib.visual(
            Box((0.026, 0.026, 0.205)),
            origin=Origin(xyz=(x, 0.0, 0.190)),
            material=yellow,
            name=f"web_post_{k}",
        )

    trolley_block = model.part("trolley_block")
    trolley_block.visual(
        Box((0.18, 0.19, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=steel,
        name="trolley_frame",
    )
    for i, x in enumerate((-0.045, 0.045)):
        for j, y in enumerate((-0.060, 0.060)):
            trolley_block.visual(
                Cylinder(radius=0.018, length=0.014),
                origin=Origin(xyz=(x, y, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark,
                name=f"trolley_wheel_{i}_{j}",
            )
            trolley_block.visual(
                Box((0.028, 0.014, 0.052)),
                origin=Origin(xyz=(x, y, -0.038)),
                material=steel,
                name=f"wheel_hanger_{i}_{j}",
            )
    trolley_block.visual(
        Cylinder(radius=0.004, length=0.215),
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
        material=black,
        name="hoist_cable",
    )
    trolley_block.visual(
        Box((0.080, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        material=yellow,
        name="load_block",
    )
    trolley_block.visual(
        Box((0.016, 0.016, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, -0.365)),
        material=black,
        name="hook_shank",
    )
    trolley_block.visual(
        Box((0.060, 0.016, 0.016)),
        origin=Origin(xyz=(0.022, 0.0, -0.420)),
        material=black,
        name="hook_throat",
    )
    trolley_block.visual(
        Box((0.016, 0.016, 0.050)),
        origin=Origin(xyz=(0.052, 0.0, -0.395)),
        material=black,
        name="hook_tip",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=telescoping_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.55),
    )
    model.articulation(
        "slew",
        ArticulationType.REVOLUTE,
        parent=telescoping_mast,
        child=jib,
        origin=Origin(xyz=(0.0, 0.0, 1.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.45, lower=-2.80, upper=2.80),
    )
    model.articulation(
        "trolley_slide",
        ArticulationType.PRISMATIC,
        parent=jib,
        child=trolley_block,
        origin=Origin(xyz=(0.25, 0.0, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    mast = object_model.get_part("telescoping_mast")
    jib = object_model.get_part("jib")
    trolley = object_model.get_part("trolley_block")
    mast_slide = object_model.get_articulation("mast_slide")
    slew = object_model.get_articulation("slew")
    trolley_slide = object_model.get_articulation("trolley_slide")

    ctx.expect_within(
        mast,
        carriage,
        axes="xy",
        margin=0.0,
        name="telescoping mast is centered in the base sleeve",
    )
    ctx.expect_overlap(
        mast,
        carriage,
        axes="z",
        min_overlap=0.30,
        name="collapsed mast remains deeply captured by sleeve height",
    )
    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.55}):
        ctx.expect_within(
            mast,
            carriage,
            axes="xy",
            margin=0.0,
            name="extended mast stays centered in guide sleeve",
        )
        ctx.expect_overlap(
            mast,
            carriage,
            axes="z",
            min_overlap=0.04,
            name="extended mast retains insertion in sleeve",
        )
        extended_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast slide raises the tower",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.50,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    rest_trolley_pos = ctx.part_world_position(trolley)
    with ctx.pose({trolley_slide: 1.20}):
        ctx.expect_overlap(
            trolley,
            jib,
            axes="x",
            min_overlap=0.10,
            name="trolley remains on the jib rails at the travel stop",
        )
        ctx.expect_within(
            trolley,
            jib,
            axes="y",
            margin=0.015,
            name="trolley straddles the paired bottom-chord rails",
        )
        extended_trolley_pos = ctx.part_world_position(trolley)
    ctx.check(
        "trolley slide travels outward along jib",
        rest_trolley_pos is not None
        and extended_trolley_pos is not None
        and extended_trolley_pos[0] > rest_trolley_pos[0] + 1.0,
        details=f"rest={rest_trolley_pos}, extended={extended_trolley_pos}",
    )

    with ctx.pose({slew: 1.0}):
        slewed_aabb = ctx.part_world_aabb(jib)
    straight_aabb = ctx.part_world_aabb(jib)
    if slewed_aabb is not None and straight_aabb is not None:
        straight_width_y = straight_aabb[1][1] - straight_aabb[0][1]
        slewed_width_y = slewed_aabb[1][1] - slewed_aabb[0][1]
        ctx.check(
            "slew joint swings the horizontal jib",
            slewed_width_y > straight_width_y + 0.65,
            details=f"straight_y_width={straight_width_y}, slewed_y_width={slewed_width_y}",
        )
    else:
        ctx.fail("slew joint swings the horizontal jib", "missing jib AABB")

    return ctx.report()


object_model = build_object_model()
