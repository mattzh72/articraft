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
    model = ArticulatedObject(name="wall_monitor_arm_without_screen")

    black = Material("satin_black", color=(0.015, 0.016, 0.017, 1.0))
    dark_metal = Material("dark_powder_coated_metal", color=(0.06, 0.065, 0.07, 1.0))
    edge_metal = Material("worn_pin_edges", color=(0.42, 0.43, 0.42, 1.0))
    screw_dark = Material("recessed_screw_heads", color=(0.005, 0.005, 0.006, 1.0))

    # World frame: +X projects away from the wall, +Z is up.  The root part is
    # the fixed wall bracket and exposed wall-side pivot hardware.
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.035, 0.20, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="flat_wall_plate",
    )
    wall_plate.visual(
        Box((0.032, 0.082, 0.110)),
        origin=Origin(xyz=(0.0335, 0.0, 0.030)),
        material=dark_metal,
        name="projecting_boss",
    )
    wall_plate.visual(
        Box((0.096, 0.112, 0.014)),
        origin=Origin(xyz=(0.088, 0.0, 0.088)),
        material=dark_metal,
        name="upper_clevis_plate",
    )
    wall_plate.visual(
        Box((0.096, 0.112, 0.014)),
        origin=Origin(xyz=(0.088, 0.0, -0.028)),
        material=dark_metal,
        name="lower_clevis_plate",
    )
    wall_plate.visual(
        Cylinder(radius=0.012, length=0.135),
        origin=Origin(xyz=(0.088, 0.0, 0.030)),
        material=edge_metal,
        name="wall_pivot_pin",
    )
    for iy in (-1, 1):
        for iz in (-1, 1):
            wall_plate.visual(
                Cylinder(radius=0.016, length=0.006),
                origin=Origin(
                    xyz=(0.0195, iy * 0.067, iz * 0.116),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=screw_dark,
                name=f"wall_screw_{iy}_{iz}",
            )

    primary_link = model.part("primary_link")
    primary_len = 0.40
    primary_link.visual(
        Box((0.326, 0.052, 0.034)),
        origin=Origin(xyz=(0.201, 0.0, 0.0)),
        material=dark_metal,
        name="primary_rect_tube",
    )
    primary_link.visual(
        Cylinder(radius=0.038, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="proximal_barrel",
    )
    primary_link.visual(
        Cylinder(radius=0.036, length=0.054),
        origin=Origin(xyz=(primary_len, 0.0, 0.0)),
        material=dark_metal,
        name="distal_barrel",
    )
    primary_link.visual(
        Box((0.26, 0.010, 0.040)),
        origin=Origin(xyz=(primary_len / 2.0, 0.031, 0.0)),
        material=black,
        name="upper_cable_channel",
    )

    secondary_link = model.part("secondary_link")
    secondary_len = 0.34
    secondary_link.visual(
        Box((0.274, 0.046, 0.032)),
        origin=Origin(xyz=(0.171, 0.0, 0.0)),
        material=dark_metal,
        name="secondary_rect_tube",
    )
    secondary_link.visual(
        Cylinder(radius=0.034, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="proximal_barrel",
    )
    secondary_link.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(xyz=(secondary_len, 0.0, 0.0)),
        material=dark_metal,
        name="distal_barrel",
    )
    secondary_link.visual(
        Box((0.22, 0.009, 0.036)),
        origin=Origin(xyz=(secondary_len / 2.0, -0.027, 0.0)),
        material=black,
        name="lower_cable_channel",
    )

    swivel_head = model.part("swivel_head")
    swivel_head.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="swivel_turntable",
    )
    swivel_head.visual(
        Box((0.014, 0.038, 0.038)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=dark_metal,
        name="swivel_neck",
    )
    swivel_head.visual(
        Box((0.042, 0.088, 0.060)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=dark_metal,
        name="compact_head_block",
    )
    swivel_head.visual(
        Box((0.066, 0.016, 0.092)),
        origin=Origin(xyz=(0.094, 0.050, 0.0)),
        material=dark_metal,
        name="yoke_cheek_0",
    )
    swivel_head.visual(
        Box((0.066, 0.016, 0.092)),
        origin=Origin(xyz=(0.094, -0.050, 0.0)),
        material=dark_metal,
        name="yoke_cheek_1",
    )

    mount_frame = model.part("mount_frame")
    mount_frame.visual(
        Cylinder(radius=0.012, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=edge_metal,
        name="tilt_axle",
    )
    mount_frame.visual(
        Box((0.055, 0.086, 0.060)),
        origin=Origin(xyz=(0.0225, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_boss",
    )
    mount_frame.visual(
        Box((0.030, 0.024, 0.024)),
        origin=Origin(xyz=(0.049, 0.0, 0.0)),
        material=dark_metal,
        name="boss_to_frame_neck",
    )
    mount_frame.visual(
        Box((0.018, 0.220, 0.018)),
        origin=Origin(xyz=(0.066, 0.0, 0.072)),
        material=black,
        name="top_bar",
    )
    mount_frame.visual(
        Box((0.018, 0.220, 0.018)),
        origin=Origin(xyz=(0.066, 0.0, -0.072)),
        material=black,
        name="bottom_bar",
    )
    mount_frame.visual(
        Box((0.018, 0.018, 0.162)),
        origin=Origin(xyz=(0.066, 0.101, 0.0)),
        material=black,
        name="side_bar_0",
    )
    mount_frame.visual(
        Box((0.018, 0.018, 0.162)),
        origin=Origin(xyz=(0.066, -0.101, 0.0)),
        material=black,
        name="side_bar_1",
    )
    mount_frame.visual(
        Box((0.016, 0.185, 0.012)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=black,
        name="center_crossbar",
    )
    mount_frame.visual(
        Box((0.016, 0.012, 0.145)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=black,
        name="vertical_crossbar",
    )
    for iy in (-1, 1):
        for iz in (-1, 1):
            mount_frame.visual(
                Cylinder(radius=0.008, length=0.005),
                origin=Origin(
                    xyz=(0.075, iy * 0.075, iz * 0.072),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=screw_dark,
                name=f"vesa_hole_{iy}_{iz}",
            )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=primary_link,
        origin=Origin(xyz=(0.088, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-1.65, upper=1.65),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(primary_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-2.45, upper=2.45),
    )
    model.articulation(
        "secondary_to_head",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=swivel_head,
        origin=Origin(xyz=(secondary_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "head_to_frame",
        ArticulationType.REVOLUTE,
        parent=swivel_head,
        child=mount_frame,
        origin=Origin(xyz=(0.094, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=math.radians(-18.0),
            upper=math.radians(18.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_plate = object_model.get_part("wall_plate")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    swivel_head = object_model.get_part("swivel_head")
    mount_frame = object_model.get_part("mount_frame")

    wall_to_primary = object_model.get_articulation("wall_to_primary")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary")
    secondary_to_head = object_model.get_articulation("secondary_to_head")
    head_to_frame = object_model.get_articulation("head_to_frame")

    ctx.allow_overlap(
        wall_plate,
        primary_link,
        elem_a="wall_pivot_pin",
        elem_b="proximal_barrel",
        reason="The fixed wall pin is intentionally captured inside the first hinge barrel.",
    )
    ctx.allow_overlap(
        primary_link,
        secondary_link,
        elem_a="distal_barrel",
        elem_b="proximal_barrel",
        reason="The coaxial elbow hinge barrels are represented as interleaved around a shared vertical pin.",
    )
    ctx.allow_overlap(
        secondary_link,
        swivel_head,
        elem_a="distal_barrel",
        elem_b="swivel_turntable",
        reason="The head swivel stem nests in the end bushing of the secondary link.",
    )
    ctx.allow_overlap(
        secondary_link,
        swivel_head,
        elem_a="distal_barrel",
        elem_b="swivel_neck",
        reason="The compact swivel neck is intentionally seated through the secondary-link end bushing.",
    )
    ctx.allow_overlap(
        swivel_head,
        mount_frame,
        elem_a="yoke_cheek_0",
        elem_b="tilt_axle",
        reason="The horizontal tilt axle passes through the yoke cheek bearing.",
    )
    ctx.allow_overlap(
        swivel_head,
        mount_frame,
        elem_a="yoke_cheek_1",
        elem_b="tilt_axle",
        reason="The horizontal tilt axle passes through the opposite yoke cheek bearing.",
    )

    ctx.expect_overlap(
        wall_plate,
        primary_link,
        axes="z",
        elem_a="wall_pivot_pin",
        elem_b="proximal_barrel",
        min_overlap=0.045,
        name="wall pivot pin spans the primary barrel",
    )
    ctx.expect_overlap(
        primary_link,
        secondary_link,
        axes="z",
        elem_a="distal_barrel",
        elem_b="proximal_barrel",
        min_overlap=0.045,
        name="elbow barrels share a vertical pin stack",
    )
    ctx.expect_overlap(
        secondary_link,
        swivel_head,
        axes="z",
        elem_a="distal_barrel",
        elem_b="swivel_turntable",
        min_overlap=0.045,
        name="head swivel remains captured vertically",
    )
    ctx.expect_overlap(
        secondary_link,
        swivel_head,
        axes="x",
        elem_a="distal_barrel",
        elem_b="swivel_neck",
        min_overlap=0.006,
        name="swivel neck stays seated in the end bushing",
    )
    ctx.expect_overlap(
        swivel_head,
        mount_frame,
        axes="y",
        elem_a="yoke_cheek_0",
        elem_b="tilt_axle",
        min_overlap=0.010,
        name="tilt axle crosses one yoke cheek",
    )
    ctx.expect_overlap(
        swivel_head,
        mount_frame,
        axes="y",
        elem_a="yoke_cheek_1",
        elem_b="tilt_axle",
        min_overlap=0.010,
        name="tilt axle crosses the other yoke cheek",
    )

    ctx.check(
        "tilt limit is roughly eighteen degrees each way",
        math.radians(15.0) <= abs(head_to_frame.motion_limits.lower) <= math.radians(20.0)
        and math.radians(15.0) <= abs(head_to_frame.motion_limits.upper) <= math.radians(20.0),
        details=str(head_to_frame.motion_limits),
    )

    rest_secondary_origin = ctx.part_world_position(secondary_link)
    with ctx.pose({wall_to_primary: 1.0}):
        folded_secondary_origin = ctx.part_world_position(secondary_link)
    ctx.check(
        "first vertical joint swings the downstream arm sideways",
        rest_secondary_origin is not None
        and folded_secondary_origin is not None
        and folded_secondary_origin[1] > rest_secondary_origin[1] + 0.25,
        details=f"rest={rest_secondary_origin}, folded={folded_secondary_origin}",
    )

    rest_head_origin = ctx.part_world_position(swivel_head)
    with ctx.pose({primary_to_secondary: -1.0}):
        folded_head_origin = ctx.part_world_position(swivel_head)
    ctx.check(
        "elbow vertical joint folds the second link",
        rest_head_origin is not None
        and folded_head_origin is not None
        and folded_head_origin[1] < rest_head_origin[1] - 0.20,
        details=f"rest={rest_head_origin}, folded={folded_head_origin}",
    )

    ctx.check(
        "head has vertical swivel articulation",
        tuple(round(v, 3) for v in secondary_to_head.axis) == (0.0, 0.0, 1.0),
        details=str(secondary_to_head.axis),
    )

    return ctx.report()


object_model = build_object_model()
