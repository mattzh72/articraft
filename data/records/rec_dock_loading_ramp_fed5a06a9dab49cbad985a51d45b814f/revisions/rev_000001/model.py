from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="pit_style_dock_leveler")

    concrete = Material("cast_concrete", color=(0.46, 0.47, 0.45, 1.0))
    dark_pocket = Material("shadowed_recess", color=(0.04, 0.045, 0.045, 1.0))
    dark_steel = Material("dark_blued_steel", color=(0.08, 0.09, 0.10, 1.0))
    deck_steel = Material("painted_blue_steel", color=(0.10, 0.19, 0.30, 1.0))
    worn_steel = Material("worn_bright_steel", color=(0.55, 0.56, 0.54, 1.0))
    hazard_yellow = Material("safety_yellow", color=(0.95, 0.72, 0.06, 1.0))
    rubber = Material("black_rubber", color=(0.015, 0.014, 0.012, 1.0))

    # Root part: the fixed concrete pit, dock face, steel rim, and the
    # full-width rear axle carried by bearings recessed in the rear wall.
    pit = model.part("pit")
    pit.visual(
        Box((2.75, 2.90, 0.08)),
        origin=Origin(xyz=(-0.05, 0.0, 0.04)),
        material=concrete,
        name="pit_floor",
    )
    pit.visual(
        Box((2.75, 0.16, 0.64)),
        origin=Origin(xyz=(-0.05, 1.45, 0.32)),
        material=concrete,
        name="side_wall_0",
    )
    pit.visual(
        Box((2.75, 0.16, 0.64)),
        origin=Origin(xyz=(-0.05, -1.45, 0.32)),
        material=concrete,
        name="side_wall_1",
    )
    pit.visual(
        Box((0.30, 2.90, 0.64)),
        origin=Origin(xyz=(-1.30, 0.0, 0.32)),
        material=concrete,
        name="rear_wall",
    )
    pit.visual(
        Box((0.18, 2.90, 0.42)),
        origin=Origin(xyz=(1.21, 0.0, 0.21)),
        material=concrete,
        name="front_dock_face",
    )
    pit.visual(
        Box((2.35, 0.045, 0.045)),
        origin=Origin(xyz=(-0.02, 1.305, 0.662)),
        material=dark_steel,
        name="side_frame_0",
    )
    pit.visual(
        Box((2.35, 0.045, 0.045)),
        origin=Origin(xyz=(-0.02, -1.305, 0.662)),
        material=dark_steel,
        name="side_frame_1",
    )
    pit.visual(
        Box((0.012, 2.55, 0.18)),
        origin=Origin(xyz=(-1.151, 0.0, 0.59)),
        material=dark_pocket,
        name="rear_axle_recess",
    )
    pit.visual(
        Cylinder(radius=0.035, length=2.60),
        origin=Origin(xyz=(-1.09, 0.0, 0.59), rpy=(pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="rear_axle",
    )
    for idx, y in enumerate((-1.25, 1.25)):
        pit.visual(
            Box((0.095, 0.18, 0.16)),
            origin=Origin(xyz=(-1.118, y, 0.59)),
            material=dark_steel,
            name=f"rear_bearing_{idx}",
        )
    for idx, y in enumerate((-1.04, 1.04)):
        pit.visual(
            Box((0.10, 0.22, 0.46)),
            origin=Origin(xyz=(1.345, y, 0.29)),
            material=rubber,
            name=f"front_bumper_{idx}",
        )

    # Moving steel deck.  The child frame is exactly on the rear hinge line; the
    # main tread plate extends forward in +X and stays inside the pit opening.
    platform = model.part("platform")
    platform.visual(
        Cylinder(radius=0.055, length=2.18),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_hinge_sleeve",
    )
    platform.visual(
        Box((2.065, 2.24, 0.10)),
        origin=Origin(xyz=(1.0875, 0.0, 0.0)),
        material=deck_steel,
        name="deck_plate",
    )
    for idx, x in enumerate((0.32, 0.58, 0.84, 1.10, 1.36, 1.62, 1.88)):
        platform.visual(
            Box((0.035, 2.05, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.057)),
            material=worn_steel,
            name=f"traction_bar_{idx}",
        )
    for idx, y in enumerate((-0.62, 0.62)):
        platform.visual(
            Box((1.72, 0.09, 0.08)),
            origin=Origin(xyz=(1.05, y, -0.09)),
            material=dark_steel,
            name=f"under_rib_{idx}",
        )
    # Front hinge axle is slightly ahead of the deck plate and connected by
    # cheek brackets, matching the exposed lip hinge at the front edge.
    platform.visual(
        Cylinder(radius=0.035, length=2.24),
        origin=Origin(xyz=(2.17, 0.0, -0.005), rpy=(pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="front_lip_axle",
    )
    for idx, y in enumerate((-1.12, 1.12)):
        platform.visual(
            Box((0.16, 0.10, 0.09)),
            origin=Origin(xyz=(2.095, y, -0.005)),
            material=dark_steel,
            name=f"front_cheek_{idx}",
        )

    platform_joint = model.articulation(
        "pit_to_platform",
        ArticulationType.REVOLUTE,
        parent=pit,
        child=platform,
        origin=Origin(xyz=(-1.09, 0.0, 0.59)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=0.35, lower=-0.30, upper=0.35),
    )

    # Pivoting bridge lip.  At q=0 it is deployed nearly flush with the deck;
    # negative lip motion folds it down for storage.
    lip = model.part("lip")
    lip.visual(
        Cylinder(radius=0.045, length=2.08),
        origin=Origin(xyz=(0.0, 0.0, -0.005), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lip_hinge_sleeve",
    )
    lip.visual(
        Box((0.48, 2.16, 0.08)),
        origin=Origin(xyz=(0.285, 0.0, 0.010)),
        material=deck_steel,
        name="lip_plate",
    )
    lip.visual(
        Box((0.026, 2.12, 0.018)),
        origin=Origin(xyz=(0.524, 0.0, 0.058)),
        material=hazard_yellow,
        name="front_safety_edge",
    )
    lip.visual(
        Box((0.36, 1.86, 0.012)),
        origin=Origin(xyz=(0.305, 0.0, 0.056)),
        material=worn_steel,
        name="scuffed_lip_tread",
    )

    model.articulation(
        "platform_to_lip",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=lip,
        origin=Origin(xyz=(2.17, 0.0, -0.005)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.8, lower=-1.55, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pit = object_model.get_part("pit")
    platform = object_model.get_part("platform")
    lip = object_model.get_part("lip")
    platform_joint = object_model.get_articulation("pit_to_platform")
    lip_joint = object_model.get_articulation("platform_to_lip")

    ctx.allow_overlap(
        pit,
        platform,
        elem_a="rear_axle",
        elem_b="rear_hinge_sleeve",
        reason="The rear axle is intentionally captured inside the platform hinge sleeve.",
    )
    ctx.allow_overlap(
        platform,
        lip,
        elem_a="front_lip_axle",
        elem_b="lip_hinge_sleeve",
        reason="The front lip axle is intentionally captured inside the lip hinge sleeve.",
    )

    ctx.expect_within(
        pit,
        platform,
        axes="xz",
        inner_elem="rear_axle",
        outer_elem="rear_hinge_sleeve",
        margin=0.002,
        name="rear axle sits inside platform sleeve",
    )
    ctx.expect_overlap(
        pit,
        platform,
        axes="y",
        min_overlap=2.0,
        elem_a="rear_axle",
        elem_b="rear_hinge_sleeve",
        name="rear hinge is nearly full width",
    )
    ctx.expect_within(
        platform,
        lip,
        axes="xz",
        inner_elem="front_lip_axle",
        outer_elem="lip_hinge_sleeve",
        margin=0.002,
        name="front axle sits inside lip sleeve",
    )
    ctx.expect_overlap(
        platform,
        lip,
        axes="y",
        min_overlap=1.9,
        elem_a="front_lip_axle",
        elem_b="lip_hinge_sleeve",
        name="lip hinge spans most of the deck width",
    )
    ctx.expect_gap(
        lip,
        platform,
        axis="x",
        min_gap=0.0,
        max_gap=0.11,
        positive_elem="lip_plate",
        negative_elem="deck_plate",
        name="deployed lip begins just beyond platform front edge",
    )
    ctx.expect_overlap(
        lip,
        platform,
        axes="y",
        min_overlap=2.0,
        elem_a="lip_plate",
        elem_b="deck_plate",
        name="lip is as wide as the platform",
    )

    rest_platform_aabb = ctx.part_element_world_aabb(platform, elem="deck_plate")
    with ctx.pose({platform_joint: 0.25}):
        raised_platform_aabb = ctx.part_element_world_aabb(platform, elem="deck_plate")
    ctx.check(
        "platform hinge raises the front of the deck",
        rest_platform_aabb is not None
        and raised_platform_aabb is not None
        and raised_platform_aabb[1][2] > rest_platform_aabb[1][2] + 0.18,
        details=f"rest={rest_platform_aabb}, raised={raised_platform_aabb}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_plate")
    with ctx.pose({lip_joint: -1.20}):
        folded_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_plate")
    ctx.check(
        "negative lip motion folds the bridge plate downward",
        rest_lip_aabb is not None
        and folded_lip_aabb is not None
        and folded_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.30,
        details=f"rest={rest_lip_aabb}, folded={folded_lip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
