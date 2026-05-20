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
    model = ArticulatedObject(name="retractable_dock_bridge")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    concrete = model.material("dock_concrete", rgba=(0.43, 0.45, 0.44, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.74, 0.06, 1.0))
    black = model.material("rubber_black", rgba=(0.02, 0.02, 0.018, 1.0))

    housing = model.part("housing")
    # Dock face around a clear rectangular bridge opening.
    housing.visual(
        Box((0.16, 2.35, 0.25)),
        origin=Origin(xyz=(-0.61, 0.0, 1.085)),
        material=concrete,
        name="concrete_header",
    )
    housing.visual(
        Box((0.16, 2.35, 0.20)),
        origin=Origin(xyz=(-0.61, 0.0, 0.095)),
        material=concrete,
        name="concrete_sill",
    )
    housing.visual(
        Box((0.16, 0.31, 0.79)),
        origin=Origin(xyz=(-0.61, 1.02, 0.59)),
        material=concrete,
        name="concrete_side_0",
    )
    housing.visual(
        Box((0.16, 0.31, 0.79)),
        origin=Origin(xyz=(-0.61, -1.02, 0.59)),
        material=concrete,
        name="concrete_side_1",
    )

    # Steel cassette that surrounds the sliding platform mouth.
    housing.visual(
        Box((0.85, 1.68, 0.08)),
        origin=Origin(xyz=(-0.25, 0.0, 0.92)),
        material=dark_steel,
        name="top_hood",
    )
    housing.visual(
        Box((0.85, 1.68, 0.12)),
        origin=Origin(xyz=(-0.25, 0.0, 0.25)),
        material=dark_steel,
        name="bottom_sill",
    )
    housing.visual(
        Box((0.85, 0.08, 0.66)),
        origin=Origin(xyz=(-0.25, 0.82, 0.63)),
        material=dark_steel,
        name="side_post_0",
    )
    housing.visual(
        Box((0.85, 0.08, 0.66)),
        origin=Origin(xyz=(-0.25, -0.82, 0.63)),
        material=dark_steel,
        name="side_post_1",
    )

    # Internal prismatic rail channels.  The moving rails pass through the
    # clearances between these plates rather than through a solid block.
    housing.visual(
        Box((0.70, 1.66, 0.03)),
        origin=Origin(xyz=(-0.18, 0.0, 0.555)),
        material=dark_steel,
        name="rail_crossmember",
    )
    for index, y in enumerate((0.50, -0.50)):
        housing.visual(
            Box((0.70, 0.18, 0.025)),
            origin=Origin(xyz=(-0.18, y, 0.575)),
            material=galvanized,
            name=f"rail_channel_bottom_{index}",
        )
        housing.visual(
            Box((0.70, 0.18, 0.025)),
            origin=Origin(xyz=(-0.18, y, 0.668)),
            material=galvanized,
            name=f"rail_channel_top_{index}",
        )
        housing.visual(
            Box((0.70, 0.025, 0.12)),
            origin=Origin(xyz=(-0.18, y + 0.055, 0.62)),
            material=galvanized,
            name=f"rail_channel_side_{index}_0",
        )
        housing.visual(
            Box((0.70, 0.025, 0.12)),
            origin=Origin(xyz=(-0.18, y - 0.055, 0.62)),
            material=galvanized,
            name=f"rail_channel_side_{index}_1",
        )

    # A visible locking slot below the rear of the platform for the dropping
    # safety pin brace.
    housing.visual(
        Box((0.90, 0.22, 0.02)),
        origin=Origin(xyz=(-0.225, 0.0, 0.32)),
        material=galvanized,
        name="slot_floor",
    )
    housing.visual(
        Box((0.90, 0.035, 0.22)),
        origin=Origin(xyz=(-0.225, 0.065, 0.44)),
        material=galvanized,
        name="slot_cheek_0",
    )
    housing.visual(
        Box((0.90, 0.035, 0.22)),
        origin=Origin(xyz=(-0.225, -0.065, 0.44)),
        material=galvanized,
        name="slot_cheek_1",
    )

    platform = model.part("platform")
    platform.visual(
        Box((1.70, 1.18, 0.07)),
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        material=galvanized,
        name="deck_plate",
    )
    platform.visual(
        Box((1.70, 0.045, 0.08)),
        origin=Origin(xyz=(0.10, 0.612, 0.055)),
        material=dark_steel,
        name="side_curb_0",
    )
    platform.visual(
        Box((1.70, 0.045, 0.08)),
        origin=Origin(xyz=(0.10, -0.612, 0.055)),
        material=dark_steel,
        name="side_curb_1",
    )
    for index, x in enumerate((-0.55, -0.20, 0.15, 0.50, 0.82)):
        platform.visual(
            Box((0.018, 1.08, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.039)),
            material=dark_steel,
            name=f"tread_bar_{index}",
        )

    for index, y in enumerate((0.50, -0.50)):
        platform.visual(
            Box((2.25, 0.08, 0.06)),
            origin=Origin(xyz=(0.075, y, -0.10)),
            material=dark_steel,
            name=f"rear_rail_{index}",
        )
        platform.visual(
            Box((0.75, 0.035, 0.07)),
            origin=Origin(xyz=(0.575, y, -0.055)),
            material=dark_steel,
            name=f"rail_web_{index}",
        )

    platform.visual(
        Box((0.08, 1.08, 0.08)),
        origin=Origin(xyz=(-0.66, 0.0, -0.08)),
        material=dark_steel,
        name="rear_crossbar",
    )
    platform.visual(
        Box((0.06, 0.035, 0.06)),
        origin=Origin(xyz=(-0.60, 0.07, -0.08)),
        material=dark_steel,
        name="brace_lug_0",
    )
    platform.visual(
        Box((0.06, 0.035, 0.06)),
        origin=Origin(xyz=(-0.60, -0.07, -0.08)),
        material=dark_steel,
        name="brace_lug_1",
    )
    platform.visual(
        Box((0.07, 0.10, 0.05)),
        origin=Origin(xyz=(0.935, 0.55, -0.075)),
        material=dark_steel,
        name="front_hinge_lug_0",
    )
    platform.visual(
        Box((0.07, 0.10, 0.05)),
        origin=Origin(xyz=(0.935, -0.55, -0.075)),
        material=dark_steel,
        name="front_hinge_lug_1",
    )
    platform.visual(
        Box((0.05, 0.08, 0.055)),
        origin=Origin(xyz=(0.925, 0.55, -0.045)),
        material=dark_steel,
        name="front_hinge_leaf_0",
    )
    platform.visual(
        Box((0.05, 0.08, 0.055)),
        origin=Origin(xyz=(0.925, -0.55, -0.045)),
        material=dark_steel,
        name="front_hinge_leaf_1",
    )

    lip = model.part("front_lip")
    lip.visual(
        Box((0.36, 1.16, 0.045)),
        origin=Origin(xyz=(0.18, 0.0, 0.08)),
        material=galvanized,
        name="lip_plate",
    )
    lip.visual(
        Box((0.05, 1.10, 0.014)),
        origin=Origin(xyz=(0.335, 0.0, 0.108)),
        material=yellow,
        name="yellow_nose",
    )
    lip.visual(
        Cylinder(radius=0.022, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lip_hinge_tube",
    )
    lip.visual(
        Box((0.10, 0.92, 0.04)),
        origin=Origin(xyz=(0.05, 0.0, 0.04)),
        material=dark_steel,
        name="lip_hinge_leaf",
    )

    pin_brace = model.part("pin_brace")
    pin_brace.visual(
        Cylinder(radius=0.02, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="brace_hinge_tube",
    )
    pin_brace.visual(
        Box((0.035, 0.035, 0.29)),
        origin=Origin(xyz=(0.0, 0.0, -0.165)),
        material=yellow,
        name="pin_bar",
    )
    pin_brace.visual(
        Box((0.035, 0.04, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=yellow,
        name="pin_neck",
    )
    pin_brace.visual(
        Box((0.075, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.2925)),
        material=dark_steel,
        name="pin_foot",
    )

    model.articulation(
        "platform_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.18, lower=0.0, upper=0.75),
    )
    model.articulation(
        "front_lip_hinge",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=lip,
        origin=Origin(xyz=(0.95, 0.0, -0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=-0.15, upper=0.95),
    )
    model.articulation(
        "pin_brace_hinge",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=pin_brace,
        origin=Origin(xyz=(-0.60, 0.0, -0.08)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    platform = object_model.get_part("platform")
    lip = object_model.get_part("front_lip")
    pin_brace = object_model.get_part("pin_brace")
    slide = object_model.get_articulation("platform_slide")
    lip_hinge = object_model.get_articulation("front_lip_hinge")
    brace_hinge = object_model.get_articulation("pin_brace_hinge")

    ctx.expect_gap(
        platform,
        housing,
        axis="z",
        positive_elem="rear_rail_0",
        negative_elem="rail_channel_bottom_0",
        min_gap=0.001,
        max_gap=0.012,
        name="sliding rail clears lower channel",
    )
    ctx.expect_overlap(
        platform,
        housing,
        axes="x",
        elem_a="rear_rail_0",
        elem_b="rail_channel_bottom_0",
        min_overlap=0.25,
        name="rail remains inserted while retracted",
    )
    ctx.expect_gap(
        lip,
        platform,
        axis="x",
        positive_elem="lip_plate",
        negative_elem="deck_plate",
        max_gap=0.002,
        max_penetration=0.001,
        name="front lip is hinged at platform edge",
    )
    ctx.expect_within(
        pin_brace,
        housing,
        axes="y",
        inner_elem="pin_bar",
        outer_elem="slot_floor",
        margin=0.0,
        name="pin brace drops inside the slot width",
    )
    ctx.expect_gap(
        pin_brace,
        housing,
        axis="z",
        positive_elem="pin_foot",
        negative_elem="slot_floor",
        max_gap=0.003,
        max_penetration=0.001,
        name="pin brace foot seats on slot floor",
    )

    rest_platform_pos = ctx.part_world_position(platform)
    with ctx.pose({slide: 0.75, brace_hinge: 1.45}):
        ctx.expect_overlap(
            platform,
            housing,
            axes="x",
            elem_a="rear_rail_0",
            elem_b="rail_channel_bottom_0",
            min_overlap=0.25,
            name="extended platform keeps rail engagement",
        )
        ctx.expect_gap(
            platform,
            housing,
            axis="z",
            positive_elem="rear_rail_0",
            negative_elem="rail_channel_bottom_0",
            min_gap=0.001,
            max_gap=0.012,
            name="extended rail still clears lower channel",
        )
        extended_platform_pos = ctx.part_world_position(platform)

    with ctx.pose({slide: 0.75, brace_hinge: 0.0}):
        ctx.expect_within(
            pin_brace,
            housing,
            axes="xy",
            inner_elem="pin_foot",
            outer_elem="slot_floor",
            margin=0.0,
            name="extended brace foot is over housing slot",
        )
        ctx.expect_gap(
            pin_brace,
            housing,
            axis="z",
            positive_elem="pin_foot",
            negative_elem="slot_floor",
            max_gap=0.003,
            max_penetration=0.001,
            name="extended brace foot seats in slot",
        )

    ctx.check(
        "platform slides outward from dock face",
        rest_platform_pos is not None
        and extended_platform_pos is not None
        and extended_platform_pos[0] > rest_platform_pos[0] + 0.70,
        details=f"rest={rest_platform_pos}, extended={extended_platform_pos}",
    )

    rest_lip_aabb = ctx.part_world_aabb(lip)
    with ctx.pose({lip_hinge: 0.95}):
        dropped_lip_aabb = ctx.part_world_aabb(lip)
    ctx.check(
        "front lip rotates downward",
        rest_lip_aabb is not None
        and dropped_lip_aabb is not None
        and dropped_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.15,
        details=f"rest={rest_lip_aabb}, dropped={dropped_lip_aabb}",
    )

    rest_brace_aabb = ctx.part_world_aabb(pin_brace)
    with ctx.pose({brace_hinge: 1.45}):
        raised_brace_aabb = ctx.part_world_aabb(pin_brace)
    ctx.check(
        "pin brace raises clear of slot",
        rest_brace_aabb is not None
        and raised_brace_aabb is not None
        and raised_brace_aabb[0][2] > rest_brace_aabb[0][2] + 0.18,
        details=f"rest={rest_brace_aabb}, raised={raised_brace_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
