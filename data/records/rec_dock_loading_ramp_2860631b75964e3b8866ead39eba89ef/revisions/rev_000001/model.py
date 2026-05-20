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
    model = ArticulatedObject(name="self_leveling_dock_leveler")

    dark_steel = Material("dark_steel", rgba=(0.11, 0.12, 0.13, 1.0))
    blue_steel = Material("blue_painted_steel", rgba=(0.05, 0.22, 0.42, 1.0))
    yellow_steel = Material("safety_yellow_steel", rgba=(0.95, 0.70, 0.05, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.62, 0.66, 0.67, 1.0))
    rubber = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    warning = Material("warning_stripe", rgba=(1.0, 0.86, 0.05, 1.0))

    # Root: a shallow steel pit frame with curb rails and the rear hinge support.
    pit_frame = model.part("pit_frame")
    pit_frame.visual(
        Box((2.78, 2.16, 0.06)),
        origin=Origin(xyz=(1.20, 0.0, 0.03)),
        material=dark_steel,
        name="pit_floor",
    )
    pit_frame.visual(
        Box((2.78, 0.08, 0.42)),
        origin=Origin(xyz=(1.20, -1.08, 0.27)),
        material=dark_steel,
        name="side_curb_0",
    )
    pit_frame.visual(
        Box((2.78, 0.08, 0.42)),
        origin=Origin(xyz=(1.20, 1.08, 0.27)),
        material=dark_steel,
        name="side_curb_1",
    )
    pit_frame.visual(
        Box((0.12, 2.16, 0.42)),
        origin=Origin(xyz=(2.62, 0.0, 0.27)),
        material=dark_steel,
        name="front_curb",
    )
    pit_frame.visual(
        Box((0.12, 2.16, 0.52)),
        origin=Origin(xyz=(-0.15, 0.0, 0.32)),
        material=dark_steel,
        name="rear_curb",
    )

    # Rear hinge knuckles are split, leaving clear spaces for platform knuckles.
    for i, y in enumerate((-0.36, 0.36)):
        pit_frame.visual(
            Box((0.07, 0.30, 0.09)),
            origin=Origin(xyz=(-0.065, y, 0.62)),
            material=dark_steel,
            name=f"rear_hinge_bracket_{i}",
        )
        pit_frame.visual(
            Cylinder(radius=0.045, length=0.30),
            origin=Origin(xyz=(0.0, y, 0.62), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"rear_frame_barrel_{i}",
        )
    pit_frame.visual(
        Cylinder(radius=0.021, length=1.84),
        origin=Origin(xyz=(0.0, 0.0, 0.62), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="rear_hinge_pin",
    )

    # Rubber dock bumpers on the front curb read as the truck-contact end.
    for i, y in enumerate((-1.00, 1.00)):
        pit_frame.visual(
            Box((0.10, 0.12, 0.24)),
            origin=Origin(xyz=(2.70, y, 0.51)),
            material=rubber,
            name=f"bumper_{i}",
        )

    # Main hinged deck.  The part frame is exactly on the rear hinge axis.
    platform = model.part("platform")
    platform.visual(
        Box((2.24, 1.84, 0.08)),
        origin=Origin(xyz=(1.20, 0.0, 0.0)),
        material=blue_steel,
        name="deck_plate",
    )
    # Raised anti-slip treads and underside stiffeners make the plate read as steel.
    for i, x in enumerate((0.35, 0.70, 1.05, 1.40, 1.75, 2.10)):
        platform.visual(
            Box((0.035, 1.64, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.046)),
            material=warning if i in (0, 5) else blue_steel,
            name=f"tread_bar_{i}",
        )
    for i, y in enumerate((-0.72, 0.72)):
        platform.visual(
            Box((2.12, 0.08, 0.10)),
            origin=Origin(xyz=(1.20, y, -0.09)),
            material=dark_steel,
            name=f"side_stringer_{i}",
        )
    for i, x in enumerate((0.70, 1.35, 2.00)):
        platform.visual(
            Box((0.07, 1.42, 0.09)),
            origin=Origin(xyz=(x, 0.0, -0.085)),
            material=dark_steel,
            name=f"cross_member_{i}",
        )

    # Platform rear hinge knuckles occupy the spaces between the frame barrels.
    for i, (y, length) in enumerate(((-0.72, 0.30), (0.0, 0.32), (0.72, 0.30))):
        platform.visual(
            Box((0.075, length, 0.07)),
            origin=Origin(xyz=(0.065, y, 0.0)),
            material=blue_steel,
            name=f"rear_hinge_leaf_{i}",
        )
        platform.visual(
            Cylinder(radius=0.043, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"rear_platform_barrel_{i}",
        )

    # Front hinge leaves and barrels for the lip plate.
    for i, y in enumerate((-0.48, 0.48)):
        platform.visual(
            Box((0.12, 0.42, 0.06)),
            origin=Origin(xyz=(2.34, y, -0.005)),
            material=blue_steel,
            name=f"front_hinge_leaf_{i}",
        )
        platform.visual(
            Cylinder(radius=0.035, length=0.42),
            origin=Origin(xyz=(2.40, y, -0.005), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"front_platform_barrel_{i}",
        )

    # Two welded square guide sleeves under the deck for the sliding support legs.
    for i, y in enumerate((-0.45, 0.45)):
        x = 1.55
        platform.visual(
            Box((0.025, 0.19, 0.26)),
            origin=Origin(xyz=(x - 0.075, y, -0.17)),
            material=dark_steel,
            name=f"sleeve_wall_outer_{i}",
        )
        platform.visual(
            Box((0.025, 0.19, 0.26)),
            origin=Origin(xyz=(x + 0.075, y, -0.17)),
            material=dark_steel,
            name=f"sleeve_wall_inner_{i}",
        )
        platform.visual(
            Box((0.17, 0.025, 0.26)),
            origin=Origin(xyz=(x, y - 0.075, -0.17)),
            material=dark_steel,
            name=f"sleeve_wall_side_a_{i}",
        )
        platform.visual(
            Box((0.17, 0.025, 0.26)),
            origin=Origin(xyz=(x, y + 0.075, -0.17)),
            material=dark_steel,
            name=f"sleeve_wall_side_b_{i}",
        )

    lip_plate = model.part("lip_plate")
    lip_plate.visual(
        Box((0.45, 1.78, 0.045)),
        origin=Origin(xyz=(0.275, 0.0, -0.006)),
        material=yellow_steel,
        name="lip_blade",
    )
    lip_plate.visual(
        Box((0.08, 1.78, 0.06)),
        origin=Origin(xyz=(0.08, 0.0, -0.012)),
        material=yellow_steel,
        name="lip_root_reinforcement",
    )
    for i, (y, length) in enumerate(((-0.80, 0.22), (0.0, 0.34), (0.80, 0.22))):
        lip_plate.visual(
            Box((0.075, length, 0.055)),
            origin=Origin(xyz=(0.045, y, -0.005)),
            material=yellow_steel,
            name=f"lip_hinge_leaf_{i}",
        )
        lip_plate.visual(
            Cylinder(radius=0.033, length=length),
            origin=Origin(xyz=(0.0, y, -0.005), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"lip_barrel_{i}",
        )

    # Sliding leg columns are centered in the fixed sleeves and can extend downward.
    for i, y in enumerate((-0.45, 0.45)):
        leg = model.part(f"support_leg_{i}")
        leg.visual(
            Box((0.125, 0.125, 0.40)),
            origin=Origin(xyz=(0.0, 0.0, -0.20)),
            material=galvanized,
            name="leg_column",
        )
        leg.visual(
            Box((0.22, 0.22, 0.035)),
            origin=Origin(xyz=(0.0, 0.0, -0.4175)),
            material=dark_steel,
            name="foot_pad",
        )
        model.articulation(
            f"platform_to_support_leg_{i}",
            ArticulationType.PRISMATIC,
            parent=platform,
            child=leg,
            origin=Origin(xyz=(1.55, y, -0.045)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=12000.0, velocity=0.12, lower=0.0, upper=0.10),
        )

    model.articulation(
        "pit_frame_to_platform",
        ArticulationType.REVOLUTE,
        parent=pit_frame,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.20, lower=-0.10, upper=0.34),
    )

    model.articulation(
        "platform_to_lip_plate",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=lip_plate,
        origin=Origin(xyz=(2.40, 0.0, -0.005)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.6, lower=-0.25, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pit_frame = object_model.get_part("pit_frame")
    platform = object_model.get_part("platform")
    lip_plate = object_model.get_part("lip_plate")
    leg_0 = object_model.get_part("support_leg_0")
    leg_1 = object_model.get_part("support_leg_1")

    platform_hinge = object_model.get_articulation("pit_frame_to_platform")
    lip_hinge = object_model.get_articulation("platform_to_lip_plate")
    leg_slide_0 = object_model.get_articulation("platform_to_support_leg_0")
    leg_slide_1 = object_model.get_articulation("platform_to_support_leg_1")

    for i in range(3):
        ctx.allow_overlap(
            pit_frame,
            platform,
            elem_a="rear_hinge_pin",
            elem_b=f"rear_platform_barrel_{i}",
            reason="The steel hinge pin is intentionally captured inside the platform hinge barrel.",
        )
        ctx.expect_within(
            pit_frame,
            platform,
            axes="xz",
            inner_elem="rear_hinge_pin",
            outer_elem=f"rear_platform_barrel_{i}",
            margin=0.0,
            name=f"rear hinge pin fits inside platform barrel {i}",
        )
        ctx.expect_overlap(
            pit_frame,
            platform,
            axes="y",
            elem_a="rear_hinge_pin",
            elem_b=f"rear_platform_barrel_{i}",
            min_overlap=0.20,
            name=f"rear hinge pin passes through platform barrel {i}",
        )

    ctx.expect_overlap(
        platform,
        pit_frame,
        axes="y",
        elem_a="deck_plate",
        elem_b="rear_hinge_pin",
        min_overlap=1.40,
        name="wide platform is carried across the rear hinge span",
    )
    ctx.expect_overlap(
        lip_plate,
        platform,
        axes="y",
        elem_a="lip_blade",
        elem_b="deck_plate",
        min_overlap=1.60,
        name="wide front lip matches deck width",
    )

    for leg, sleeve_index in ((leg_0, 0), (leg_1, 1)):
        ctx.expect_gap(
            leg,
            platform,
            axis="x",
            positive_elem="leg_column",
            negative_elem=f"sleeve_wall_outer_{sleeve_index}",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"support leg {sleeve_index} bears against outer sleeve wall",
        )
        ctx.expect_gap(
            platform,
            leg,
            axis="x",
            positive_elem=f"sleeve_wall_inner_{sleeve_index}",
            negative_elem="leg_column",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"support leg {sleeve_index} bears against inner sleeve wall",
        )
        ctx.expect_gap(
            leg,
            platform,
            axis="y",
            positive_elem="leg_column",
            negative_elem=f"sleeve_wall_side_a_{sleeve_index}",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"support leg {sleeve_index} bears against first side wall",
        )
        ctx.expect_gap(
            platform,
            leg,
            axis="y",
            positive_elem=f"sleeve_wall_side_b_{sleeve_index}",
            negative_elem="leg_column",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"support leg {sleeve_index} bears against second side wall",
        )
        ctx.expect_overlap(
            leg,
            platform,
            axes="z",
            elem_a="leg_column",
            elem_b=f"sleeve_wall_side_a_{sleeve_index}",
            min_overlap=0.20,
            name=f"support leg {sleeve_index} remains engaged in sleeve",
        )

    rest_leg_z = ctx.part_world_position(leg_0)[2]
    with ctx.pose({leg_slide_0: 0.10, leg_slide_1: 0.10}):
        extended_leg_z = ctx.part_world_position(leg_0)[2]
    ctx.check(
        "support legs extend downward on prismatic joints",
        extended_leg_z < rest_leg_z - 0.08,
        details=f"rest_z={rest_leg_z}, extended_z={extended_leg_z}",
    )

    rest_platform_aabb = ctx.part_world_aabb(platform)
    with ctx.pose({platform_hinge: 0.25}):
        raised_platform_aabb = ctx.part_world_aabb(platform)
    ctx.check(
        "rear hinge raises the platform front edge",
        rest_platform_aabb is not None
        and raised_platform_aabb is not None
        and raised_platform_aabb[1][2] > rest_platform_aabb[1][2] + 0.10,
        details=f"rest={rest_platform_aabb}, raised={raised_platform_aabb}",
    )

    rest_lip_aabb = ctx.part_world_aabb(lip_plate)
    with ctx.pose({lip_hinge: 1.0}):
        lowered_lip_aabb = ctx.part_world_aabb(lip_plate)
    ctx.check(
        "front lip rotates downward for truck-bed contact",
        rest_lip_aabb is not None
        and lowered_lip_aabb is not None
        and lowered_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.25,
        details=f"rest={rest_lip_aabb}, lowered={lowered_lip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
