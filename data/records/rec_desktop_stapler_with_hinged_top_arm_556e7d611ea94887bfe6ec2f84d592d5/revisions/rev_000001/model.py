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


BASE_LENGTH = 0.162
BASE_WIDTH = 0.044
BASE_REAR_OVERHANG = 0.006
BASE_BODY_HEIGHT = 0.014
BASE_BODY_TOP_Z = -0.010
DECK_LENGTH = 0.144
DECK_WIDTH = 0.032
DECK_HEIGHT = 0.004
DECK_TOP_Z = BASE_BODY_TOP_Z + DECK_HEIGHT
TRACK_START_X = 0.024
TRACK_LENGTH = 0.098
TRACK_RAIL_WIDTH = 0.004
TRACK_RAIL_HEIGHT = 0.0015
TRACK_RAIL_Y = 0.008
BASE_EAR_RADIUS = 0.0035
BASE_EAR_LENGTH = 0.007
BASE_EAR_Y = 0.0105

MAG_WIDTH = 0.016
MAG_ROOF_LENGTH = 0.112
MAG_ROOF_HEIGHT = 0.003
MAG_SIDE_THICKNESS = 0.0025
MAG_SIDE_HEIGHT = 0.0075
MAG_BOTTOM_Z = 0.002
MAG_FRONT_LENGTH = 0.012
MAG_FRONT_HEIGHT = 0.006
MAG_REAR_LENGTH = 0.016
MAG_REAR_HEIGHT = 0.008
MAG_HINGE_RADIUS = 0.0032
MAG_HINGE_LENGTH = 0.014
MAG_ARM_HINGE_X = 0.010
MAG_ARM_HINGE_Z = 0.015
MAG_ARM_BARREL_RADIUS = 0.0027
MAG_ARM_BARREL_LENGTH = 0.008

ARM_WIDTH = 0.022
ARM_MAIN_LENGTH = 0.104
ARM_MAIN_HEIGHT = 0.006
ARM_REAR_HUMP_LENGTH = 0.032
ARM_REAR_HUMP_HEIGHT = 0.012
ARM_TOP_HUMP_RADIUS = 0.006
ARM_NOSE_RADIUS = 0.004
ARM_SIDE_BARREL_RADIUS = 0.0027
ARM_SIDE_BARREL_LENGTH = 0.006
ARM_SIDE_BARREL_Y = 0.007

STOP_START_X = 0.028
STOP_TRAVEL = 0.070
STOP_FOOT_LENGTH = 0.018
STOP_FOOT_WIDTH = 0.008
STOP_FOOT_HEIGHT = 0.002
STOP_STEM_LENGTH = 0.003
STOP_STEM_WIDTH = 0.003
STOP_STEM_HEIGHT = 0.003
STOP_FENCE_THICKNESS = 0.0015
STOP_FENCE_WIDTH = 0.009
STOP_FENCE_HEIGHT = 0.0055


def _add_y_cylinder(part, *, radius: float, length: float, xyz: tuple[float, float, float], material: str, name: str):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="half_strip_stapler")

    model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("magazine_metal", rgba=(0.25, 0.26, 0.29, 1.0))
    model.material("arm_black", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("stop_gray", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                (BASE_LENGTH / 2.0) - BASE_REAR_OVERHANG,
                0.0,
                BASE_BODY_TOP_Z - (BASE_BODY_HEIGHT / 2.0),
            )
        ),
        material="base_black",
        name="base_body",
    )
    base.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_HEIGHT)),
        origin=Origin(xyz=(0.076, 0.0, BASE_BODY_TOP_Z + (DECK_HEIGHT / 2.0) - 0.0001)),
        material="base_black",
        name="base_deck",
    )
    base.visual(
        Box((0.018, 0.028, 0.006)),
        origin=Origin(xyz=(0.145, 0.0, BASE_BODY_TOP_Z - 0.001)),
        material="base_black",
        name="front_nose",
    )
    for idx, rail_y in enumerate((-TRACK_RAIL_Y, TRACK_RAIL_Y)):
        base.visual(
            Box((TRACK_LENGTH, TRACK_RAIL_WIDTH, TRACK_RAIL_HEIGHT)),
            origin=Origin(
                xyz=(
                    TRACK_START_X + (TRACK_LENGTH / 2.0),
                    rail_y,
                    DECK_TOP_Z + (TRACK_RAIL_HEIGHT / 2.0) - 0.0002,
                )
            ),
            material="base_black",
            name=f"track_rail_{idx}",
        )
    for idx, ear_y in enumerate((-BASE_EAR_Y, BASE_EAR_Y)):
        _add_y_cylinder(
            base,
            radius=BASE_EAR_RADIUS,
            length=BASE_EAR_LENGTH,
            xyz=(0.0, ear_y, 0.0),
            material="base_black",
            name=f"base_ear_{idx}",
        )
        base.visual(
            Box((0.010, 0.003, 0.008)),
            origin=Origin(xyz=(0.002, ear_y, -0.003)),
            material="base_black",
            name=f"ear_boss_{idx}",
        )

    magazine = model.part("magazine")
    magazine.visual(
        Box((MAG_ROOF_LENGTH, MAG_WIDTH, MAG_ROOF_HEIGHT)),
        origin=Origin(xyz=(0.060, 0.0, 0.010)),
        material="magazine_metal",
        name="mag_roof",
    )
    for idx, side_y in enumerate((-0.00675, 0.00675)):
        magazine.visual(
            Box((MAG_ROOF_LENGTH, MAG_SIDE_THICKNESS, MAG_SIDE_HEIGHT)),
            origin=Origin(xyz=(0.060, side_y, MAG_BOTTOM_Z + (MAG_SIDE_HEIGHT / 2.0))),
            material="magazine_metal",
            name=f"mag_side_{idx}",
        )
    magazine.visual(
        Box((MAG_FRONT_LENGTH, 0.014, MAG_FRONT_HEIGHT)),
        origin=Origin(xyz=(0.111, 0.0, 0.005)),
        material="magazine_metal",
        name="mag_front",
    )
    magazine.visual(
        Box((MAG_REAR_LENGTH, 0.012, MAG_REAR_HEIGHT)),
        origin=Origin(xyz=(0.008, 0.0, 0.004)),
        material="magazine_metal",
        name="mag_rear",
    )
    _add_y_cylinder(
        magazine,
        radius=MAG_HINGE_RADIUS,
        length=MAG_HINGE_LENGTH,
        xyz=(0.0, 0.0, 0.0),
        material="magazine_metal",
        name="mag_hinge_barrel",
    )
    magazine.visual(
        Box((0.014, 0.010, 0.013)),
        origin=Origin(xyz=(MAG_ARM_HINGE_X, 0.0, 0.0085)),
        material="magazine_metal",
        name="arm_tower",
    )
    _add_y_cylinder(
        magazine,
        radius=MAG_ARM_BARREL_RADIUS,
        length=MAG_ARM_BARREL_LENGTH,
        xyz=(MAG_ARM_HINGE_X, 0.0, MAG_ARM_HINGE_Z),
        material="magazine_metal",
        name="arm_barrel",
    )

    top_arm = model.part("top_arm")
    top_arm.visual(
        Box((ARM_MAIN_LENGTH, ARM_WIDTH, ARM_MAIN_HEIGHT)),
        origin=Origin(xyz=(0.066, 0.0, 0.0075)),
        material="arm_black",
        name="arm_main",
    )
    top_arm.visual(
        Box((ARM_REAR_HUMP_LENGTH, ARM_WIDTH, ARM_REAR_HUMP_HEIGHT)),
        origin=Origin(xyz=(0.026, 0.0, 0.010)),
        material="arm_black",
        name="arm_rear_hump",
    )
    _add_y_cylinder(
        top_arm,
        radius=ARM_TOP_HUMP_RADIUS,
        length=ARM_WIDTH,
        xyz=(0.072, 0.0, 0.011),
        material="arm_black",
        name="arm_top_hump",
    )
    _add_y_cylinder(
        top_arm,
        radius=ARM_NOSE_RADIUS,
        length=ARM_WIDTH,
        xyz=(0.118, 0.0, 0.0065),
        material="arm_black",
        name="arm_nose",
    )
    for idx, barrel_y in enumerate((-ARM_SIDE_BARREL_Y, ARM_SIDE_BARREL_Y)):
        _add_y_cylinder(
            top_arm,
            radius=ARM_SIDE_BARREL_RADIUS,
            length=ARM_SIDE_BARREL_LENGTH,
            xyz=(0.0, barrel_y, 0.0),
            material="arm_black",
            name=f"arm_side_barrel_{idx}",
        )
        cheek_y = -0.0085 if barrel_y < 0.0 else 0.0085
        top_arm.visual(
            Box((0.018, 0.005, 0.010)),
            origin=Origin(xyz=(0.008, cheek_y, 0.005)),
            material="arm_black",
            name=f"arm_cheek_{idx}",
        )

    paper_stop = model.part("paper_stop")
    paper_stop.visual(
        Box((STOP_FOOT_LENGTH, STOP_FOOT_WIDTH, STOP_FOOT_HEIGHT)),
        origin=Origin(xyz=(STOP_FOOT_LENGTH / 2.0, 0.0, STOP_FOOT_HEIGHT / 2.0)),
        material="stop_gray",
        name="stop_foot",
    )
    paper_stop.visual(
        Box((STOP_STEM_LENGTH, STOP_STEM_WIDTH, STOP_STEM_HEIGHT)),
        origin=Origin(xyz=(STOP_FOOT_LENGTH - 0.0035, 0.0, 0.0035)),
        material="stop_gray",
        name="stop_stem",
    )
    paper_stop.visual(
        Box((STOP_FENCE_THICKNESS, STOP_FENCE_WIDTH, STOP_FENCE_HEIGHT)),
        origin=Origin(xyz=(STOP_FOOT_LENGTH - 0.001, 0.0, 0.00475)),
        material="stop_gray",
        name="stop_fence",
    )

    model.articulation(
        "magazine_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=magazine,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.85, effort=15.0, velocity=2.5),
    )
    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=magazine,
        child=top_arm,
        origin=Origin(xyz=(MAG_ARM_HINGE_X, 0.0, MAG_ARM_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=10.0, velocity=3.0),
    )
    model.articulation(
        "paper_stop_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=paper_stop,
        origin=Origin(xyz=(STOP_START_X, 0.0, DECK_TOP_Z - 0.0001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=STOP_TRAVEL, effort=4.0, velocity=0.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    magazine = object_model.get_part("magazine")
    top_arm = object_model.get_part("top_arm")
    paper_stop = object_model.get_part("paper_stop")

    magazine_hinge = object_model.get_articulation("magazine_hinge")
    arm_hinge = object_model.get_articulation("arm_hinge")
    paper_stop_slide = object_model.get_articulation("paper_stop_slide")

    mag_limits = magazine_hinge.motion_limits
    arm_limits = arm_hinge.motion_limits
    stop_limits = paper_stop_slide.motion_limits

    with ctx.pose({magazine_hinge: 0.0, arm_hinge: 0.0}):
        ctx.expect_overlap(magazine, base, axes="x", min_overlap=0.090, name="closed magazine spans the base deck")
        ctx.expect_overlap(top_arm, magazine, axes="x", min_overlap=0.090, name="closed arm covers the magazine")
        ctx.expect_within(paper_stop, base, axes="xy", margin=0.002, name="paper stop stays inside the base footprint at rest")

    if mag_limits is not None and mag_limits.upper is not None:
        with ctx.pose({magazine_hinge: 0.0, arm_hinge: 0.0}):
            closed_mag_aabb = ctx.part_world_aabb(magazine)
        with ctx.pose({magazine_hinge: mag_limits.upper, arm_hinge: 0.0}):
            open_mag_aabb = ctx.part_world_aabb(magazine)
        ctx.check(
            "magazine opens upward from the rear hinge",
            closed_mag_aabb is not None
            and open_mag_aabb is not None
            and open_mag_aabb[1][2] > closed_mag_aabb[1][2] + 0.03,
            details=f"closed={closed_mag_aabb}, open={open_mag_aabb}",
        )

    if arm_limits is not None and arm_limits.upper is not None:
        with ctx.pose({magazine_hinge: 0.35, arm_hinge: 0.0}):
            closed_arm_aabb = ctx.part_world_aabb(top_arm)
        with ctx.pose({magazine_hinge: 0.35, arm_hinge: arm_limits.upper}):
            open_arm_aabb = ctx.part_world_aabb(top_arm)
        ctx.check(
            "top arm lifts above the magazine",
            closed_arm_aabb is not None
            and open_arm_aabb is not None
            and open_arm_aabb[1][2] > closed_arm_aabb[1][2] + 0.02,
            details=f"closed={closed_arm_aabb}, open={open_arm_aabb}",
        )

    if stop_limits is not None and stop_limits.upper is not None:
        rest_pos = ctx.part_world_position(paper_stop)
        with ctx.pose({paper_stop_slide: stop_limits.upper}):
            ctx.expect_within(
                paper_stop,
                base,
                axes="xy",
                margin=0.002,
                name="paper stop remains inside the base footprint at full travel",
            )
            extended_pos = ctx.part_world_position(paper_stop)
        ctx.check(
            "paper stop slides forward along the base deck",
            rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.05,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
