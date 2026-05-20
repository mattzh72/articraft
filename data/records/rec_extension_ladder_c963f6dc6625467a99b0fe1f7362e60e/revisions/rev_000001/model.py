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

LOWER_LENGTH = 2.02
LOWER_RAIL_DEPTH = 0.042
LOWER_RAIL_WIDTH = 0.026
LOWER_RAIL_Y = 0.205
LOWER_RUNG_RADIUS = 0.015
LOWER_RUNG_LENGTH = 0.392
LOWER_RUNG_Z = (0.26, 0.56, 0.86, 1.16, 1.46, 1.76)

UPPER_LENGTH = 1.86
UPPER_RAIL_DEPTH = 0.034
UPPER_RAIL_WIDTH = 0.022
UPPER_RAIL_Y = 0.171
UPPER_X_OFFSET = 0.046
UPPER_RUNG_RADIUS = 0.013
UPPER_RUNG_LENGTH = 0.338
UPPER_RUNG_Z = (0.20, 0.50, 0.80, 1.10, 1.40, 1.70)
UPPER_HOME_Z = 0.46
UPPER_TRAVEL = 0.82

LEVELER_HINGE_X = 0.046
LEVELER_HINGE_Z = 0.26
LEVELER_HINGE_Y = 0.236
LEVELER_SWEEP = 2.05


def _add_ladder_section(
    part,
    *,
    rail_depth: float,
    rail_width: float,
    rail_y: float,
    rail_length: float,
    rail_x: float,
    rung_radius: float,
    rung_length: float,
    rung_positions: tuple[float, ...],
    material: str,
    rung_material: str,
    prefix: str,
) -> None:
    for index, rail_sign in enumerate((-1.0, 1.0)):
        part.visual(
            Box((rail_depth, rail_width, rail_length)),
            origin=Origin(xyz=(rail_x, rail_sign * rail_y, rail_length / 2.0)),
            material=material,
            name=f"{prefix}_rail_{index}",
        )

    for index, z_pos in enumerate(rung_positions):
        part.visual(
            Cylinder(radius=rung_radius, length=rung_length),
            origin=Origin(xyz=(rail_x, 0.0, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rung_material,
            name=f"{prefix}_rung_{index}",
        )


def _add_leveler(part, *, material: str, foot_material: str) -> None:
    part.visual(
        Box((0.032, 0.032, 0.060)),
        origin=Origin(xyz=(0.010, 0.0, 0.030)),
        material=material,
        name="mount_block",
    )
    part.visual(
        Box((0.024, 0.022, 0.380)),
        origin=Origin(xyz=(0.010, 0.0, 0.190)),
        material=material,
        name="leg_bar",
    )
    part.visual(
        Box((0.038, 0.058, 0.030)),
        origin=Origin(xyz=(0.010, 0.0, 0.395)),
        material=foot_material,
        name="foot_pad",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[idx] + upper[idx]) / 2.0 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extension_ladder")

    model.material("aluminum", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("dark_aluminum", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    lower_section = model.part("lower_section")
    _add_ladder_section(
        lower_section,
        rail_depth=LOWER_RAIL_DEPTH,
        rail_width=LOWER_RAIL_WIDTH,
        rail_y=LOWER_RAIL_Y,
        rail_length=LOWER_LENGTH,
        rail_x=0.0,
        rung_radius=LOWER_RUNG_RADIUS,
        rung_length=LOWER_RUNG_LENGTH,
        rung_positions=LOWER_RUNG_Z,
        material="aluminum",
        rung_material="dark_aluminum",
        prefix="lower",
    )

    for index, rail_sign in enumerate((-1.0, 1.0)):
        lower_section.visual(
            Box((0.055, 0.050, 0.050)),
            origin=Origin(xyz=(0.002, rail_sign * LOWER_RAIL_Y, 0.025)),
            material="rubber",
            name=f"base_shoe_{index}",
        )
        lower_section.visual(
            Box((0.060, 0.048, 0.160)),
            origin=Origin(xyz=(0.010, rail_sign * LEVELER_HINGE_Y, 0.240)),
            material="dark_aluminum",
            name=f"side_bracket_{index}",
        )

    upper_section = model.part("upper_section")
    _add_ladder_section(
        upper_section,
        rail_depth=UPPER_RAIL_DEPTH,
        rail_width=UPPER_RAIL_WIDTH,
        rail_y=UPPER_RAIL_Y,
        rail_length=UPPER_LENGTH,
        rail_x=0.0,
        rung_radius=UPPER_RUNG_RADIUS,
        rung_length=UPPER_RUNG_LENGTH,
        rung_positions=UPPER_RUNG_Z,
        material="aluminum",
        rung_material="dark_aluminum",
        prefix="upper",
    )

    for index, rail_sign in enumerate((-1.0, 1.0)):
        upper_section.visual(
            Box((0.040, 0.014, 0.100)),
            origin=Origin(xyz=(-0.005, rail_sign * 0.185, 0.050)),
            material="rubber",
            name=f"guide_pad_{index}",
        )

    leveler_0 = model.part("leveler_0")
    _add_leveler(leveler_0, material="dark_aluminum", foot_material="rubber")

    leveler_1 = model.part("leveler_1")
    _add_leveler(leveler_1, material="dark_aluminum", foot_material="rubber")

    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_section,
        child=upper_section,
        origin=Origin(xyz=(UPPER_X_OFFSET, 0.0, UPPER_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=UPPER_TRAVEL,
            effort=180.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "lower_to_leveler_0",
        ArticulationType.REVOLUTE,
        parent=lower_section,
        child=leveler_0,
        origin=Origin(xyz=(LEVELER_HINGE_X, -LEVELER_HINGE_Y, LEVELER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LEVELER_SWEEP,
            effort=30.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "lower_to_leveler_1",
        ArticulationType.REVOLUTE,
        parent=lower_section,
        child=leveler_1,
        origin=Origin(xyz=(LEVELER_HINGE_X, LEVELER_HINGE_Y, LEVELER_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LEVELER_SWEEP,
            effort=30.0,
            velocity=1.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_section = object_model.get_part("lower_section")
    upper_section = object_model.get_part("upper_section")
    leveler_0 = object_model.get_part("leveler_0")
    leveler_1 = object_model.get_part("leveler_1")

    upper_slide = object_model.get_articulation("lower_to_upper")
    leveler_joint_0 = object_model.get_articulation("lower_to_leveler_0")
    leveler_joint_1 = object_model.get_articulation("lower_to_leveler_1")

    ctx.expect_within(
        upper_section,
        lower_section,
        axes="y",
        margin=0.015,
        name="upper section stays between lower rails",
    )
    ctx.expect_overlap(
        upper_section,
        lower_section,
        axes="z",
        min_overlap=1.45,
        name="collapsed upper section remains deeply nested",
    )

    rest_upper_pos = ctx.part_world_position(upper_section)
    with ctx.pose({upper_slide: UPPER_TRAVEL}):
        ctx.expect_within(
            upper_section,
            lower_section,
            axes="y",
            margin=0.015,
            name="extended upper section stays centered between lower rails",
        )
        ctx.expect_overlap(
            upper_section,
            lower_section,
            axes="z",
            min_overlap=0.70,
            name="extended upper section retains rail engagement",
        )
        extended_upper_pos = ctx.part_world_position(upper_section)

    ctx.check(
        "upper section extends upward",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.50,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )

    leveler_0_rest = _aabb_center(
        ctx.part_element_world_aabb(leveler_0, elem="foot_pad")
    )
    leveler_1_rest = _aabb_center(
        ctx.part_element_world_aabb(leveler_1, elem="foot_pad")
    )
    with ctx.pose(
        {
            leveler_joint_0: LEVELER_SWEEP,
            leveler_joint_1: LEVELER_SWEEP,
        }
    ):
        leveler_0_open = _aabb_center(
            ctx.part_element_world_aabb(leveler_0, elem="foot_pad")
        )
        leveler_1_open = _aabb_center(
            ctx.part_element_world_aabb(leveler_1, elem="foot_pad")
        )

    ctx.check(
        "negative-side leveler folds outward and down",
        leveler_0_rest is not None
        and leveler_0_open is not None
        and leveler_0_open[1] < leveler_0_rest[1] - 0.22
        and leveler_0_open[2] < leveler_0_rest[2] - 0.45,
        details=f"rest={leveler_0_rest}, open={leveler_0_open}",
    )
    ctx.check(
        "positive-side leveler folds outward and down",
        leveler_1_rest is not None
        and leveler_1_open is not None
        and leveler_1_open[1] > leveler_1_rest[1] + 0.22
        and leveler_1_open[2] < leveler_1_rest[2] - 0.45,
        details=f"rest={leveler_1_rest}, open={leveler_1_open}",
    )

    return ctx.report()


object_model = build_object_model()
